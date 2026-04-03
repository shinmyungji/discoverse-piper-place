import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import os
import shutil  # 원본 코드 유지
import argparse
import math
import multiprocessing as mp  # 원본 코드 유지

# =========================
# [MOD-RPC-IMPORT] (유지)
# =========================
import zmq
import msgpack
import cv2

import discoverse
from discoverse.envs import make_env
from discoverse.robots_env.piper_base import PiperCfg
from discoverse import DISCOVERSE_ROOT_DIR, DISCOVERSE_ASSETS_DIR
from discoverse.utils import get_body_tmat, get_site_tmat, step_func, SimpleStateMachine
from discoverse.task_base import AirbotPlayTaskBase, recoder_airbot_play, copypy2
from discoverse.task_base.airbot_task_base import PyavImageEncoder
from discoverse.robots.piper_ik import PiperIK


class SimNode(AirbotPlayTaskBase):

    def __init__(self, config: PiperCfg):
        super().__init__(config)

    def domain_randomization(self):
        # [MOD-RAND-0] ✅ 이 함수만 "랜덤화"로 변경한다.
        # [MOD-RAND-0] ❗주의: 아래 랜덤화 외에는 절대로 다른 로직을 건드리지 않는다.
        #
        # [MOD-RAND-1] 랜덤화 대상:
        #   - block_green (큐브)
        #   - bowl_pink   (그릇)
        #
        # [MOD-RAND-2] 랜덤화 방식(중요):
        #   - reset마다 "+="로 누적 이동시키면 drift(점점 멀리 이동) 발생 가능 → 실패율 증가
        #   - 그래서 "초기(base) 위치"를 1번 저장해두고,
        #     매 reset마다 base + uniform_offset 으로 "절대 위치"를 다시 세팅한다.
        #   - 결과적으로 항상 같은 영역 근처에서만 랜덤화됨(안전).
        #
        # [MOD-RAND-3] 무엇을 유지하나:
        #   - z(높이) 유지: 테이블 위 높이 깨짐 방지
        #   - quat(회전) 유지: 물체가 회전/뒤집히는 것 방지
        #
        # [MOD-RAND-4] 랜덤 범위(처음엔 보수적으로):
        #   - block: x,y 각각 ±8cm
        #   - bowl : x,y 각각 ±6cm
        #   ※ 성공률 유지가 우선이면 작게 시작(예: 0.03~0.05), 잘 되면 늘리기.
        #
        # [MOD-RAND-5] 최소 거리 조건:
        #   - block과 bowl이 너무 붙으면 충돌/비정상 접촉으로 실패가 많아질 수 있어
        #     최소 거리(min_dist)를 둔다.
        #   - 필요 없으면 min_dist = 0.0 으로 두면 된다.

        block_xy_range = 0.08  # [m] block x,y 각각 ±8cm
        bowl_xy_range  = 0.06  # [m] bowl  x,y 각각 ±6cm
        min_dist       = 0.10  # [m] block-bowl 최소 거리 10cm (원치 않으면 0.0)

        model = self.mj_model
        data = self.mj_data

        # [MOD-RAND-6] 특정 body가 "freejoint"인지 찾아서 qpos 인덱스를 얻는 함수
        #   - freejoint이면 qpos에 [x,y,z,qw,qx,qy,qz]가 들어감
        #   - 우리는 x,y만 수정할 것
        def _get_freejoint_qposadr_and_dofadr(body_name: str):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0:
                raise RuntimeError(f"[MOD-RAND] body not found: {body_name}")

            jntnum = int(model.body_jntnum[body_id])
            jntadr = int(model.body_jntadr[body_id])

            # body에 joint가 여러 개 달릴 수 있으니, 그 중 FREE joint만 찾는다.
            for k in range(jntnum):
                j_id = jntadr + k
                if int(model.jnt_type[j_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
                    qposadr = int(model.jnt_qposadr[j_id])  # qpos 시작 위치
                    dofadr  = int(model.jnt_dofadr[j_id])   # qvel 시작 위치(6DoF)
                    return qposadr, dofadr

            # freejoint가 아니면 None 반환
            return None, None

        # [MOD-RAND-7] freejoint가 아닌 경우 fallback(가능하면 피하지만 안전장치):
        #   - model.body_pos를 바꾸면 "모델 자체"가 바뀌는 방식이라 권장되진 않음.
        #   - 하지만 어떤 XML이 freejoint가 아닌 fixed body로 박혀있으면 qpos로 못 바꿔서
        #     최소한 x,y라도 바꾸기 위한 안전장치로 둔다.
        def _fallback_set_bodypos_xy(body_name: str, new_xy: np.ndarray):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0:
                raise RuntimeError(f"[MOD-RAND] body not found: {body_name}")
            model.body_pos[body_id][0] = float(new_xy[0])
            model.body_pos[body_id][1] = float(new_xy[1])

        # [MOD-RAND-8] base 위치를 1회 저장(drfit 방지 핵심)
        #   - "현재 reset 시점의 위치"를 base로 저장해두고,
        #     매 reset마다 base ± range로 새 절대 위치를 만든다.
        if not hasattr(self, "_rand_base_xy"):
            # 가능하면 freejoint qpos에서 base를 읽고,
            # 아니면 body tmat에서 base를 읽는다.
            self._rand_base_xy = {}

            # block base
            b_qposadr, _ = _get_freejoint_qposadr_and_dofadr("block_green")
            if b_qposadr is not None:
                self._rand_base_xy["block_green"] = np.array([data.qpos[b_qposadr + 0],
                                                              data.qpos[b_qposadr + 1]], dtype=np.float32)
            else:
                tmat_block0 = get_body_tmat(data, "block_green")
                self._rand_base_xy["block_green"] = tmat_block0[:2, 3].astype(np.float32).copy()

            # bowl base
            w_qposadr, _ = _get_freejoint_qposadr_and_dofadr("bowl_pink")
            if w_qposadr is not None:
                self._rand_base_xy["bowl_pink"] = np.array([data.qpos[w_qposadr + 0],
                                                            data.qpos[w_qposadr + 1]], dtype=np.float32)
            else:
                tmat_bowl0 = get_body_tmat(data, "bowl_pink")
                self._rand_base_xy["bowl_pink"] = tmat_bowl0[:2, 3].astype(np.float32).copy()

        base_block_xy = self._rand_base_xy["block_green"]
        base_bowl_xy  = self._rand_base_xy["bowl_pink"]

        # [MOD-RAND-9] reject sampling: min_dist 만족할 때까지 샘플링
        #   - 너무 붙게 나오면 다시 뽑는다.
        #   - 무한루프 방지 위해 최대 200회 시도 후 마지막 샘플 사용.
        rng = np.random.default_rng()
        block_xy = None
        bowl_xy = None

        for _ in range(200):
            block_xy_try = base_block_xy + rng.uniform(
                low=[-block_xy_range, -block_xy_range],
                high=[+block_xy_range, +block_xy_range],
            ).astype(np.float32)

            bowl_xy_try = base_bowl_xy + rng.uniform(
                low=[-bowl_xy_range, -bowl_xy_range],
                high=[+bowl_xy_range, +bowl_xy_range],
            ).astype(np.float32)

            if min_dist <= 0.0 or np.linalg.norm(block_xy_try - bowl_xy_try) >= min_dist:
                block_xy = block_xy_try
                bowl_xy = bowl_xy_try
                break

        if block_xy is None or bowl_xy is None:
            block_xy = block_xy_try
            bowl_xy = bowl_xy_try

        # [MOD-RAND-10] 실제 적용:
        #   - freejoint면 qpos[x,y]만 바꾸고 z/quaternion은 그대로 둔다.
        #   - qvel(속도)도 0으로 만들어 reset 직후 튀는 현상 방지.
        def _apply_xy(body_name: str, new_xy: np.ndarray):
            qposadr, dofadr = _get_freejoint_qposadr_and_dofadr(body_name)
            if qposadr is not None:
                data.qpos[qposadr + 0] = float(new_xy[0])
                data.qpos[qposadr + 1] = float(new_xy[1])
                # 속도 제거(6DoF)
                if dofadr is not None:
                    data.qvel[dofadr:dofadr + 6] = 0.0
            else:
                _fallback_set_bodypos_xy(body_name, new_xy)

        _apply_xy("block_green", block_xy)
        _apply_xy("bowl_pink", bowl_xy)

        # [MOD-RAND-11] qpos/model 변경을 물리엔진 상태에 반영
        mujoco.mj_forward(model, data)


    def check_success(self):
        tmat_block = get_body_tmat(self.mj_data, "block_green")
        tmat_bowl = get_body_tmat(self.mj_data, "bowl_pink")

        return (abs(tmat_bowl[2, 2]) > 0.99) and np.hypot(
            tmat_block[0, 3] - tmat_bowl[0, 3],
            tmat_block[1, 3] - tmat_bowl[1, 3]
        ) < 0.02


cfg = PiperCfg()

cfg.gs_model_dict["background"] = "scene/lab3/point_cloud.ply"
cfg.gs_model_dict["drawer_1"] = "hinge/drawer_1.ply"
cfg.gs_model_dict["drawer_2"] = "hinge/drawer_2.ply"
cfg.gs_model_dict["bowl_pink"] = "object/bowl_pink.ply"
cfg.gs_model_dict["block_green"] = "object/block_green.ply"

cfg.init_qpos[:] = [
    0.0,
    1.2,
    -1.35,
    0.0,
    1.05,
    0.0,
    0.04
]

robot_name = "piper"
task_name = "place_block"

cfg.mjcf_file_path = f"mjcf/tmp/{robot_name}_{task_name}.xml"

cfg.timestep = 1 / 240
cfg.decimation = 4

cfg.sync = False
cfg.headless = False

cfg.render_set = {
    "fps": 20,
    "width": 640,
    "height": 480
}

cfg.obs_rgb_cam_id = [0, 1, 2]
cfg.save_mjb_and_task_config = True


# =========================
# [MOD-RPC-UTIL] (유지)
# =========================
def _encode_jpeg(rgb: np.ndarray, quality: int = 90) -> bytes:
    # DISCOVERSE 이미지가 RGB라고 가정
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ValueError("JPEG encode failed")
    return buf.tobytes()

# ============================================================
# [MOD-VISION-CHECK] ✅ (추가)
# - 로컬에서 "정규화 후 범위"를 근사 확인 (Imagenet mean/std)
# - JPEG 재압축(encode->decode)으로 정보가 얼마나 깎이는지 확인
# ============================================================
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

def _rgb_to_float01(rgb: np.ndarray) -> np.ndarray:
    # rgb: HWC uint8 -> HWC float32 [0,1]
    return rgb.astype(np.float32) / 255.0

def _imagenet_norm_hwc01(x01: np.ndarray) -> np.ndarray:
    # x01: HWC float32 [0,1] -> HWC float32 (x-mean)/std
    return (x01 - _IMAGENET_MEAN) / _IMAGENET_STD

def _dbg_print_image_stats(tag: str, rgb: np.ndarray):
    x01 = _rgb_to_float01(rgb)
    xn = _imagenet_norm_hwc01(x01)
    print(f"[DBG-VISION] {tag} raw01   : min={x01.min():.4f} max={x01.max():.4f} mean={x01.mean():.4f} std={x01.std():.4f}")
    print(f"[DBG-VISION] {tag} imagenet: min={xn.min():.4f} max={xn.max():.4f} mean={xn.mean():.4f} std={xn.std():.4f}")

def _decode_jpeg_bytes(jpg_bytes: bytes) -> np.ndarray:
    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("JPEG decode failed")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

def _dbg_print_jpeg_roundtrip(tag: str, rgb: np.ndarray, quality: int):
    jpg = _encode_jpeg(rgb, quality=quality)
    rgb2 = _decode_jpeg_bytes(jpg)
    diff = rgb.astype(np.int16) - rgb2.astype(np.int16)
    mse = float(np.mean(diff.astype(np.float32) ** 2))
    max_abs = int(np.max(np.abs(diff)))
    if mse <= 1e-12:
        psnr = float("inf")
    else:
        psnr = 10.0 * math.log10((255.0 ** 2) / mse)
    print(f"[DBG-JPEG] {tag} q={quality} mse={mse:.3f} psnr={psnr:.2f}dB max_abs={max_abs}")


class PolicyClientZMQ:
    def __init__(self, connect: str = "tcp://127.0.0.1:5555", timeout_ms: int = 10000):
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.connect(connect)
        self.sock.RCVTIMEO = timeout_ms
        self.sock.SNDTIMEO = timeout_ms

    def reset(self):
        self.sock.send(msgpack.packb({"cmd": "reset"}, use_bin_type=True))
        rep = msgpack.unpackb(self.sock.recv(), raw=False)
        if not rep.get("ok", False):
            raise RuntimeError(rep)

    # =========================
    # [MOD-RPC-CLIENT-POSTPROCESSED]
    # 서버가 postprocessor 적용했는지(rep["postprocessed"]) 같이 받는다.
    # - postprocessed=True: eval에서 action_map을 하면 안 됨(이중 변환)
    # - postprocessed=False: 기존처럼 action_map 적용
    # =========================
    def act(
        self,
        state: np.ndarray,
        images_rgb: dict,
        jpeg_quality: int = 90,
        image_transport: str = "jpeg",
    ):
        if image_transport not in ("jpeg", "raw"):
            raise ValueError(f"Unsupported image_transport: {image_transport}")

        if image_transport == "jpeg":
            images = {
                k: {
                    "encoding": "jpeg",
                    "data": _encode_jpeg(v, quality=jpeg_quality),
                }
                for k, v in images_rgb.items()
            }
        else:
            images = {}
            for k, v in images_rgb.items():
                rgb = np.ascontiguousarray(v)
                if rgb.dtype != np.uint8:
                    raise ValueError(f"{k}: expected uint8 image, got {rgb.dtype}")
                images[k] = {
                    "encoding": "raw",
                    "shape": list(rgb.shape),   # [H, W, C]
                    "dtype": str(rgb.dtype),    # "uint8"
                    "data": rgb.tobytes(),
                }

        req = {
            "cmd": "act",
            "state": state.astype(np.float32).tolist(),
            "image_transport": image_transport,
            "images": images,
        }

        self.sock.send(msgpack.packb(req, use_bin_type=True))
        rep = msgpack.unpackb(self.sock.recv(), raw=False)
        if not rep.get("ok", False):
            raise RuntimeError(rep)

        action = np.asarray(rep["action"], dtype=np.float32)
        postprocessed = bool(rep.get("postprocessed", False))  # 서버가 보내주는 flag
        return action, postprocessed


def print_action_debug_once(action):
    print("\n[MOD-EVAL-DEBUG] action.shape =", action.shape)
    print("[MOD-EVAL-DEBUG] action =", action)


# ============================================================
# [MOD-RPC-ACTION-MAP]
# 핵심 수정: policy action([-1,1]같은 정규화 값)을
# Mujoco actuator가 제어하는 joint range로 역정규화해서
# sim_node.step()에 "실제 qpos target"으로 넣는다.
# ============================================================
def _get_actuated_joint_ranges(mj_model):
    """
    actuator -> joint id 매핑을 이용해, 실제로 제어되는 관절들의 range를 가져온다.
    - mj_model.nu == 제어 입력 차원(대부분 7이어야 정상)
    - actuator_trnid[a,0] == joint id
    """
    lows, highs, names = [], [], []
    for a in range(mj_model.nu):
        jid = int(mj_model.actuator_trnid[a, 0])
        jname = mj_model.joint(jid).name
        rng = np.array(mj_model.jnt_range[jid], dtype=np.float32)

        # range가 (0,0)처럼 비정상인 경우 방어
        if np.isclose(rng[0], rng[1]):
            rng = np.array([-1.0, 1.0], dtype=np.float32)

        lows.append(rng[0])
        highs.append(rng[1])
        names.append(jname)

    return np.asarray(lows, dtype=np.float32), np.asarray(highs, dtype=np.float32), names


def _denorm_unit_to_range(x, low, high):
    """
    x in [-1,1] -> [low, high]
    """
    return (x + 1.0) * 0.5 * (high - low) + low


def _maybe_map_action_to_env(action, act_low, act_high):
    """
    heuristics:
    - action이 거의 [-1,1] 범위면 "정규화 action"으로 보고 joint range로 매핑
    - 아니면 그대로 통과
    """
    if action.shape != act_low.shape:
        return action, False

    if np.all(action >= -1.2) and np.all(action <= 1.2):
        mapped = _denorm_unit_to_range(action, act_low, act_high)
        mapped = np.clip(mapped, act_low, act_high)
        return mapped.astype(np.float32), True

    return action.astype(np.float32), False


# ============================================================
# [MOD-PARTIAL-METRICS] (추가)
# "실패해도 어디까지 성공했는지"를 보기 위한 중간 단계 지표 계산 유틸
# - check_success()와 동일한 body 이름(block_green, bowl_pink)을 사용
# - 따라서 별도 설정 없이 바로 동작함
# ============================================================
def dist_xy_block_to_bowl(sim_node) -> float:
    """
    블록과 볼의 XY 평면 거리.
    - 정렬(ALIGN) 정도를 수치화하는 가장 중요한 지표
    """
    tmat_block = get_body_tmat(sim_node.mj_data, "block_green")
    tmat_bowl = get_body_tmat(sim_node.mj_data, "bowl_pink")
    dx = tmat_block[0, 3] - tmat_bowl[0, 3]
    dy = tmat_block[1, 3] - tmat_bowl[1, 3]
    return float(np.hypot(dx, dy))


def block_height(sim_node) -> float:
    """
    블록의 Z 높이.
    - LIFT(픽/리프트) 정도를 수치화
    """
    tmat_block = get_body_tmat(sim_node.mj_data, "block_green")
    return float(tmat_block[2, 3])


# ============================================================
# [MOD-EJECT-DIAG] (추가)
# "짚고 올리고 이동하자마자 튕겨나감" 원인 진단용 유틸
# - 블록과 EE(엔드이펙터) 거리: 잡고 있으면 작아야 함
# - obs에 'ep' (EE position) 키가 이미 존재하므로 그걸 활용
# ============================================================
def block_to_ee_distance(sim_node, obs) -> float:
    """
    블록 위치(mj_data)와 EE 위치(obs["ep"])의 3D 거리.
    - 잡고 있으면 일정 거리 이하로 유지됨
    - 튕기면 갑자기 크게 증가함
    """
    block_pos = get_body_tmat(sim_node.mj_data, "block_green")[:3, 3]
    ee_pos = np.asarray(obs["ep"], dtype=np.float32)
    return float(np.linalg.norm(block_pos - ee_pos))


# ============================================================
# [MOD-PARTIAL-METRICS] (유지)
# 단계(Stage) 판정 임계값들
# ============================================================
Z_LIFT = 0.06
Z_PLACE = 0.03
D_CARRY = 0.06
D_ALIGN = 0.03

# ============================================================
# [MOD-EJECT-DIAG] (추가)
# 핵심: 기존 Z_LIFT(절대 높이)는 월드 좌표 기준이라 의미가 없을 수 있음.
# 그래서 에피소드 시작 후 baseline(z0)을 찍고, Δz로 리프트를 판정한다.
# ============================================================
Z_LIFT_DELTA = 0.03   # baseline 대비 이만큼 올라가면 "진짜 들어올림"으로 간주 (필요시 조절)

# ============================================================
# [MOD-EJECT-DIAG] (추가)
# "튕김(eject)" 감지/로그 임계값
# - D_GRASP: 이 거리보다 작으면 "잡았다(접촉/그립)"로 추정
# - D_EJECT: 잡은 뒤 이 거리보다 커지면 "튕겨나갔다"로 추정
# ============================================================
D_GRASP = 0.05
D_EJECT = 0.10

# ============================================================
# [MOD-EJECT-DIAG] (추가)
# 주기 로그 출력 간격(너무 많으면 느려지니 5~20 권장)
# ============================================================
PRINT_EVERY = 5


# ============================================================
# [MOD-EJECT-DIAG] (추가)
# 진단용 스위치 (기본 False)
# ============================================================
FORCE_HOLD_GRIPPER = False
HOLD_GRIPPER_VALUE = 0.0   # finger_joint1에서 0.0이 닫힘이라는 가정(필요시 0.04로 바꿔 테스트)

LIMIT_JERK = False
MAX_DELTA = np.array([0.05, 0.05, 0.05, 0.07, 0.07, 0.10, 0.005], dtype=np.float32)


# ============================================================
# [MOD-FINGER2-CHECK] (추가)
# finger_joint1 / finger_joint2가 실제로 어떻게 모델에 들어있는지,
# 그리고 finger_joint2가 actuator로 구동되는지 확인하기 위한 유틸.
# ============================================================
def _find_joint_id_by_name(mj_model, name: str):
    for j in range(mj_model.njnt):
        if mj_model.joint(j).name == name:
            return j
    return None


def _find_actuator_index_by_joint_id(mj_model, jid: int):
    # actuator_trnid[a,0] == joint id
    for a in range(mj_model.nu):
        if int(mj_model.actuator_trnid[a, 0]) == int(jid):
            return a
    return None


# ============================================================
# [MOD-SUCCESS-STRICT] (추가) ✅✅✅
# "그릇에 안 들어갔는데 SUCCESS" 문제 해결 핵심.
# 기존 sim_node.check_success()는 사실상 XY 정렬만 보고 통과할 수 있음.
# 따라서 "진짜 place"로 보기 위해 아래를 추가로 요구:
#  1) 기존 check_success() 통과 (일단 XY 조건/기존 기준 유지)
#  2) finger_joint1, finger_joint2가 충분히 OPEN 되었는지 (release)
#  3) 블록이 볼 높이 근처까지 내려왔는지 (들고 있는 상태 제외)
#
# ※ SUCCESS_OPEN_THR / SUCCESS_Z_MARGIN 는 환경별로 조정 필요.
#    처음엔 conservative(엄격)하게 잡고, 로그 보고 완화하는 게 안전.
# ============================================================
SUCCESS_OPEN_THR = 0.032   # 0.04가 완전 open이면, 0.032 이상이면 "거의 열림"으로 간주
SUCCESS_Z_MARGIN = 0.035   # bowl_z + margin 보다 block_z가 낮아야 "내려놓음"으로 간주

def strict_success(sim_node, mj_model, finger1_jid, finger2_jid):
    # (1) 기존 성공 조건 (XY 정렬) 먼저 만족해야 함
    if not sim_node.check_success():
        return False

    d = sim_node.mj_data

    # (2) finger qpos 읽기 (양쪽 손가락)
    # - finger2가 actuator가 없어도 joint는 존재하므로 qpos는 읽을 수 있음
    # - 만약 finger2_jid가 None이면, 최소 조건으로 finger1만이라도 본다(그래도 기존보단 훨씬 엄격)
    f1_q = None
    f2_q = None

    if finger1_jid is not None:
        f1_q = float(d.qpos[int(mj_model.jnt_qposadr[finger1_jid])])
    if finger2_jid is not None:
        f2_q = float(d.qpos[int(mj_model.jnt_qposadr[finger2_jid])])

    if f1_q is None:
        return False

    if f2_q is None:
        open_ok = (f1_q >= SUCCESS_OPEN_THR)
    else:
        open_ok = (f1_q >= SUCCESS_OPEN_THR) and (f2_q >= SUCCESS_OPEN_THR)

    # (3) block_z가 bowl_z + margin 보다 낮아야 "실제로 내려놨다"로 판단
    tmat_block = get_body_tmat(sim_node.mj_data, "block_green")
    tmat_bowl  = get_body_tmat(sim_node.mj_data, "bowl_pink")
    block_z = float(tmat_block[2, 3])
    bowl_z  = float(tmat_bowl[2, 3])

    z_ok = (block_z <= bowl_z + SUCCESS_Z_MARGIN)

    return open_ok and z_ok


if __name__ == "__main__":

    print(discoverse.__logo__)

    np.set_printoptions(
        precision=3,
        suppress=True,
        linewidth=500
    )

    parser = argparse.ArgumentParser()

    # 원본 인자 유지
    parser.add_argument("--data_idx", type=int, default=0)
    parser.add_argument("--data_set_size", type=int, default=1)
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--use_gs", action="store_true")

    # 평가용 인자(호환용 유지)
    parser.add_argument("--model_path", type=str, default="/home/qwer1234/lerobot_ws/pretrained_model")
    parser.add_argument("--num_eval_episodes", type=int, default=10)
    parser.add_argument("--max_steps", type=int, default=150)
    parser.add_argument("--save_video", action="store_true")

    # [MOD-RPC-ARGS] 유지
    parser.add_argument("--policy_server", type=str, default="tcp://127.0.0.1:5555")

    # ============================================================
    # [MOD-ADD-DIAG-ARGS] (추가)
    # ============================================================
    parser.add_argument("--diag_grip_force", action="store_true",
                        help="finger_joint1 actuator_force/ctrl/qpos + finger_joint2 qpos를 PRINT_EVERY마다 출력")
    parser.add_argument("--print_every", type=int, default=5,
                        help="DIAG 출력 간격(step). 기존 PRINT_EVERY를 런타임에서 덮어씀")

    parser.add_argument("--demo_like", action="store_true",
                        help="데모 수집 코드처럼: (1) grasp 직후 대기(0.6s) (2) arm step_func 스무딩 (3) policy 호출 주기 조절")
    parser.add_argument("--demo_action_repeat", type=int, default=1,
                        help="demo_like일 때 policy를 몇 step마다 1번 호출할지. 예: 3이면 60Hz sim에서 20Hz로 policy 호출")
    parser.add_argument("--demo_hold_wait_s", type=float, default=0.6,
                        help="demo_like일 때 grasp(닫힘) 감지 후 대기 시간(초). 데모 코드의 0.6s와 동일 기본값")
    parser.add_argument("--demo_grip_close_thresh", type=float, default=0.016,
                        help="demo_like에서 '닫힘 감지' 임계값. 네 로그 기준 0.016 근처가 grasp 순간이었음")
    parser.add_argument("--demo_move_speed", type=float, default=0.75,
                        help="demo_like에서 arm 스무딩 속도. 데모 코드 move_speed=0.75와 동일 기본값")
    # ============================================================
    # [MOD-VISION-CHECK] ✅ (추가)
    # - jpeg_quality: 서버로 보낼 때 JPEG 재압축 품질(A/B)
    # - dbg_preproc_stats: (로컬 근사) imagenet 정규화 후 통계 출력
    # - dbg_jpeg_stats: JPEG 라운드트립(원본 vs 디코드) 오차/PSNR 출력
    # ============================================================
    parser.add_argument("--jpeg_quality", type=int, default=100,
                    help="policy_server로 보낼 JPEG 품질(0~100). A/B 테스트용. 기본 100")
    parser.add_argument("--image_transport", type=str, default="jpeg", choices=["jpeg", "raw"],
                    help="policy_server로 이미지 전송 방식 선택: jpeg 또는 raw")
    parser.add_argument("--dbg_preproc_stats", action="store_true",
                    help="(로컬 근사) imagenet 정규화 후 이미지 텐서 통계를 에피소드 시작(step=0) 1회 출력")
    parser.add_argument("--dbg_jpeg_stats", action="store_true",
                    help="JPEG 라운드트립(encode->decode) 오차(MSE/PSNR/max_abs)를 에피소드 시작(step=0) 1회 출력")

    # ============================================================
    # [MOD-SETTLE-ARGS] ✅✅✅ (추가)
    # "큐브가 완전히 떨어지기 전에 리셋/종료되는 문제"를 해결하기 위한 옵션.
    #
    # 왜 필요한가?
    # - 기존 코드는 매 step마다 성공(strict_success/check_success)을 보자마자 break할 수 있음.
    # - release 직후(큐브가 공중에서 흔들리는 순간) 일시적으로 조건이 맞으면 조기 종료 가능.
    #
    # 해결 아이디어:
    # - release(open)을 1번 감지(latch)한 뒤,
    # - 일정 시간(post_release_wait_s) 동안 팔을 고정+그리퍼 open 유지해서 큐브가 "완전히" 떨어질 시간을 준다.
    # - 그 다음 성공판정을 "딱 1번만" 수행하고 종료한다.
    #
    # success_mode:
    # - demo  : 데모 수집 코드와 동일한 sim_node.check_success()로 최종 판정(너가 원한 '데모 기준' 일치)
    # - strict: strict_success()로 최종 판정(더 엄격: release+z 조건 포함)
    # ============================================================
    parser.add_argument("--post_release_wait_s", type=float, default=1.2,
                        help="release 감지 후 settle 대기 시간(초). 1.0~2.0 권장")
    parser.add_argument("--success_mode", type=str, default="demo", choices=["demo", "strict"],
                        help="성공판정: demo=check_success(데모 동일), strict=strict_success(엄격)")
    parser.add_argument("--post_release_open_cmd_thresh", type=float, default=0.03,
                        help="release 감지 임계값. action[-1] >= thresh 이면 open으로 간주")

    args = parser.parse_args()

    # ============================================================
    # [MOD-PRINT-EVERY] (추가)
    # ============================================================
    PRINT_EVERY = int(args.print_every)

    if not hasattr(args, "save_segment"):
        args.save_segment = False

    data_idx = args.data_idx
    data_set_size = args.data_idx + args.data_set_size

    if args.auto:
        cfg.headless = True
        cfg.sync = False

    cfg.use_gaussian_renderer = args.use_gs

    save_dir = os.path.join(
        DISCOVERSE_ROOT_DIR,
        "data",
        os.path.splitext(os.path.basename(__file__))[0]
    )
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    sim_node = SimNode(cfg)
    print("camera_names =", sim_node.camera_names)

    if cfg.save_mjb_and_task_config and data_idx == 0:
        mujoco.mj_saveModel(
            sim_node.mj_model,
            os.path.join(save_dir, os.path.basename(cfg.mjcf_file_path).replace(".xml", ".mjb"))
        )
        copypy2(os.path.abspath(__file__), os.path.join(save_dir, os.path.basename(__file__)))

    # [MOD-RPC-POLICY] 유지
    client = PolicyClientZMQ(connect=args.policy_server)
    print(f"[MOD-RPC-POLICY] Connected policy server: {args.policy_server}")
    print(f"[MOD-RPC-POLICY] (model_path arg is kept but server uses its own model) model_path={args.model_path}")

    # ============================================================
    # [MOD-RPC-ACTION-MAP] 추가
    # ============================================================
    act_low, act_high, act_names = _get_actuated_joint_ranges(sim_node.mj_model)
    print("[MOD-RPC-ACTION-MAP] mj_model.nu =", sim_node.mj_model.nu)
    print("[MOD-RPC-ACTION-MAP] actuator_joint_names =", act_names)
    print("[MOD-RPC-ACTION-MAP] actuator_ranges(low, high) =", list(zip(act_low.tolist(), act_high.tolist())))

    # ============================================================
    # [MOD-FINGER2-CHECK] (추가)
    # ============================================================
    m = sim_node.mj_model
    finger1_jid = _find_joint_id_by_name(m, "finger_joint1")
    finger2_jid = _find_joint_id_by_name(m, "finger_joint2")
    finger1_aid = _find_actuator_index_by_joint_id(m, finger1_jid) if finger1_jid is not None else None
    finger2_aid = _find_actuator_index_by_joint_id(m, finger2_jid) if finger2_jid is not None else None

    print("\n" + "-" * 80)
    print("[MOD-FINGER2-CHECK] joint/actuator wiring check")
    print("-" * 80)
    print("finger_joint1 jid =", finger1_jid, "actuator_id =", finger1_aid)
    print("finger_joint2 jid =", finger2_jid, "actuator_id =", finger2_aid)
    if finger1_jid is not None:
        print("finger_joint1 range =", m.jnt_range[finger1_jid], "qpos_adr =", int(m.jnt_qposadr[finger1_jid]))
    if finger2_jid is not None:
        print("finger_joint2 range =", m.jnt_range[finger2_jid], "qpos_adr =", int(m.jnt_qposadr[finger2_jid]))
    if finger2_aid is None:
        print("[MOD-FINGER2-CHECK] ⚠️ finger_joint2는 actuator로 구동되지 않는 상태일 가능성이 큼(=한쪽만 구동).")
    print("-" * 80 + "\n")

    # 평가 결과 누적용 변수 (원본 유지)
    success_count = 0
    episode_results = []

    # ============================================================
    # [MOD-PARTIAL-METRICS] (추가)
    # ============================================================
    lift_count = 0
    carry_count = 0
    align_count = 0
    release_count = 0

    min_dist_list = []
    final_dist_list = []
    max_z_list = []

    has_printed_obs_debug = False
    has_printed_action_debug = False
    has_printed_action_map_debug = False  # [MOD-RPC-ACTION-MAP] 추가: 매핑 로그 1회만

    for ep in range(args.num_eval_episodes):

        print("\n" + "=" * 80)
        print(f"[MOD-EVAL] EPISODE {ep} START")
        print("=" * 80)

        sim_node.reset()

        # [MOD-RPC-RESET] 유지
        client.reset()

        warmup_action = np.array(cfg.init_qpos, dtype=np.float32)

        # ============================================================
        # [MOD-RPC-WARMUP-OBS] 유지
        # ============================================================
        obs, _, _, _, _ = sim_node.step(warmup_action)
        obs, _, _, _, _ = sim_node.step(warmup_action)

        if not has_printed_obs_debug:
            print("\n[MOD-EVAL-DEBUG] obs.keys() =", list(obs.keys()))
            print("[MOD-EVAL-DEBUG] obs.get('jq') shape =",
                  None if obs.get("jq", None) is None else np.array(obs.get("jq")).shape)
            print("[MOD-EVAL-DEBUG] obs.get('img') keys =",
                  None if obs.get("img", None) is None else list(obs["img"].keys()))
            has_printed_obs_debug = True

        ep_success = False
        ep_steps = 0

        # ============================================================
        # [MOD-EJECT-DIAG] (추가)
        # ============================================================
        z0 = block_height(sim_node)
        d0 = dist_xy_block_to_bowl(sim_node)
        print(f"[BASELINE] EP {ep} z0={z0:.4f} d0(dist_xy)={d0:.4f}")

        # ============================================================
        # [MOD-PARTIAL-METRICS] (추가) 에피소드별 stage 초기화
        # ============================================================
        did_lift = False
        did_carry = False
        did_align = False
        did_release = False

        min_dist = 1e9
        max_z = -1e9

        prev_d_be = None
        ever_grasped = False
        ejected_once = False

        # ============================================================
        # [MOD-DEMO-LIKE] (추가)
        # ============================================================
        last_action = warmup_action.copy()
        applied_action = warmup_action.copy()
        demo_wait_steps = 0
        demo_seen_close = False

        # ============================================================
        # [MOD-SETTLE-STATE] ✅✅✅ (추가)
        # 목적: "큐브가 완전히 떨어지기 전에 평가가 종료되는 문제" 해결
        #
        # release_latched:
        # - 한번 release(open)를 감지하면 True로 고정.
        # - 이후에는 policy 출력이 다시 그리퍼를 닫거나 팔을 움직여도 무시하고
        #   settle 동안 팔 고정 + 그리퍼 open 유지로 안정화 시간을 확보.
        #
        # post_release_steps:
        # - settle 대기 step 카운터.
        # - post_release_wait_s / sim_node.delta_t 로 환산.
        #
        # final_checked:
        # - 최종 성공판정을 이미 수행했는지(중복 방지).
        #
        # 왜 이런 방식이 필요한가?
        # - 기존 코드처럼 매 step 성공검사(특히 strict_success)를 하다 보면,
        #   release 직후 block이 공중/접촉 중인데도 순간적으로 조건이 맞아 조기 종료될 수 있음.
        # - 데모 수집은 "동작 끝난 뒤" 성공판정 1회였기 때문에,
        #   eval도 동일한 철학(마지막 1회 판정)으로 맞추려면 settle 로직이 필요.
        # ============================================================
        release_latched = False
        post_release_steps = 0
        final_checked = False

        # ============================================================
        # [MOD-SUCCESS-STRICT] (추가)
        # "기존 check_success는 True인데 strict_success는 False" 상황을
        # 에피소드 당 1번만 찍어서 디버깅 가능하게 함.
        # (너가 겪은 '안 담겼는데 성공' 바로 이 케이스를 잡아냄)
        # ============================================================
        printed_loose_success_warning = False

        # 비디오 저장 (원본 유지)
        encoders = None
        video_save_path = None
        if args.save_video:
            video_save_path = os.path.join(save_dir, f"eval_ep_{ep:03d}")
            if os.path.exists(video_save_path):
                shutil.rmtree(video_save_path, ignore_errors=True)
            os.makedirs(video_save_path, exist_ok=True)

            encoders = {
                cam_id: PyavImageEncoder(
                    cfg.render_set["width"],
                    cfg.render_set["height"],
                    video_save_path,
                    cam_id
                )
                for cam_id in cfg.obs_rgb_cam_id
            }

        for step_idx in range(args.max_steps):

            jq = np.asarray(obs["jq"], dtype=np.float32)
            imgs = obs["img"]  # dict: {0: img0, 1: img1, 2: img2}

            eye_side_img = imgs[0]
            eye_front_img = imgs[1]
            cam_wrist_img = imgs[2]

            images = {
                "observation.images.eye_side": eye_side_img,
                "observation.images.eye_front": eye_front_img,
                "observation.images.cam_wrist": cam_wrist_img,
            }
            # ============================================================
            # [MOD-VISION-CHECK] ✅ (추가)
            # 에피소드 시작(step=0) 1회만:
            # 1) (로컬 근사) Imagenet 정규화 후 통계로 "정규화 범위가 말이 되는지" 확인
            # 2) JPEG 라운드트립(encode->decode) 오차/PSNR로 "재압축이 정밀도를 깎는지" 확인
            # ============================================================
            if step_idx == 0:
                if args.dbg_preproc_stats:
                    _dbg_print_image_stats("eye_side", eye_side_img)
                    _dbg_print_image_stats("eye_front", eye_front_img)
                    _dbg_print_image_stats("cam_wrist", cam_wrist_img)
                if args.dbg_jpeg_stats:
                    _dbg_print_jpeg_roundtrip("eye_side", eye_side_img, int(args.jpeg_quality))
                    _dbg_print_jpeg_roundtrip("eye_front", eye_front_img, int(args.jpeg_quality))
                    _dbg_print_jpeg_roundtrip("cam_wrist", cam_wrist_img, int(args.jpeg_quality))

            # ============================================================
            # [MOD-DEMO-LIKE] (추가)
            # ============================================================
            do_query = True
            if args.demo_like:
                r = max(1, int(args.demo_action_repeat))
                do_query = (step_idx % r == 0)

            # ============================================================
            # [MOD-SETTLE-NO-QUERY] ✅✅✅ (추가)
            # settle 대기 중에는 action을 강제로 고정하므로,
            # policy 서버 호출(do_query)을 끊어서:
            # - 불필요한 통신/지연/로그를 줄이고
            # - policy가 다시 그리퍼를 닫는 출력 등을 보내는 것을 원천 차단
            # ============================================================
            if release_latched and post_release_steps > 0:
                do_query = False

            if do_query:
                action, postprocessed = client.act(
                    state=jq,
                    images_rgb=images,
                    jpeg_quality=int(args.jpeg_quality),
                    image_transport=str(args.image_transport),
                )

                # ============================================================
                # [MOD-RPC-ACTION-MAP-CONDITIONAL] (유지)
                # ============================================================
                if postprocessed:
                    mapped_action = action
                    did_map = False
                else:
                    mapped_action, did_map = _maybe_map_action_to_env(action, act_low, act_high)

                if not has_printed_action_map_debug:
                    print("\n[MOD-RPC-ACTION-MAP-DEBUG] raw_action    =", action)
                    print("[MOD-RPC-ACTION-MAP-DEBUG] postprocessed =", postprocessed)
                    print("[MOD-RPC-ACTION-MAP-DEBUG] mapped_action =", mapped_action)
                    print("[MOD-RPC-ACTION-MAP-DEBUG] did_map =", did_map)
                    has_printed_action_map_debug = True

                action = mapped_action
                last_action = action.copy()
            else:
                action = last_action.copy()

            # ============================================================
            # [MOD-EJECT-DIAG] (기존 유지)
            # ============================================================
            if LIMIT_JERK:
                action = jq + np.clip(action - jq, -MAX_DELTA, MAX_DELTA)

            if FORCE_HOLD_GRIPPER:
                try:
                    d_be_now = block_to_ee_distance(sim_node, obs)
                    if d_be_now < D_GRASP:
                        action[-1] = HOLD_GRIPPER_VALUE
                except Exception:
                    pass

            # ============================================================
            # [MOD-DEMO-LIKE] (추가)
            # ============================================================
            if args.demo_like:
                close_th = float(args.demo_grip_close_thresh)
                if (not demo_seen_close) and (action[-1] < close_th):
                    demo_seen_close = True
                    demo_wait_steps = int(float(args.demo_hold_wait_s) / max(1e-6, sim_node.delta_t))

                if demo_wait_steps > 0:
                    action[:6] = jq[:6]
                    demo_wait_steps -= 1

                move_speed = float(args.demo_move_speed)
                for i in range(6):
                    applied_action[i] = step_func(
                        applied_action[i],
                        action[i],
                        move_speed * sim_node.delta_t
                    )
                applied_action[6] = action[6]
                action = applied_action.copy()

            # ============================================================
            # [MOD-SETTLE-DETECT] ✅✅✅ (추가)
            # ============================================================
            if (not release_latched) and demo_seen_close and (float(action[-1]) >= float(args.post_release_open_cmd_thresh)):
                release_latched = True
                post_release_steps = int(float(args.post_release_wait_s) / max(1e-6, sim_node.delta_t))
                post_release_steps = max(1, post_release_steps)
                print(f"[MOD-SETTLE] EP{ep} release detected at step={step_idx}, settle_steps={post_release_steps}, wait_s={args.post_release_wait_s}")

            # ============================================================
            # [MOD-SETTLE-HOLD] ✅✅✅ (추가)
            # ============================================================
            if release_latched and post_release_steps > 0:
                action[:6] = jq[:6]
                action[6] = 0.04
                applied_action[:6] = action[:6]
                applied_action[6] = action[6]

            if not has_printed_action_debug:
                print_action_debug_once(action)
                has_printed_action_debug = True

            # step (원본 유지)
            obs, _, _, _, _ = sim_node.step(action)
            ep_steps += 1

            # ============================================================
            # [MOD-PARTIAL-METRICS] + [MOD-EJECT-DIAG] (기존 유지)
            # ============================================================
            z = block_height(sim_node)
            d = dist_xy_block_to_bowl(sim_node)

            if d < min_dist:
                min_dist = d
            if z > max_z:
                max_z = z

            dz = z - z0
            if (not did_lift) and (dz > Z_LIFT_DELTA):
                did_lift = True

            if did_lift and (not did_carry) and (d < D_CARRY):
                did_carry = True

            if (not did_align) and (d < D_ALIGN):
                did_align = True

            if did_align and (not did_release) and (z < Z_PLACE) and (d < D_ALIGN):
                did_release = True

            d_be = block_to_ee_distance(sim_node, obs)
            grip_cmd = float(action[-1])
            grip_q = float(obs["jq"][-1])

            # ============================================================
            # [MOD-GRIP-FORCE] (추가)
            # ============================================================
            if args.diag_grip_force and (step_idx % PRINT_EVERY == 0):
                ddata = sim_node.mj_data
                f1_q = None
                f2_q = None
                if finger1_jid is not None:
                    f1_q = float(ddata.qpos[int(m.jnt_qposadr[finger1_jid])])
                if finger2_jid is not None:
                    f2_q = float(ddata.qpos[int(m.jnt_qposadr[finger2_jid])])

                if finger1_aid is not None:
                    ctrl = float(ddata.ctrl[finger1_aid])
                    af = float(ddata.actuator_force[finger1_aid])
                else:
                    ctrl = None
                    af = None

                print(f"[GRIP-FORCE] EP{ep} step={step_idx} ctrl={ctrl} act_force={af}  f1_qpos={f1_q} f2_qpos={f2_q}")

            if d_be < D_GRASP:
                ever_grasped = True

            if (not ejected_once) and ever_grasped and (d_be > D_EJECT):
                ejected_once = True
                print(f"[EJECT-DETECTED] EP{ep} step={step_idx} d(block-ee)={d_be:.4f}  grip_cmd={grip_cmd:.4f}  grip_q={grip_q:.4f}  dz={dz:.4f}")

            if step_idx % PRINT_EVERY == 0:
                if args.demo_like:
                    print(f"[DIAG] EP{ep} step={step_idx} dz={dz:.4f} dist_xy={d:.4f} d(block-ee)={d_be:.4f} grip_cmd={grip_cmd:.4f} grip_q={grip_q:.4f} demo_wait={demo_wait_steps}")
                else:
                    print(f"[DIAG] EP{ep} step={step_idx} dz={dz:.4f} dist_xy={d:.4f} d(block-ee)={d_be:.4f} grip_cmd={grip_cmd:.4f} grip_q={grip_q:.4f}")

            prev_d_be = d_be

            if args.save_video and encoders is not None:
                imgs = obs["img"]
                for cam_id, img in imgs.items():
                    encoders[cam_id].encode(img, obs["time"])

            # ============================================================
            # [MOD-SETTLE-FINAL] ✅✅✅ (추가)
            # ============================================================
            if release_latched and post_release_steps > 0:
                post_release_steps -= 1

                if (post_release_steps == 0) and (not final_checked):
                    final_checked = True

                    if args.success_mode == "demo":
                        ep_success = bool(sim_node.check_success())
                        mode_name = "DEMO(check_success)"
                    else:
                        ep_success = bool(strict_success(sim_node, m, finger1_jid, finger2_jid))
                        mode_name = "STRICT(strict_success)"

                    if ep_success:
                        success_count += 1
                        print(f"[MOD-SETTLE] EP {ep} FINAL SUCCESS after settle (mode={mode_name})")
                    else:
                        print(f"[MOD-SETTLE] EP {ep} FINAL FAIL after settle (mode={mode_name})")

                    break

            # ============================================================
            # [MOD-SUCCESS-STRICT] (추가)
            # ============================================================
            if (not printed_loose_success_warning) and sim_node.check_success():
                printed_loose_success_warning = True
                # finger qpos / z 비교를 찍어서 margin 조절 근거로 사용
                tmat_block = get_body_tmat(sim_node.mj_data, "block_green")
                tmat_bowl  = get_body_tmat(sim_node.mj_data, "bowl_pink")
                block_z = float(tmat_block[2, 3])
                bowl_z  = float(tmat_bowl[2, 3])

                f1_q = None
                f2_q = None
                if finger1_jid is not None:
                    f1_q = float(sim_node.mj_data.qpos[int(m.jnt_qposadr[finger1_jid])])
                if finger2_jid is not None:
                    f2_q = float(sim_node.mj_data.qpos[int(m.jnt_qposadr[finger2_jid])])

                print("[STRICT-WARN] loose check_success=True but strict_success=False")
                print(f"[STRICT-WARN] f1_q={f1_q} f2_q={f2_q}  block_z={block_z:.4f} bowl_z={bowl_z:.4f} (block_z-bowl_z)={block_z-bowl_z:.4f}")
                print(f"[STRICT-WARN] need: open>={SUCCESS_OPEN_THR}, block_z <= bowl_z+{SUCCESS_Z_MARGIN}")

        # ============================================================
        # [MOD-SETTLE-FALLBACK] ✅✅✅ (추가)
        # ============================================================
        if not final_checked:
            final_checked = True

            if args.success_mode == "demo":
                ep_success = bool(sim_node.check_success())
                mode_name = "DEMO(check_success)"
            else:
                ep_success = bool(strict_success(sim_node, m, finger1_jid, finger2_jid))
                mode_name = "STRICT(strict_success)"

            if ep_success:
                success_count += 1
                print(f"[MOD-SETTLE] EP {ep} FINAL SUCCESS at episode end (mode={mode_name})")
            else:
                print(f"[MOD-SETTLE] EP {ep} FINAL FAIL at episode end (mode={mode_name})")

        if args.save_video and encoders is not None:
            for encoder in encoders.values():
                try:
                    encoder.close()
                except Exception:
                    pass

        if did_lift:
            lift_count += 1
        if did_carry:
            carry_count += 1
        if did_align:
            align_count += 1
        if did_release:
            release_count += 1

        min_dist_list.append(min_dist)
        final_dist_list.append(dist_xy_block_to_bowl(sim_node))
        max_z_list.append(max_z)

        print(f"[PARTIAL] EP {ep} stages: lift={did_lift}, carry={did_carry}, align={did_align}, release={did_release}, success={ep_success}")
        print(f"[PARTIAL] EP {ep} stats : min_dist_xy={min_dist:.4f}, final_dist_xy={final_dist_list[-1]:.4f}, max_z={max_z:.4f}")

        episode_results.append({
            "episode": ep,
            "success": ep_success,
            "steps": ep_steps
        })

        print(f"[MOD-EVAL] EP {ep} RESULT -> success={ep_success}, steps={ep_steps}")

    print("\n" + "=" * 80)
    print("[MOD-EVAL] FINAL SUMMARY")
    print("=" * 80)

    success_rate = success_count / args.num_eval_episodes if args.num_eval_episodes > 0 else 0.0

    print(f"[MOD-EVAL] num_eval_episodes = {args.num_eval_episodes}")
    print(f"[MOD-EVAL] success_count     = {success_count}")
    print(f"[MOD-EVAL] success_rate      = {success_rate:.4f}")

    N = args.num_eval_episodes if args.num_eval_episodes > 0 else 1
    lift_rate = lift_count / N
    carry_rate = carry_count / N
    align_rate = align_count / N
    release_rate = release_count / N

    print("\n" + "-" * 80)
    print("[PARTIAL] STAGE SUCCESS RATES")
    print("-" * 80)
    print(f"[PARTIAL] lift_rate    = {lift_rate*100:.2f}% ({lift_count}/{N})")
    print(f"[PARTIAL] carry_rate   = {carry_rate*100:.2f}% ({carry_count}/{N})")
    print(f"[PARTIAL] align_rate   = {align_rate*100:.2f}% ({carry_count}/{N})")
    print(f"[PARTIAL] release_rate = {release_rate*100:.2f}% ({release_count}/{N})")
    print(f"[PARTIAL] success_rate = {success_rate*100:.2f}% ({success_count}/{N})")

    print("\n" + "-" * 80)
    print("[PARTIAL] CONTINUOUS PROGRESS STATS")
    print("-" * 80)
    print(f"[PARTIAL] avg_min_dist_xy   = {float(np.mean(min_dist_list)):.4f} m")
    print(f"[PARTIAL] avg_final_dist_xy = {float(np.mean(final_dist_list)):.4f} m")
    print(f"[PARTIAL] avg_max_z         = {float(np.mean(max_z_list)):.4f} m")

    print("\n[MOD-EVAL] episode-wise results")
    for item in episode_results:
        print(item)
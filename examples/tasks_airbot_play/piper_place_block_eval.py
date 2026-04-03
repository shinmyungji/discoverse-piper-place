import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import os
import shutil  # 원본 코드 유지
import argparse
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
        pass

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
    def act(self, state: np.ndarray, images_rgb: dict, jpeg_quality: int = 90):
        images = {k: _encode_jpeg(v, quality=jpeg_quality) for k, v in images_rgb.items()}
        req = {
            "cmd": "act",
            "state": state.astype(np.float32).tolist(),
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
                action, postprocessed = client.act(state=jq, images_rgb=images, jpeg_quality=90)

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
            # "release(open)"를 감지하는 기준:
            # - demo_like를 켜면, 최소한 한번 '닫힘'을 봤는지(demo_seen_close)로 gating
            #   (초기부터 open인 이상 출력으로 settle이 잘못 걸리는 것을 방지)
            # - action[-1] >= post_release_open_cmd_thresh 이면 open으로 간주
            #
            # release 감지 순간에:
            # - release_latched=True로 고정하고
            # - post_release_steps를 설정(초->step 변환)
            #
            # 중요:
            # - 이 release 감지는 "한 번만" 걸려야 함.
            #   그래서 (not release_latched) 조건을 반드시 둔다.
            # ============================================================
            if (not release_latched) and demo_seen_close and (float(action[-1]) >= float(args.post_release_open_cmd_thresh)):
                release_latched = True
                post_release_steps = int(float(args.post_release_wait_s) / max(1e-6, sim_node.delta_t))
                post_release_steps = max(1, post_release_steps)
                print(f"[MOD-SETTLE] EP{ep} release detected at step={step_idx}, settle_steps={post_release_steps}, wait_s={args.post_release_wait_s}")

            # ============================================================
            # [MOD-SETTLE-HOLD] ✅✅✅ (추가)
            # release 이후 settle 동안에는:
            # - 팔 6축을 현재 jq로 고정 -> 큐브 낙하/정착 동안 팔이 움직이면 튕김/접촉이 바뀌어 판정이 흔들림
            # - 그리퍼는 open으로 유지 -> release 상태 유지(다시 닫히면 안 됨)
            #
            # 그리고 demo_like의 applied_action 내부 상태도 함께 고정해서,
            # 다음 step에서도 스무딩 상태 때문에 팔이 미세하게 따라가는 현상을 방지.
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
            # settle 카운트다운:
            # - release_latched 상태에서 post_release_steps가 0이 되는 순간,
            #   성공판정은 "딱 1번만" 수행하고 break 한다.
            #
            # 성공판정 기준을 데모와 맞추고 싶다면:
            #   --success_mode demo  -> sim_node.check_success()로만 판정
            #
            # 더 엄격하게(진짜 place) 하고 싶다면:
            #   --success_mode strict -> strict_success()로 판정
            #
            # 왜 여기서 판정하나?
            # - 큐브가 완전히 떨어져서 정착할 시간을 준 뒤 판정해야
            #   "떨어지는 중인데 성공 처리" 같은 애매함이 사라진다.
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
            # [MOD-SUCCESS-STRICT] (수정) ✅✅✅
            # 기존 코드에서는 strict_success가 True 되는 즉시 break 했음.
            # 그런데 너가 원하는 건:
            #   - 동작을 끝까지 수행하고
            #   - 큐브가 완전히 떨어진 후
            #   - 성공판정을 1회만 하는 것
            #
            # 그래서 "조기 성공 break"는 제거해야 함.
            # (대신 위의 [MOD-SETTLE-FINAL] 블록에서 settle 후 1회 판정)
            # ============================================================
            # if strict_success(sim_node, m, finger1_jid, finger2_jid):
            #     ep_success = True
            #     success_count += 1
            #     print(f"[MOD-EVAL] EP {ep} STRICT SUCCESS at step {step_idx}")
            #     break

            # ============================================================
            # [MOD-SUCCESS-STRICT] (추가)
            # 디버깅 도움:
            # - 기존 check_success는 True인데 strict는 False인 경우가 "너가 겪는 증상"의 핵심
            # - 에피소드 당 1회만 이유를 출력(터미널 도배 방지)
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
        # max_steps에 도달해서 루프가 자연 종료된 경우에도,
        # 최종 성공판정을 "딱 1번" 수행한다.
        #
        # 왜 필요?
        # - release를 못했거나(release_latched=False)
        # - release는 했지만 max_steps가 너무 짧아 settle 판정까지 못 갔거나
        # 이런 경우에도 episode별 success 결과를 남기기 위함.
        #
        # 데모와 동일하게 "한 번만 판정"하려면 여기에서 ep_success를 최종 확정하는 게 안전.
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
    print(f"[PARTIAL] align_rate   = {align_rate*100:.2f}% ({align_count}/{N})")
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
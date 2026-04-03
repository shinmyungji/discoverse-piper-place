import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import os
import shutil  # [MOD-SAVE] 실패한 임시 폴더 삭제용
import argparse
import multiprocessing as mp

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

        tmat_block = get_body_tmat(self.mj_data,"block_green")
        tmat_bowl = get_body_tmat(self.mj_data,"bowl_pink")

        return (abs(tmat_bowl[2,2])>0.99) and np.hypot(
            tmat_block[0,3]-tmat_bowl[0,3],
            tmat_block[1,3]-tmat_bowl[1,3]
        )<0.02



cfg = PiperCfg()

cfg.gs_model_dict["background"]="scene/lab3/point_cloud.ply"
cfg.gs_model_dict["drawer_1"]="hinge/drawer_1.ply"
cfg.gs_model_dict["drawer_2"]="hinge/drawer_2.ply"
cfg.gs_model_dict["bowl_pink"]="object/bowl_pink.ply"
cfg.gs_model_dict["block_green"]="object/block_green.ply"


# 안정적인 시작 자세
cfg.init_qpos[:] = [
    0.0,
    1.2,
    -1.35,
    0.0,
    1.05,
    0.0,
    0.04
]


robot_name="piper"
task_name="place_block"


cfg.mjcf_file_path=f"mjcf/tmp/{robot_name}_{task_name}.xml"

#env=make_env(robot_name,task_name)

#env.export_xml(
#    os.path.join(
#        DISCOVERSE_ASSETS_DIR,
#        cfg.mjcf_file_path
#    )
#)


cfg.timestep=1/240
cfg.decimation=4

cfg.sync=False # 수정함
cfg.headless=False


cfg.render_set={
    "fps":20,
    "width":640,
    "height":480
}


cfg.obs_rgb_cam_id=[0,1,2]

cfg.save_mjb_and_task_config=True



if __name__=="__main__":


    print(discoverse.__logo__)

    np.set_printoptions(
        precision=3,
        suppress=True,
        linewidth=500
    )


    parser=argparse.ArgumentParser()

    parser.add_argument("--data_idx",type=int,default=0)
    parser.add_argument("--data_set_size",type=int,default=1)
    parser.add_argument("--auto",action="store_true")
    parser.add_argument("--use_gs",action="store_true")

    args=parser.parse_args()


    if not hasattr(args,"save_segment"):
        args.save_segment=False


    data_idx=args.data_idx
    data_set_size=args.data_idx+args.data_set_size


    if args.auto:

        cfg.headless=True
        cfg.sync=False


    cfg.use_gaussian_renderer=args.use_gs


    save_dir=os.path.join(
        DISCOVERSE_ROOT_DIR,
        "data",
        os.path.splitext(
            os.path.basename(__file__)
        )[0]
    )


    if not os.path.exists(save_dir):

        os.makedirs(save_dir)



    sim_node=SimNode(cfg)
    print("camera_names =", sim_node.camera_names)



    if cfg.save_mjb_and_task_config and data_idx==0:

        mujoco.mj_saveModel(
            sim_node.mj_model,
            os.path.join(
                save_dir,
                os.path.basename(cfg.mjcf_file_path).replace(".xml",".mjb")
            )
        )

        copypy2(
            os.path.abspath(__file__),
            os.path.join(save_dir,os.path.basename(__file__))
        )



    arm_ik=PiperIK(
        sim_node.mj_model,
        eef_site_name="endpoint",
        arm_dof=6
    )


    trmat=Rotation.from_euler(
        "xyz",
        [0,np.pi/2,0],
        degrees=False
    ).as_matrix()


    target_rot_world=get_site_tmat(
        sim_node.mj_data,
        "armbase"
    )[:3,:3]@trmat



    stm=SimpleStateMachine()

    stm.max_state_cnt=9


    max_time=10.0


    action=np.zeros(7)


    move_speed=0.75

    # [MOD-JSTATE] obs 안에 joint state가 실제로 들어오는지 딱 한 번만 확인하기 위한 플래그
    # [MOD-JSTATE] 반복 출력되면 터미널이 너무 지저분해지므로 1회만 출력
    has_printed_obs_debug = False


    sim_node.reset()



    while sim_node.running:



        if sim_node.reset_sig:

            sim_node.reset_sig=False

            stm.reset()

            action[:]=sim_node.target_control[:]

            act_lst=[]
            obs_lst=[]

            # [MOD-SAVE] 최종 저장 폴더가 아니라 임시 폴더에 먼저 저장
            final_save_path=os.path.join(
                save_dir,
                "{:03d}".format(data_idx)
            )

            save_path=os.path.join(
                save_dir,
                "_tmp_{:03d}".format(data_idx)
            )

            # [MOD-SAVE] 이전 실패 흔적이 남아있으면 제거
            if os.path.exists(save_path):
                shutil.rmtree(save_path)

            os.makedirs(save_path,exist_ok=True)


            encoders={

                cam_id:PyavImageEncoder(
                    cfg.render_set["width"],
                    cfg.render_set["height"],
                    save_path,
                    cam_id
                )

                for cam_id in cfg.obs_rgb_cam_id
            }



        try:



            if stm.trigger():


                if stm.state_idx==0:

                    tmat_block=get_body_tmat(
                        sim_node.mj_data,
                        "block_green"
                    )

                    # 접근 높이
                    tmat_block[:3,3]+=np.array([0,0,0.08])

                    sim_node.target_control[:6]=arm_ik.properIK(

                        sim_node.mj_data,
                        tmat_block[:3,3],
                        target_rot_world,
                        sim_node.mj_data.qpos[:6]

                    )

                    sim_node.target_control[6]=0.04



                elif stm.state_idx==1:

                    tmat_block=get_body_tmat(
                        sim_node.mj_data,
                        "block_green"
                    )

                    # 실제 grasp 높이 (핵심 수정)
                    tmat_block[:3,3]+=np.array([0,0,0.017])

                    sim_node.target_control[:6]=arm_ik.properIK(

                        sim_node.mj_data,
                        tmat_block[:3,3],
                        target_rot_world,
                        sim_node.mj_data.qpos[:6]

                    )



                elif stm.state_idx==2:

                    # 그리퍼 닫기
                    sim_node.target_control[6]=0.012



                elif stm.state_idx==3:

                    # grasp 안정화 대기
                    sim_node.delay_cnt=int(
                        0.6/sim_node.delta_t
                    )



                elif stm.state_idx==4:

                    cur_ep=get_site_tmat(
                        sim_node.mj_data,
                        "endpoint"
                    )

                    target_pos=cur_ep[:3,3].copy()

                    target_pos[2]+=0.10

                    sim_node.target_control[:6]=arm_ik.properIK(

                        sim_node.mj_data,
                        target_pos,
                        target_rot_world,
                        sim_node.mj_data.qpos[:6]

                    )



                elif stm.state_idx==5:

                    tmat_plate=get_body_tmat(
                        sim_node.mj_data,
                        "bowl_pink"
                    )

                    target_pos=tmat_plate[:3,3]+np.array([0,0,0.13])

                    sim_node.target_control[:6]=arm_ik.properIK(

                        sim_node.mj_data,
                        target_pos,
                        target_rot_world,
                        sim_node.mj_data.qpos[:6]

                    )



                elif stm.state_idx==6:

                    cur_ep=get_site_tmat(
                        sim_node.mj_data,
                        "endpoint"
                    )

                    target_pos=cur_ep[:3,3].copy()

                    target_pos[2]-=0.02

                    sim_node.target_control[:6]=arm_ik.properIK(

                        sim_node.mj_data,
                        target_pos,
                        target_rot_world,
                        sim_node.mj_data.qpos[:6]

                    )



                elif stm.state_idx==7:

                    sim_node.target_control[6]=0.04



                elif stm.state_idx==8:

                    cur_ep=get_site_tmat(
                        sim_node.mj_data,
                        "endpoint"
                    )

                    target_pos=cur_ep[:3,3].copy()

                    target_pos[2]+=0.05

                    sim_node.target_control[:6]=arm_ik.properIK(

                        sim_node.mj_data,
                        target_pos,
                        target_rot_world,
                        sim_node.mj_data.qpos[:6]

                    )



                dif=np.abs(action-sim_node.target_control)

                sim_node.joint_move_ratio=dif/(np.max(dif)+1e-6)



            elif sim_node.mj_data.time>max_time:

                raise ValueError("timeout")



            else:

                stm.update()



            if sim_node.checkActionDone():

                stm.next()



        except ValueError:

            # [MOD-SAVE] 예외 발생 시 임시 폴더 정리
            for encoder in encoders.values():
                try:
                    encoder.close()
                except Exception:
                    pass

            if os.path.exists(save_path):
                shutil.rmtree(save_path, ignore_errors=True)

            sim_node.reset()



        for i in range(sim_node.nj-1):

            action[i]=step_func(

                action[i],
                sim_node.target_control[i],
                move_speed*
                sim_node.joint_move_ratio[i]*
                sim_node.delta_t

            )


        action[6]=sim_node.target_control[6]


        obs,_,_,_,_=sim_node.step(action)

        # [MOD-JSTATE] observation 안에 어떤 키가 들어오는지 1회 출력
        # [MOD-JSTATE] 여기서 "jq"가 보이면 그게 joint position state임
        # [MOD-JSTATE] "jv"는 joint velocity, "jf"는 joint force일 가능성이 큼
        if not has_printed_obs_debug:
            print("\n[MOD-JSTATE] obs.keys() =", list(obs.keys()))
            print("[MOD-JSTATE] obs.get('jq') =", obs.get("jq", None))
            print("[MOD-JSTATE] obs.get('jv') =", obs.get("jv", None))
            print("[MOD-JSTATE] obs.get('jf') =", obs.get("jf", None))
            has_printed_obs_debug = True



        if len(obs_lst)<sim_node.mj_data.time*cfg.render_set["fps"]:

            imgs=obs.pop('img')

            for cam_id,img in imgs.items():

                encoders[cam_id].encode(img,obs["time"])


            act_lst.append(
                action.tolist().copy()
            )

            obs_lst.append(obs)



        if stm.state_idx>=stm.max_state_cnt:


            if sim_node.check_success():

                recoder_airbot_play(
                    save_path,
                    act_lst,
                    obs_lst,
                    cfg
                )


                for encoder in encoders.values():

                    encoder.close()

                # [MOD-SAVE] 성공했을 때만 최종 폴더로 확정
                if os.path.exists(final_save_path):
                    shutil.rmtree(final_save_path, ignore_errors=True)

                os.replace(save_path, final_save_path)


                data_idx+=1


                if data_idx>=data_set_size:

                    break


            else:

                print("Failed")

                # [MOD-SAVE] 실패했으면 임시 폴더 제거
                for encoder in encoders.values():
                    try:
                        encoder.close()
                    except Exception:
                        pass

                if os.path.exists(save_path):
                    shutil.rmtree(save_path, ignore_errors=True)


            sim_node.reset()
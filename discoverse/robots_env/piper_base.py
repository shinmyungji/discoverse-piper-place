import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from discoverse.envs import SimulatorBase
from discoverse.utils.base_config import BaseConfig


class PiperCfg(BaseConfig):
    # [MOD] Airbot 기본 MJCF 대신 Piper 기본 MJCF 사용
    mjcf_file_path = "mjcf/manipulator/robot_piper.xml"
    decimation     = 4
    timestep       = 0.005
    sync           = True
    headless       = False
    render_set     = {
        "fps"    : 30,
        "width"  : 1280,
        "height" : 720,
    }

    # [MOD] Piper는 6축 + gripper 1축 = 7
    init_qpos = np.zeros(7)

    obs_rgb_cam_id = None
    use_gaussian_renderer = False

    # [MOD] GS는 일단 비워도 됨
    # --use_gs 안 쓰면 문제 없음
    gs_model_dict = {}


class PiperBase(SimulatorBase):
    def __init__(self, config: PiperCfg):
        # [MOD] Piper도 7차원 제어(6 arm + 1 gripper)
        self.nj = 7
        super().__init__(config)

    def post_load_mjcf(self):
        try:
            if hasattr(self.config, "init_qpos") and self.config.init_qpos is not None:
                assert len(self.config.init_qpos) == self.nj, "init_qpos length must match the number of joints"
                self.init_joint_pose = np.array(self.config.init_qpos)
                self.init_joint_ctrl = self.init_joint_pose.copy()
            else:
                raise KeyError("init_qpos not found in config")
        except KeyError:
            self.init_joint_pose = np.zeros(self.nj)
            self.init_joint_ctrl = np.zeros(self.nj)

        # [MOD] 네가 piper_sensor.xml에 추가한 sensor 순서 기준
        self.sensor_joint_qpos = self.mj_data.sensordata[:self.nj]
        self.sensor_joint_qvel = self.mj_data.sensordata[self.nj:2*self.nj]
        self.sensor_joint_force = self.mj_data.sensordata[2*self.nj:3*self.nj]

        self.sensor_endpoint_posi_local = self.mj_data.sensordata[3*self.nj:3*self.nj+3]
        self.sensor_endpoint_quat_local = self.mj_data.sensordata[3*self.nj+3:3*self.nj+7]
        self.sensor_endpoint_linear_vel_local = self.mj_data.sensordata[3*self.nj+7:3*self.nj+10]
        self.sensor_endpoint_gyro = self.mj_data.sensordata[3*self.nj+10:3*self.nj+13]
        self.sensor_endpoint_acc = self.mj_data.sensordata[3*self.nj+13:3*self.nj+16]

    def printMessage(self):
        print("-" * 100)
        print("mj_data.time  = {:.3f}".format(self.mj_data.time))
        print("    arm .qpos  = {}".format(np.array2string(self.sensor_joint_qpos, separator=', ')))
        print("    arm .qvel  = {}".format(np.array2string(self.sensor_joint_qvel, separator=', ')))
        print("    arm .ctrl  = {}".format(np.array2string(self.mj_data.ctrl[:self.nj], separator=', ')))
        print("    arm .force = {}".format(np.array2string(self.sensor_joint_force, separator=', ')))
        print("    sensor end posi  = {}".format(np.array2string(self.sensor_endpoint_posi_local, separator=', ')))
        print("    sensor end euler = {}".format(
            np.array2string(
                Rotation.from_quat(self.sensor_endpoint_quat_local[[1, 2, 3, 0]]).as_euler("xyz"),
                separator=', '
            )
        ))

    def resetState(self):
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:self.nj] = self.init_joint_pose.copy()
        self.mj_data.ctrl[:self.nj] = self.init_joint_ctrl.copy()
        mujoco.mj_forward(self.mj_model, self.mj_data)

    def updateControl(self, action):
        if self.mj_data.qpos[self.nj-1] < 0.0:
            self.mj_data.qpos[self.nj-1] = 0.0
        self.mj_data.ctrl[:self.nj] = np.clip(
            action[:self.nj],
            self.mj_model.actuator_ctrlrange[:self.nj, 0],
            self.mj_model.actuator_ctrlrange[:self.nj, 1]
        )

    def checkTerminated(self):
        return False

    def getObservation(self):
        self.obs = {
            "time": self.mj_data.time,
            "jq": self.sensor_joint_qpos.tolist(),
            "jv": self.sensor_joint_qvel.tolist(),
            "jf": self.sensor_joint_force.tolist(),
            "ep": self.sensor_endpoint_posi_local.tolist(),
            "eq": self.sensor_endpoint_quat_local.tolist(),
            "img": self.img_rgb_obs_s.copy(),
            "depth": self.img_depth_obs_s.copy()
        }
        return self.obs

    def getPrivilegedObservation(self):
        return self.obs

    def getReward(self):
        return None
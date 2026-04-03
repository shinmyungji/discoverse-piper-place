import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


class PiperIK:
    def __init__(self, mj_model, eef_site_name="endpoint", arm_dof=6):
        self.mj_model = mj_model
        self.arm_dof = arm_dof
        self.eef_site_id = mujoco.mj_name2id(
            mj_model,
            mujoco.mjtObj.mjOBJ_SITE,
            eef_site_name
        )
        if self.eef_site_id < 0:
            raise ValueError(f"site '{eef_site_name}' not found")

    def properIK(self, mj_data, target_pos, target_rot, ref_q=None,
                 max_iter=80, damping=1e-4, step_scale=0.8):
        if ref_q is None:
            q = mj_data.qpos[:self.arm_dof].copy()
        else:
            q = ref_q.copy()

        tmp_data = mujoco.MjData(self.mj_model)
        tmp_data.qpos[:] = mj_data.qpos.copy()
        tmp_data.qvel[:] = 0
        tmp_data.qpos[:self.arm_dof] = q
        mujoco.mj_forward(self.mj_model, tmp_data)

        for _ in range(max_iter):
            cur_pos = tmp_data.site_xpos[self.eef_site_id].copy()
            cur_rot = tmp_data.site_xmat[self.eef_site_id].reshape(3, 3).copy()

            pos_err = target_pos - cur_pos
            rot_err_mat = target_rot @ cur_rot.T
            rot_err = Rotation.from_matrix(rot_err_mat).as_rotvec()

            if np.linalg.norm(pos_err) < 2e-3 and np.linalg.norm(rot_err) < 5e-2:
                break

            jacp = np.zeros((3, self.mj_model.nv))
            jacr = np.zeros((3, self.mj_model.nv))
            mujoco.mj_jacSite(self.mj_model, tmp_data, jacp, jacr, self.eef_site_id)

            J = np.vstack([jacp[:, :self.arm_dof], jacr[:, :self.arm_dof]])
            err = np.concatenate([pos_err, rot_err])

            A = J @ J.T + damping * np.eye(6)
            dq = J.T @ np.linalg.solve(A, err)

            tmp_data.qpos[:self.arm_dof] += step_scale * dq

            for j in range(self.arm_dof):
                low, high = self.mj_model.jnt_range[j]
                tmp_data.qpos[j] = np.clip(tmp_data.qpos[j], low, high)

            mujoco.mj_forward(self.mj_model, tmp_data)

        return tmp_data.qpos[:self.arm_dof].copy()
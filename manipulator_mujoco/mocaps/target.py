import mujoco
import numpy as np

class Target(object):
    """
    A class representing a mocap target.
    """

    def __init__(self, model, data, mocap_body_name="mocap"):
        """
        Args:
            model: The MjModel object.
            data: The MjData object.
            mocap_body_name: The name of the mocap body.
        """
        self._model = model
        self._data = data
        self._mocap_body_name = mocap_body_name
        self._mocap_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, mocap_body_name)

    @property
    def mocap_id(self):
        """Returns the mocap body id."""
        return self._mocap_id

    def set_mocap_pose(self, data, position=None, quaternion=None):
        """
        Sets the pose of the mocap body.
        """
        if position is not None:
            data.mocap_pos[self._mocap_id][:] = position
        if quaternion is not None:
            # Convert [x, y, z, w] → [w, x, y, z]
            quat = np.roll(np.array(quaternion), 1)
            data.mocap_quat[self._mocap_id][:] = quat

    def get_mocap_pose(self, data):
        """
        Returns pose [x, y, z, qw, qx, qy, qz] in MuJoCo convention.
        """
        position = data.mocap_pos[self._mocap_id]
        quaternion = data.mocap_quat[self._mocap_id]
        # Convert [w, x, y, z] → [x, y, z, w]
        quaternion = np.roll(quaternion, -1)
        return np.concatenate([position, quaternion])

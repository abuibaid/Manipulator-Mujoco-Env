import numpy as np

class JointEffortController:
    def __init__(
        self,
        model,
        data,
        joints,
        min_effort: np.ndarray,
        max_effort: np.ndarray,
    ) -> None:
        self._model = model
        self._data = data
        self._joints = joints
        self._min_effort = min_effort
        self._max_effort = max_effort

        # Get the DOF indices for the specified joints
        # self._jnt_dof_ids = [int(self._model.joint(joint_name).dofadr) for joint_name in self._joints]
        self._jnt_dof_ids = np.concatenate([
            np.atleast_1d(model.joint(joint_name).dofadr)
            for joint_name in joints
        ]).tolist()

        # print(f"Dof indices: {self._jnt_dof_ids}")
        # print(f"Dof shape: {np.shape(self._jnt_dof_ids)}")

    def run(self, target) -> None:
        """
        Run the robot controller.

        Parameters:
            target (numpy.ndarray): The desired target joint efforts for the robot.
        """
        # Clip the target efforts to ensure they are within the allowable effort range
        target_effort = np.clip(target, self._min_effort, self._max_effort)

        # Set the control signals for the actuators to the desired target joint efforts
        self._data.qfrc_applied[self._jnt_dof_ids] = target_effort

    def reset(self) -> None:
        pass
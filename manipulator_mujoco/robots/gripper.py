import mujoco

class Gripper:
    def __init__(self, xml_path, joint_name, actuator_name, name: str = None):
        # Load the model from XML
        self._model = mujoco.MjSpec.from_file(xml_path)
        if name:
            self._model.name = name

        # Store joint and actuator names
        self._joint_name = joint_name
        self._actuator_name = actuator_name

        # Optionally, collect all body names/ids
        # self._body_names = [b.name for b in self._model.worldbody.body.values()]

    @property
    def joint(self):
        """Returns the joint index belonging to the gripper."""
        return self._joint_id

    @property
    def actuator(self):
        """Returns the actuator index belonging to the gripper."""
        return self._actuator_id

    @property
    def mjcf_model(self):
        """Returns the MjModel object corresponding to this gripper."""
        return self._model

    @property
    def data(self):
        """Returns the MjData object corresponding to this gripper."""
        return self._data
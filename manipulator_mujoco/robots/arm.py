import mujoco

class Arm:
    def __init__(self, xml_path, attachment_site_name, joint_names=None, name: str = None):
        self._mjcf_model = mujoco.MjSpec.from_file(xml_path)
        if name:
            self._mjcf_model.name = name

        self._eef_site_name = attachment_site_name
        self._attachment_site_name = attachment_site_name
        self._joint_names = joint_names  # may be None

    def attach_tool(self, tool_mjcf_model, pos=[0, 0, 0], quat=[1, 0, 0, 0]):
        """
        Attach the tool MJCF to the attachment site in the MJCF tree (before MjModel is created).
        """
        site = self._mjcf_model.site(self._attachment_site_name)
        self._mjcf_model.attach(tool_mjcf_model, prefix="gripper/", site=site)

    @property
    def mjcf_model(self):
        return self._mjcf_model

    @property
    def joints(self):
        return self._joint_names

    @property
    def eef_site(self):
        return self._eef_site_name  # used later when building MjModel

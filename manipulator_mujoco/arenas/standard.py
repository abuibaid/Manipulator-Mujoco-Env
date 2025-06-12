
import mujoco as mj
import os

class StandardArena(object):
    def __init__(self):
        """
        Creates an MJCF root with a checkerboard floor, basic lighting,
        and allows for robot, gripper, and mocap attachment.
        """
        # self._mjcf_model = MjSpec()
        xml_path = os.path.join(os.path.dirname(__file__), "../assets/world/world.xml")
        self._mjcf_model = mj.MjSpec.from_file(xml_path)
        # # World options
        # self._mjcf_model.option.timestep = 0.002

        #        # Add texture and material
        # checker_tex = self._mjcf_model.add_texture(
        #     name="checker",
        #     type=mujoco.mjtTexture.mjTEXTURE_2D,
        #     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        #     width=300,
        #     height=300,
        #     rgb1=[0.2, 0.3, 0.4],
        #     rgb2=[0.3, 0.4, 0.5],
        # )

        # grid_mat = self._mjcf_model.add_material(
        #     name="grid",
        #     texrepeat=[5, 5],
        #     reflectance=0.2,
        # ).textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = checker_tex.name

        # # Add floor geom
        # self._mjcf_model.worldbody.add_geom(
        #     name="floor",
        #     type=mujoco.mjtGeom.mjGEOM_PLANE,
        #     size=[2, 2, 0.1],
        #     material="grid",
        # )

        # # Add lights
        # self._mjcf_model.worldbody.add_light(
        #     name="light1",
        #     pos=[-2, -1, 3],
        #     dir=[2, 1, -2],
        # )

        # self._mjcf_model.worldbody.add_light(
        #     name="light2",
        #     pos=[2, -1, 3],
        #     dir=[-2, 1, -2],
        # )

        # Prepare reference handles for attachment
        self._robot_mount = None  # Where robot will be attached
        self._mocap_target = None

    def attach_robot(self, robot_spec: mj.MjSpec, pos=[0, 0, 0], quat=[0.7071, 0, 0, -0.7071]):
        site = self._mjcf_model.worldbody.add_site(
        name="robot_attachment_site",
        pos=pos,
        quat=quat,
        size=[0.001, 0.001, 0.001],
        )

        self._mjcf_model.attach(robot_spec, site=site)
        self._robot_mount = site.parent

    def attach_gripper(self, gripper_spec: mj.MjSpec, pos=[0, 0, 0], quat=[0, 0, 0, 1]):
        """
        Attaches a gripper MJCF model to the robot end-effector.
        """
        if self._robot_mount is None:
            raise RuntimeError("Attach robot first before attaching gripper.")

        # Create a site to serve as attachment point
        site = self._robot_mount.add_site(
            name="gripper_attachment_site",
            pos=pos,
            quat=quat,
            size=[0.001, 0.001, 0.001],
            type=mj.mjtGeom.mjGEOM_SPHERE,  # optional visual marker
        )

        # Attach the gripper MJCF using the site
        self._mjcf_model.attach(gripper_spec, site=site)

    def attach_mocap(self, name="target", pos=[0.5, 0, 1.2], quat=[0, 0, 0, 1]):
        """
        Adds a mocap body for target tracking.
        """
        mocap_body = self._mjcf_model.worldbody.add_body(
            name=name,
            mocap=True,
            pos=pos,
            quat=quat,
        )
        mocap_body.add_geom(
            type=mj.mjtGeom.mjGEOM_SPHERE,
            size=[0.02, 0.02, 0.02],  # must be 3D vector
            rgba=[1, 0, 0, 1],
            contype=0,
            conaffinity=0,
        )
        self._mocap_target = mocap_body


    @property
    def mjcf_model(self) -> mj.MjSpec:
        return self._mjcf_model

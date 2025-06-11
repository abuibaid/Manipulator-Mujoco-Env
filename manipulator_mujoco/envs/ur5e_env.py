import time
import numpy as np
import mujoco
import mujoco.viewer
import gymnasium as gym
from gymnasium import spaces

from manipulator_mujoco.arenas import StandardArena
from manipulator_mujoco.robots import UR5e, Robotiq2f85
from manipulator_mujoco.mocaps import Target
from manipulator_mujoco.controllers import OperationalSpaceController
from loop_rate_limiters import RateLimiter
from manipulator_mujoco.mujoco.mujoco_utils_ import MujocoModelNames

class UR5eEnv(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array", "depth_array"],
        "render_fps": None,
    }

    def __init__(self, render_mode=None):
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        )
        self.action_space = spaces.Box(
            low=-0.1, high=0.1, shape=(6,), dtype=np.float64
        )

        assert render_mode in self.metadata["render_modes"] or render_mode is None
        self._render_mode = render_mode

        ############################
        # Create MJCF model tree
        ############################

        # 1. Create arena (MjSpec root)
        self._arena = StandardArena()

        # 2. Create arm
        self._arm = UR5e()

        # 3. Create gripper
        self._gripper = Robotiq2f85()

        # 4. Attach gripper to arm
        self._arm.attach_tool(self._gripper.mjcf_model, pos=[0, 0, 0], quat=[0, 0, 0, 1])

        # 5. Attach arm to arena
        self._arena.attach_robot(self._arm.mjcf_model,pos=[0, 0, 0], quat=[0, 0, 0, 1])
        
        # 6. Attach mocap target
        self._arena.attach_mocap(pos=[-0.5, 0, 1.2], quat=[0, 0, 0, 1])

        # 7. Convert full MjSpec to XML, then to MjModel
        self._model = self._arena.mjcf_model.compile()
        self._data = mujoco.MjData(self._model)

        self._target = Target(self._model, self._data)

        # Operational Space Controller
        self._controller = OperationalSpaceController(
            model=self._model,
            data=self._data,
            joints=self._arm.joints,
            eef_site=self._arm.eef_site,
            min_effort=-150.0,
            max_effort=150.0,
            kp=200,
            ko=200,
            kv=50,
            vmax_xyz=1.0,
            vmax_abg=2.0,
        )

        self._timestep = self._model.opt.timestep
        self._viewer = None
        self._step_start = None
        self.rate = RateLimiter(frequency=1/self._timestep, warn=False)

        self.model_names = MujocoModelNames(self._model) 

        # self.init_qpos_config = {
        #     "shoulder_pan_joint": 0,
        #     "shoulder_lift_joint": -np.pi / 2.0,
        #     "elbow_joint": -np.pi / 2.0,
        #     "wrist_1_joint": -np.pi / 2.0,
        #     "wrist_2_joint": np.pi / 2.0,
        #     "wrist_3_joint": 0,
        # }
        # for joint_name, joint_pos in self.init_qpos_config.items():
        #     joint_id = self.model_names.joint_name2id[joint_name]
        #     qpos_id = self._model.jnt_qposadr[joint_id]
        #     self.init_qpos[qpos_id] = joint_pos

        # print(f"UR5eEnv initialized with model names: {self.model_names}")
        # print(f"Model names: {self.model_names._joint_names}")
        # print(f"model body names: {self.model_names._body_names}")
        # print(f"joint name to id mapping: {self.model_names.joint_name2id}")
        # print(f"site name to id mapping: {self.model_names.site_name2id}")
        # print(f"mocap body name to id mapping: {self._target.mocap_id}")
        # # print(f"Initial joint positions: {self.init_qpos_config}")
        # print(f"actuators:{self.model_names.actuator_names}")
        # print(f"actuator name to id mapping: {self.model_names.actuator_name2id}")
        # print(f"camera names: {self.model_names.camera_names}")

    def _get_obs(self) -> np.ndarray:
        # TODO: Replace with meaningful observation
        # print(f"robot joint positions: {self._data.qpos[:6].copy()}")
        return self._data.qpos[:6].copy()

    def _get_info(self) -> dict:
        return {}

    def reset(self, seed=None, options=None) -> tuple:
        super().reset(seed=seed)

        mujoco.mj_resetData(self._model, self._data)

        # Set initial robot joint position
        # Set only the robot arm joint positions (first 6 joints)
        self._data.qpos[:6] = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        
        self._data.qpos[6] = np.array([0])

        mujoco.mj_forward(self._model, self._data)

        self._target.set_mocap_pose(self._data, position=[-0.5, 0, 1.2], quaternion=[-1, 1, 0, 0])
        return self._get_obs(), self._get_info()

    def step(self, action: np.ndarray) -> tuple:
        # Run OSC to follow target
        target_pose = self._target.get_mocap_pose(self._data)
        # print(f"Target Pose: {target_pose}")
        self._controller.run(target_pose)

        # Advance simulation
        mujoco.mj_step(self._model, self._data)

        if self._render_mode == "human":
            self._render_frame()

        observation = self._get_obs()
        reward = 0
        terminated = False
        truncated = False
        return observation, reward, terminated, truncated, self._get_info()

    def render(self) -> np.ndarray:
        """
        Renders the current frame and returns it as an RGB array if the render mode is set to "rgb_array".

        Returns:
            np.ndarray: RGB array of the current frame.
        """
        if self._render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        """
        Renders the current frame and updates the viewer if the render mode is set to "human".
        """

        if self._viewer is None and self._render_mode == "human":
            # launch viewer
            self._viewer = mujoco.viewer.launch_passive(
                model=self._model,
                data=self._data,
                show_left_ui=False,
                show_right_ui=False,
            )
            mujoco.mjv_defaultFreeCamera(self._model, self._viewer.cam)
        if self._step_start is None and self._render_mode == "human":
            # initialize step timer
            self._step_start = time.time()

        if self._render_mode == "human":
            # render viewer
            self._viewer.sync()

            # TODO come up with a better frame rate keeping strategy
            # time_until_next_step = self._timestep - (time.time() - self._step_start)
            # if time_until_next_step > 0:
            #     time.sleep(time_until_next_step)
            # self._step_start = time.time()
            self.rate.sleep()

        else:  # rgb_array
            return mujoco.mj_render(self._model, self._data)

    def close(self):
        """
        Closes the viewer if it's open.
        """
        if self._viewer is not None:
            self._viewer.close()


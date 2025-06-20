import os
import random
from pathlib import Path

import gym
from gym import spaces
import numpy as np
import mujoco

from ..utils import update_mjcf


class ActiveSuspensionEnv(gym.Env):
    """Mujoco environment for active suspension control."""

    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode=None, domain_randomization=True, n_bump=5):
        super().__init__()
        self.render_mode = render_mode
        self.domain_randomization = domain_randomization
        self.n_bump = n_bump

        repo_root = Path(__file__).resolve().parent.parent
        self.models_dir = repo_root / "models"
        self.scene_xml_path = self.models_dir / "generated_scene.xml"

        # observation: [acc_z, pitch_rate, suspension_pos(4), suspension_vel(4)]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(10,),
            dtype=np.float32,
        )

        # actions correspond to 4 suspension actuators
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

        self.model = None
        self.data = None
        self.viewer = None

    # ------------------------------------------------------------------
    def _load_model(self):
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_xml_path))
        self.data = mujoco.MjData(self.model)
        if self.render_mode == "human" and self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    # ------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if self.domain_randomization:
            update_mjcf.insert_stl_to_mjcf(
                base_xml_path=str(self.models_dir / "base_scene.xml"),
                output_xml_path=str(self.scene_xml_path),
                n_bump=self.n_bump,
                random_seed=random.randint(0, 1_000_000),
            )

        self._load_model()
        observation = self._get_obs()
        info = {"time": self.data.time}
        return observation, info

    # ------------------------------------------------------------------
    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        actuators = ["fl_spring", "fr_spring", "rl_spring", "rr_spring"]
        for i, name in enumerate(actuators):
            self.data.ctrl[self.model.actuator(name).id] = float(action[i])

        # constant motor torque to move forward
        torque = 100.0
        for name in ["fl_motor", "fr_motor", "rl_motor", "rr_motor"]:
            self.data.ctrl[self.model.actuator(name).id] = torque

        mujoco.mj_step(self.model, self.data)
        if self.viewer:
            self.viewer.sync()

        obs = self._get_obs()
        reward = self._compute_reward(obs, action)
        done = self._check_done(obs)
        info = {"time": self.data.time}
        return obs, reward, done, False, info

    # ------------------------------------------------------------------
    def _get_obs(self):
        acc_z = float(self.data.sensordata[self.model.sensor("accel_z").adr])
        pitch_rate = float(self.data.sensordata[self.model.sensor("gyro_pitch").adr])
        qpos_names = [
            "fl_suspension",
            "fr_suspension",
            "rl_suspension",
            "rr_suspension",
        ]
        qpos = np.array([self.data.joint(name).qpos[0] for name in qpos_names])
        qvel = np.array([self.data.joint(name).qvel[0] for name in qpos_names])
        obs = np.concatenate(([acc_z, pitch_rate], qpos, qvel)).astype(np.float32)
        return obs

    # ------------------------------------------------------------------
    def _compute_reward(self, obs, action):
        acc_z, pitch_rate = obs[0], obs[1]
        return -float(abs(acc_z) + abs(pitch_rate))

    # ------------------------------------------------------------------
    def _check_done(self, obs):
        return bool(self.data.time > 5.0)

    # ------------------------------------------------------------------
    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


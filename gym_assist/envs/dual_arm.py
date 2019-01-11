import os
import numpy as np

from gym import utils, spaces
from gym.envs.robotics.utils import robot_get_obs

from gym_assist.envs import assist_env


k_DOF = 16
# Environment Names
SUTURE_XML = os.path.join('suture', 'suture_body.xml')

class DualArmEnv(assist_env.AssistEnv, utils.EzPickle):
	def __init__(self, model_path, reward_type, initial_qpos={}, 
		randomize_initial_position=True, randomize_initial_rotation=True, 
		n_substeps=20, relative_control=False):
		
		self.randomize_initial_rotation = randomize_initial_rotation
		self.randomize_initial_position = randomize_initial_position
		self.relative_control = relative_control
		self.reward_type = reward_type

		assist_env.AssistEnv.__init__(self, model_path, n_substeps=n_substeps, initial_qpos=initial_qpos, n_actions=k_DOF)
		utils.EzPickle.__init__(self)

	# AssistEnv methods
	# ----------------------------
	def _get_obs(self):
		robot_qpos, robot_qvel = robot_get_obs(self.sim)
		object_qpos = self.sim.data.get_joint_qpos('object:joint')
		object_qvel = self.sim.data.get_joint_qvel('object:joint')
		#target_qpos, target_qvel # TODO: Fix this 
		observation = np.concatenate([robot_qpos, robot_qvel, object_qpos, object_qvel])
		return {
			'observation': observation.copy()
		}

	def _reset_sim(self):
		self.sim.set_state(self.initial_state)
		self.sim.forward()

		object_initial_qpos = self.sim.data.get_joint_qpos('object:joint')
		object_initial_pos, object_initial_quat = object_initial_qpos[:3], object_initial_qpos[3:]
		assert object_initial_qpos.shape == (7,)
		assert object_initial_pos.shape == (3,)
		assert object_initial_quat.shape == (4,)
		
		object_initial_qpos = None
		target_initial_qpos = None # TODO: Fix this

		# Randomize initial rotation
		axis = np.array([0., 0., 1.]) # around z-axis
		if self.randomize_initial_rotation:
			object_angle = self.np_random.uniform(-np.pi, np.pi)
			object_offset_quat = quat_from_angle_and_axis(angle, axis)
			object_initial_quat = rotations.quat_mul(object_initial_quat, object_offset_quat)

		# Randomize initial position
		if self.randomize_initial_position:
			object_initial_pos += self.np_random.normal(size=3, scale=0.005)

		object_initial_quat /= np.linalg.norm(object_initial_quat)
		object_initial_qpos = np.concatenate([object_initial_pos, object_initial_quat])
		
		self.sim.data.set_joint_qpos('object:joint', object_initial_qpos)

		# Run the simulation for a bunch of timesteps to let everything settle in.
		for _ in range(10):
			self._set_action(np.zeros(k_DOF))
			try:
				self.sim.step()
			except mujoco_py.MujocoException:
				return False
		return True # TODO: Fix this

	def _env_setup(self, initial_qpos):
		for name, value in initial_qpos.items():
			self.sim.data.set_joint_qpos(name, value)
		self.sim.forward()

	def _set_action(self, action):
		assert action.shape == (k_DOF,)

		ctrlrange = self.sim.model.actuator_ctrlrange
		actuation_range = (ctrlrange[:,1] - ctrlrange[:,0]) / 2.

		if self.relative_control:
			actuation_center = np.zeros_like(action)

			for i in range(self.sim.data.ctrl.shape[0]):
				actuation_center[i] = self.sim.data.get_joint_qpos(self.sim.model.actuator_names[i].replace(':A_', ':'))

			for joint_name in ['r_', 'l_']:
				act_idx = self.sim.model.actuator_name2id('robot0:A_{}J1'.format(joint_name))
				actuation_center[act_idx] += self.sim.data.get_joint_qpos('robot0:{}J0'.format(joint_name))
		else:
			actuation_center = (ctrlrange[:, 1] + ctrlrange[:,0]) / 2

		self.sim.data.ctrl[:] = actuation_center + action * actuation_range
		self.sim.data.ctrl[:] = np.clip(self.sim.data.ctrl, ctrlrange[:, 0], ctrlrange[:, 1])

	def _viewer_setup(self):
		lookat = (0.65, 0, 0)
		for idx, value in enumerate(lookat):
			self.viewer.cam.lookat[idx] = value
		self.viewer.cam.distance = 1.3
		self.viewer.cam.azimuth = 0
		self.viewer.cam.elevation = -50

	# Helper methods
	# ----------------------------
	def quat_from_angle_and_axis(angle, axis):
		assert axis.shape == (3,)
		axis /= np.linalg.norm(axis)
		quat = np.concatenate([[np.cos(angle / 2.)], np.sin(angle / 2.) * axis])
		quat /= np.linalg.norm(quat)
		return quat


class SutureEnv(DualArmEnv):
	def __init__(self, reward_type='sparse'):
		super(SutureEnv, self).__init__(
			model_path=SUTURE_XML, reward_type=reward_type, 
			randomize_initial_position=False, randomize_initial_rotation=False)
		
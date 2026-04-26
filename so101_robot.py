"""OmniGibson robot wrapper for the SO-101 arm."""

from __future__ import annotations

import math
import os

import torch as th

from omnigibson.robots.manipulation_robot import GraspingPoint, ManipulationRobot
from omnigibson.utils.transform_utils import euler2quat


REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
SO101_ROOT = os.path.join(REPO_ROOT, "assert", "SO101")
SO101_USD_PATH = os.path.join(
    SO101_ROOT,
    "so101_new_calib",
    "so101_new_calib_og_usdobject.usd",
)
SO101_URDF_PATH = os.path.join(SO101_ROOT, "so101_new_calib.urdf")
SO101_DESCRIPTOR_PATH = os.path.join(SO101_ROOT, "lula", "so101_robot_descriptor.yaml")


class SO101(ManipulationRobot):
    """SO-101 5-DOF robot arm with a single-actuated parallel gripper."""

    def __init__(
        self,
        # Shared kwargs in hierarchy
        name,
        relative_prim_path=None,
        scale=None,
        visible=True,
        visual_only=False,
        self_collisions=False,
        load_config=None,
        fixed_base=True,
        # Unique to USDObject hierarchy
        abilities=None,
        # Unique to ControllableObject hierarchy
        control_freq=None,
        controller_config=None,
        action_type="continuous",
        action_normalize=True,
        reset_joint_pos=None,
        # Unique to BaseRobot
        obs_modalities=("proprio",),
        proprio_obs="default",
        sensor_config=None,
        # Unique to ManipulationRobot
        grasping_mode="assisted",
        **kwargs,
    ):
        super().__init__(
            relative_prim_path=relative_prim_path,
            name=name,
            scale=scale,
            visible=visible,
            fixed_base=fixed_base,
            visual_only=visual_only,
            self_collisions=self_collisions,
            load_config=load_config,
            abilities=abilities,
            control_freq=control_freq,
            controller_config=controller_config,
            action_type=action_type,
            action_normalize=action_normalize,
            reset_joint_pos=reset_joint_pos,
            obs_modalities=obs_modalities,
            proprio_obs=proprio_obs,
            sensor_config=sensor_config,
            grasping_mode=grasping_mode,
            grasping_direction="upper",
            **kwargs,
        )

    @property
    def model_name(self):
        return "SO101"

    @property
    def usd_path(self):
        return SO101_USD_PATH

    @property
    def urdf_path(self):
        return SO101_URDF_PATH

    @property
    def robot_arm_descriptor_yamls(self):
        return {self.default_arm: SO101_DESCRIPTOR_PATH}

    @property
    def curobo_path(self):
        raise NotImplementedError("SO101 does not currently ship a cuRobo descriptor.")

    @property
    def eef_usd_path(self):
        raise NotImplementedError("SO101 does not currently ship a separate end-effector USD.")

    @property
    def discrete_action_list(self):
        raise NotImplementedError()

    def _create_discrete_action_space(self):
        raise ValueError("SO101 does not support discrete actions.")

    @property
    def controller_order(self):
        return [f"arm_{self.default_arm}", f"gripper_{self.default_arm}"]

    @property
    def _default_controllers(self):
        controllers = super()._default_controllers
        controllers[f"arm_{self.default_arm}"] = "JointController"
        controllers[f"gripper_{self.default_arm}"] = "MultiFingerGripperController"
        return controllers

    @property
    def default_joint_pos(self):
        return [0.0, -0.5, 1.0, 0.0, 0.0, 0.5]

    @property
    def _default_joint_pos(self):
        return th.tensor(self.default_joint_pos, dtype=th.float32)

    @property
    def arm_link_names(self):
        return {
            self.default_arm: [
                "base_link",
                "shoulder_link",
                "upper_arm_link",
                "lower_arm_link",
                "wrist_link",
                "gripper_link",
                "gripper_frame_link",
            ]
        }

    @property
    def arm_joint_names(self):
        return {
            self.default_arm: [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ]
        }

    @property
    def gripper_link_names(self):
        return {self.default_arm: ["gripper_link", "moving_jaw_so101_v1_link"]}

    @property
    def gripper_joint_names(self):
        return {self.default_arm: ["gripper"]}

    @property
    def eef_link_names(self):
        return {self.default_arm: "gripper_frame_link"}

    @property
    def finger_link_names(self):
        return self.gripper_link_names

    @property
    def finger_joint_names(self):
        return self.gripper_joint_names

    @property
    def assisted_grasp_finger_links(self):
        return self.finger_link_names

    @property
    def assisted_grasp_start_points(self):
        return {
            self.default_arm: [
                GraspingPoint(
                    link_name="moving_jaw_so101_v1_link",
                    position=th.tensor([0.0134, -0.0196, 0.0188], dtype=th.float32),
                ),
                GraspingPoint(
                    link_name="moving_jaw_so101_v1_link",
                    position=th.tensor([0.0, -0.015, 0.035], dtype=th.float32),
                ),
                GraspingPoint(
                    link_name="moving_jaw_so101_v1_link",
                    position=th.tensor([0.008, -0.031, -0.005], dtype=th.float32),
                ),
                GraspingPoint(
                    link_name="moving_jaw_so101_v1_link",
                    position=th.tensor([0.008, -0.031, 0.008], dtype=th.float32),
                ),
            ]
        }

    @property
    def assisted_grasp_end_points(self):
        return {
            self.default_arm: [
                GraspingPoint(
                    link_name="gripper_link",
                    position=th.tensor([-0.03, 0.0, -0.045], dtype=th.float32),
                ),
                GraspingPoint(
                    link_name="gripper_link",
                    position=th.tensor([0.0, 0.0, -0.08], dtype=th.float32),
                ),
                GraspingPoint(
                    link_name="gripper_link",
                    position=th.tensor([0.021, 0.024, -0.055], dtype=th.float32),
                ),
                GraspingPoint(
                    link_name="gripper_link",
                    position=th.tensor([0.021, 0.024, -0.070], dtype=th.float32),
                ),
            ]
        }

    @property
    def finger_lengths(self):
        return {self.default_arm: 0.035}

    @property
    def arm_workspace_range(self):
        return {self.default_arm: (-math.pi, math.pi)}

    @property
    def teleop_rotation_offset(self):
        return {self.default_arm: euler2quat([-math.pi, 0.0, 0.0])}

    @property
    def disabled_collision_pairs(self):
        return []

    def _establish_grasp_rigid(self, arm="default", ag_data=None, contact_pos=None):
        if ag_data is not None and contact_pos is None:
            _, ag_link = ag_data
            contact_pos = ag_link.get_position_orientation()[0]
        return super()._establish_grasp_rigid(arm=arm, ag_data=ag_data, contact_pos=contact_pos)

    @property
    def aabb(self):
        try:
            return super().aabb
        except RuntimeError as exc:
            if "expected a non-empty list of Tensors" not in str(exc):
                raise
            center = th.as_tensor(self.get_position_orientation()[0], dtype=th.float32)
            half_extent = th.tensor([0.25, 0.25, 0.25], dtype=th.float32)
            return center - half_extent, center + half_extent

    @property
    def _default_arm_joint_controller_configs(self):
        configs = super()._default_arm_joint_controller_configs
        for arm in self.arm_names:
            configs[arm]["command_output_limits"] = None
            configs[arm]["use_impedances"] = False
        return configs

    @property
    def _default_gripper_multi_finger_controller_configs(self):
        configs = super()._default_gripper_multi_finger_controller_configs
        for arm in self.arm_names:
            configs[arm]["open_qpos"] = [-0.174533]
            configs[arm]["closed_qpos"] = [1.74533]
        return configs

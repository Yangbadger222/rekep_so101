import os
import sys


# ---------------------------------------------------------------------------
# Bootstrap: configure environment before importing omnigibson / heavy deps so
# `python main.py --task trash` works out-of-the-box (mirrors the original
# ReKep entry point UX). Three things happen here, all best-effort:
#   1. Re-exec under the omnigibson conda env Python if we are not already
#      running inside it -- Isaac Sim only loads from that interpreter.
#   2. Default OMNIGIBSON_HEADLESS=1 unless the user explicitly opts in to
#      a window via `--gui` (or sets the env var beforehand).
#   3. Pull OPENAI_API_KEY from ~/.zshrc (codex-style JSON line) when it is
#      missing, so live VLM queries do not silently fail with a swallowed
#      RuntimeError after Isaac Kit has already started shutting down.
# ---------------------------------------------------------------------------
_REKEP_BOOTSTRAPPED = os.environ.get('REKEP_BOOTSTRAPPED') == '1'
if not _REKEP_BOOTSTRAPPED:
    _OG_PYTHON = '/home/badger/anaconda3/envs/omnigibson/bin/python'
    if os.path.exists(_OG_PYTHON) and os.path.realpath(sys.executable) != os.path.realpath(_OG_PYTHON):
        os.environ['REKEP_BOOTSTRAPPED'] = '1'
        os.execv(_OG_PYTHON, [_OG_PYTHON, os.path.abspath(__file__), *sys.argv[1:]])
    os.environ['REKEP_BOOTSTRAPPED'] = '1'

if '--gui' not in sys.argv and 'OMNIGIBSON_HEADLESS' not in os.environ:
    os.environ['OMNIGIBSON_HEADLESS'] = '1'

if not os.environ.get('OPENAI_API_KEY'):
    _zshrc = os.path.expanduser('~/.zshrc')
    if os.path.exists(_zshrc):
        try:
            import re
            with open(_zshrc, 'r') as _f:
                _m = re.search(r'"OPENAI_API_KEY"\s*:\s*"([^"]+)"', _f.read())
            if _m:
                os.environ['OPENAI_API_KEY'] = _m.group(1)
        except Exception:
            pass

import torch
import numpy as np
import json
import argparse
import omnigibson as og
from environment import ReKepOGEnv
from keypoint_proposal import KeypointProposer
from constraint_generation import ConstraintGenerator
from ik_solver import IKSolver
from subgoal_solver import SubgoalSolver
from path_solver import PathSolver
from visualizer import Visualizer
import transform_utils as T
from so101_robot import SO101
from utils import (
    bcolors,
    get_config,
    load_functions_from_txt,
    get_linear_interpolation_steps,
    spline_interpolate_poses,
    get_callable_grasping_cost_fn,
    print_opt_debug_dict,
)

class Main:
    def __init__(self, scene_file, visualize=False):
        global_config = get_config(config_path="./configs/config.yaml")
        self.config = global_config['main']
        self.bounds_min = np.array(self.config['bounds_min'])
        self.bounds_max = np.array(self.config['bounds_max'])
        self.visualize = visualize
        # set random seed
        np.random.seed(self.config['seed'])
        torch.manual_seed(self.config['seed'])
        torch.cuda.manual_seed(self.config['seed'])
        # initialize keypoint proposer and constraint generator
        self.keypoint_proposer = KeypointProposer(global_config['keypoint_proposer'])
        self.constraint_generator = ConstraintGenerator(global_config['constraint_generator'])
        # initialize environment
        self.env = ReKepOGEnv(global_config['env'], scene_file, verbose=False)
        # setup ik solver (for reachability cost)
        assert isinstance(self.env.robot, SO101), (
            f"The IK solver assumes the robot is an SO101 robot, got {type(self.env.robot).__name__}"
        )
        ik_solver = IKSolver(
            robot_description_path=self.env.robot.robot_arm_descriptor_yamls[self.env.robot.default_arm],
            robot_urdf_path=self.env.robot.urdf_path,
            eef_name=self.env.robot.eef_link_names[self.env.robot.default_arm],
            reset_joint_pos=self.env.reset_joint_pos,
            world2robot_homo=self.env.world2robot_homo,
        )
        # initialize solvers
        self.subgoal_solver = SubgoalSolver(global_config['subgoal_solver'], ik_solver, self.env.reset_joint_pos)
        self.path_solver = PathSolver(global_config['path_solver'], ik_solver, self.env.reset_joint_pos)
        # initialize visualizer
        if self.visualize:
            self.visualizer = Visualizer(global_config['visualizer'], self.env)

    def perform_task(self, instruction, rekep_program_dir=None, disturbance_seq=None):
        self.env.reset()
        cam_obs = self.env.get_cam_obs()
        rgb = cam_obs[self.config['vlm_camera']]['rgb']
        points = cam_obs[self.config['vlm_camera']]['points']
        mask = cam_obs[self.config['vlm_camera']]['seg']
        # ====================================
        # = keypoint proposal and constraint generation
        # ====================================
        if rekep_program_dir is None:
            keypoints, projected_img = self.keypoint_proposer.get_keypoints(rgb, points, mask)
            print(f'{bcolors.HEADER}Got {len(keypoints)} proposed keypoints{bcolors.ENDC}')
            if self.visualize:
                self.visualizer.show_img(projected_img)
            metadata = {'init_keypoint_positions': keypoints, 'num_keypoints': len(keypoints)}
            rekep_program_dir = self.constraint_generator.generate(projected_img, instruction, metadata)
            print(f'{bcolors.HEADER}Constraints generated{bcolors.ENDC}')
        # ====================================
        # = execute
        # ====================================
        self._execute(rekep_program_dir, disturbance_seq)

    def _update_disturbance_seq(self, stage, disturbance_seq):
        if disturbance_seq is not None:
            if stage in disturbance_seq and not self.applied_disturbance[stage]:
                # set the disturbance sequence, the generator will yield and instantiate one disturbance function for each env.step until it is exhausted
                self.env.disturbance_seq = disturbance_seq[stage](self.env)
                self.applied_disturbance[stage] = True

    def _execute(self, rekep_program_dir, disturbance_seq=None):
        # load metadata
        with open(os.path.join(rekep_program_dir, 'metadata.json'), 'r') as f:
            self.program_info = json.load(f)
        self.applied_disturbance = {stage: False for stage in range(1, self.program_info['num_stages'] + 1)}
        # register keypoints to be tracked
        self.env.register_keypoints(self.program_info['init_keypoint_positions'])
        # load constraints
        self.constraint_fns = dict()
        for stage in range(1, self.program_info['num_stages'] + 1):  # stage starts with 1
            stage_dict = dict()
            for constraint_type in ['subgoal', 'path']:
                load_path = os.path.join(rekep_program_dir, f'stage{stage}_{constraint_type}_constraints.txt')
                get_grasping_cost_fn = get_callable_grasping_cost_fn(self.env)  # special grasping function for VLM to call
                stage_dict[constraint_type] = load_functions_from_txt(load_path, get_grasping_cost_fn) if os.path.exists(load_path) else []
            self.constraint_fns[stage] = stage_dict
        
        # bookkeeping of which keypoints can be moved in the optimization
        self.keypoint_movable_mask = np.zeros(self.program_info['num_keypoints'] + 1, dtype=bool)
        self.keypoint_movable_mask[0] = True  # first keypoint is always the ee, so it's movable

        # main loop
        self.last_sim_step_counter = -np.inf
        self._update_stage(1)
        while True:
            scene_keypoints = self.env.get_keypoint_positions()
            self.keypoints = np.concatenate([[self.env.get_ee_pos()], scene_keypoints], axis=0)  # first keypoint is always the ee
            self.curr_ee_pose = self.env.get_ee_pose()
            self.curr_joint_pos = self.env.get_arm_joint_postions()
            self.sdf_voxels = self.env.get_sdf_voxels(self.config['sdf_voxel_size'])
            self.collision_points = self.env.get_collision_points()
            # ====================================
            # = decide whether to backtrack
            # ====================================
            backtrack = False
            if self.stage > 1:
                path_constraints = self.constraint_fns[self.stage]['path']
                for constraints in path_constraints:
                    violation = constraints(self.keypoints[0], self.keypoints[1:])
                    if violation > self.config['constraint_tolerance']:
                        backtrack = True
                        break
            if backtrack:
                # determine which stage to backtrack to based on constraints
                for new_stage in range(self.stage - 1, 0, -1):
                    path_constraints = self.constraint_fns[new_stage]['path']
                    # if no constraints, we can safely backtrack
                    if len(path_constraints) == 0:
                        break
                    # otherwise, check if all constraints are satisfied
                    all_constraints_satisfied = True
                    for constraints in path_constraints:
                        violation = constraints(self.keypoints[0], self.keypoints[1:])
                        if violation > self.config['constraint_tolerance']:
                            all_constraints_satisfied = False
                            break
                    if all_constraints_satisfied:   
                        break
                print(f"{bcolors.HEADER}[stage={self.stage}] backtrack to stage {new_stage}{bcolors.ENDC}")
                self._update_stage(new_stage)
            else:
                # apply disturbance
                self._update_disturbance_seq(self.stage, disturbance_seq)
                # ====================================
                # = get optimized plan
                # ====================================
                if self.last_sim_step_counter == self.env.step_counter:
                    print(f"{bcolors.WARNING}sim did not step forward within last iteration (HINT: adjust action_steps_per_iter to be larger or the pos_threshold to be smaller){bcolors.ENDC}")

                # Grasp / release stages are executed directly with the
                # closed-loop top-down primitive: SubgoalSolver's 6D pose
                # search is unstable for SO-101's 5-DOF arm, and the dense
                # spline path it produces routinely contains waypoints the
                # IK cannot track. The primitive always uses an
                # IK-reachable orientation (verified empirically).
                if self.is_grasp_stage or self.is_release_stage:
                    self.action_queue = []
                    self.first_iter = False
                    self.last_sim_step_counter = self.env.step_counter
                else:
                    next_subgoal = self._get_next_subgoal(from_scratch=self.first_iter)
                    next_path = self._get_next_path(next_subgoal, from_scratch=self.first_iter)
                    self.first_iter = False
                    self.action_queue = next_path.tolist()
                    self.last_sim_step_counter = self.env.step_counter

                # ====================================
                # = execute
                # ====================================
                count = 0
                while len(self.action_queue) > 0 and count < self.config['action_steps_per_iter']:
                    next_action = self.action_queue.pop(0)
                    precise = len(self.action_queue) == 0
                    self.env.execute_action(next_action, precise=precise)
                    count += 1
                if len(self.action_queue) == 0:
                    if self.is_grasp_stage:
                        self._execute_grasp_action()
                    elif self.is_release_stage:
                        self._execute_release_action()
                    # if completed, save video and return
                    if self.stage == self.program_info['num_stages']:
                        self.env.sleep(2.0)
                        save_path = self.env.save_video()
                        print(f"{bcolors.OKGREEN}Video saved to {save_path}\n\n{bcolors.ENDC}")
                        return
                    # progress to next stage
                    self._update_stage(self.stage + 1)

    def _get_next_subgoal(self, from_scratch):
        subgoal_constraints = self.constraint_fns[self.stage]['subgoal']
        path_constraints = self.constraint_fns[self.stage]['path']
        subgoal_pose, debug_dict = self.subgoal_solver.solve(self.curr_ee_pose,
                                                            self.keypoints,
                                                            self.keypoint_movable_mask,
                                                            subgoal_constraints,
                                                            path_constraints,
                                                            self.sdf_voxels,
                                                            self.collision_points,
                                                            self.is_grasp_stage,
                                                            self.curr_joint_pos,
                                                            from_scratch=from_scratch)
        subgoal_pose_homo = T.convert_pose_quat2mat(subgoal_pose)
        # if grasp stage, back up a bit to leave room for grasping
        if self.is_grasp_stage:
            subgoal_pose[:3] += subgoal_pose_homo[:3, :3] @ np.array([-self.config['grasp_depth'] / 2.0, 0, 0])
        debug_dict['stage'] = self.stage
        print_opt_debug_dict(debug_dict)
        if self.visualize:
            self.visualizer.visualize_subgoal(subgoal_pose)
        return subgoal_pose

    def _get_next_path(self, next_subgoal, from_scratch):
        path_constraints = self.constraint_fns[self.stage]['path']
        path, debug_dict = self.path_solver.solve(self.curr_ee_pose,
                                                    next_subgoal,
                                                    self.keypoints,
                                                    self.keypoint_movable_mask,
                                                    path_constraints,
                                                    self.sdf_voxels,
                                                    self.collision_points,
                                                    self.curr_joint_pos,
                                                    from_scratch=from_scratch)
        print_opt_debug_dict(debug_dict)
        processed_path = self._process_path(path)
        if self.visualize:
            self.visualizer.visualize_path(processed_path)
        return processed_path

    def _process_path(self, path):
        # spline interpolate the path from the current ee pose
        full_control_points = np.concatenate([
            self.curr_ee_pose.reshape(1, -1),
            path,
        ], axis=0)
        num_steps = get_linear_interpolation_steps(full_control_points[0], full_control_points[-1],
                                                    self.config['interpolate_pos_step_size'],
                                                    self.config['interpolate_rot_step_size'])
        dense_path = spline_interpolate_poses(full_control_points, num_steps)
        # add gripper action
        ee_action_seq = np.zeros((dense_path.shape[0], 8))
        ee_action_seq[:, :7] = dense_path
        ee_action_seq[:, 7] = self.env.get_gripper_null_action()
        return ee_action_seq

    def _update_stage(self, stage):
        # update stage
        self.stage = stage
        self.is_grasp_stage = self.program_info['grasp_keypoints'][self.stage - 1] != -1
        self.is_release_stage = self.program_info['release_keypoints'][self.stage - 1] != -1
        # can only be grasp stage or release stage or none
        assert self.is_grasp_stage + self.is_release_stage <= 1, "Cannot be both grasp and release stage"
        if self.is_grasp_stage:  # ensure gripper is open for grasping stage
            self.env.open_gripper()
        # clear action queue
        self.action_queue = []
        # update keypoint movable mask
        self._update_keypoint_movable_mask()
        self.first_iter = True

    def _update_keypoint_movable_mask(self):
        for i in range(1, len(self.keypoint_movable_mask)):  # first keypoint is ee so always movable
            keypoint_object = self.env.get_object_by_keypoint(i - 1)
            self.keypoint_movable_mask[i] = self.env.is_grasping(keypoint_object)

    def _execute_grasp_action(self):
        # Closed-loop top-down grasp targeted at the actual trash keypoint.
        # Strategy: drive joint positions directly via IK (bypassing the
        # SubgoalSolver/path_solver/execute_action interpolation chain) so we
        # skip the spline waypoint that the SO-101 5-DOF wrist cannot track.
        # This is the same control surface a real SO-101 uses (Lula IK +
        # JointController), so it is sim2real-friendly.
        grasp_keypoint_idx = self.program_info['grasp_keypoints'][self.stage - 1]
        if grasp_keypoint_idx == -1:
            return
        target_obj = self.env.get_object_by_keypoint(grasp_keypoint_idx)
        target_pos = self.env.get_keypoint_positions()[grasp_keypoint_idx]

        # Top-down orientation reachable by SO-101 over the workspace
        # (verified empirically: EEF +Z -> world -Z is reachable, +X -> -Z is not).
        topdown_rotmat = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
        ])
        topdown_quat = T.mat2quat(topdown_rotmat)

        pregrasp_xyz = target_pos + np.array([0.0, 0.0, 0.07])
        descend_xyz = target_pos + np.array([0.0, 0.0, max(0.005, self.config['grasp_depth'] * 0.5)])
        lift_xyz = target_pos + np.array([0.0, 0.0, 0.06])

        # Phase 1: pre-grasp hover. Drive joints directly via IK at a fixed
        # top-down orientation; no spline interpolation between current and
        # target poses (which is what was killing the wrist before).
        self._drive_to_pose(pregrasp_xyz, topdown_quat, gripper_action=self.env.get_gripper_open_action(), max_iters=80)

        # Phase 2: descend straight down to grasp depth.
        self._drive_to_pose(descend_xyz, topdown_quat, gripper_action=self.env.get_gripper_open_action(), max_iters=40)

        # Phase 3: close gripper, wait for assisted grasp confirmation.
        self.env.close_gripper()
        for _ in range(20):
            if self.env.is_grasping(target_obj):
                break
            self.env._step(self.env._gripper_command_action(self.env.get_gripper_close_action()))

        # Phase 4: lift to clear the table before stage 2 starts.
        self._drive_to_pose(lift_xyz, topdown_quat, gripper_action=self.env.get_gripper_close_action(), max_iters=40)

    def _drive_to_pose(self, target_xyz, target_quat, gripper_action, max_iters=60, pos_tol=0.01, rot_tol_deg=10.0):
        """Drive the SO-101 toward a Cartesian pose via direct IK + JointController.

        Unlike env.execute_action(), this does NOT spline-interpolate between
        the current pose and the target pose; it asks the IK solver for a
        single joint configuration, then steps the simulator with that
        joint target until the EEF reaches it (or max_iters elapses). This
        avoids dense intermediate waypoints that the SO-101 wrist cannot
        reach during large rotation changes.
        """
        env = self.env
        target_quat = np.asarray(target_quat, dtype=float)
        target_xyz = np.asarray(target_xyz, dtype=float)
        target_pose_homo = T.pose2mat([target_xyz, target_quat])
        ik_result = env.ik_solver.solve(
            target_pose_homo,
            position_tolerance=pos_tol,
            orientation_tolerance=np.deg2rad(rot_tol_deg),
            initial_joint_pos=env.get_arm_joint_postions(),
            max_iterations=200,
        )
        from environment import _lula_result_value
        ik_success = bool(_lula_result_value(ik_result, "success"))
        target_joint_pos = np.asarray(_lula_result_value(ik_result, "cspace_position"), dtype=float)
        if not ik_success or not np.all(np.isfinite(target_joint_pos)):
            print(f"{bcolors.WARNING}[main.py] _drive_to_pose: IK could not solve for "
                  f"xyz={target_xyz}, falling back to closest-feasible joints{bcolors.ENDC}")
            if not np.all(np.isfinite(target_joint_pos)):
                return
        action = env._blank_robot_action()
        action[env.arm_action_idx] = target_joint_pos
        action[env.gripper_action_idx] = gripper_action
        for _ in range(max_iters):
            env._step(action=action)
            curr_pos = env.get_ee_pos()
            if np.linalg.norm(curr_pos - target_xyz) < pos_tol * 2:
                break

    def _execute_release_action(self):
        # Closed-loop release: hover above the release target (drop zone) with a
        # top-down EEF, then open. Bypasses SubgoalSolver for the same reason
        # as _execute_grasp_action -- IK reach + sim2real friendliness.
        release_kp_idx = self.program_info['release_keypoints'][self.stage - 1]
        if release_kp_idx == -1:
            self.env.open_gripper()
            return
        # Build the drop position. If the release keypoint is the grasped
        # object itself (typical: trash released "above the bin"), fall back
        # to the bin keypoints (any keypoint that is NOT a grasp_keypoint and
        # is reasonably far from the trash).
        release_pos = self.env.get_keypoint_positions()[release_kp_idx]
        # If the release keypoint is the grasped object, look for a non-grasp
        # keypoint near the bin to use as the drop target.
        grasp_keypoints = self.program_info.get('grasp_keypoints', [])
        if release_kp_idx in grasp_keypoints:
            other_kps = self.env.get_keypoint_positions()
            non_grasp_idxs = [i for i in range(len(other_kps)) if i not in grasp_keypoints]
            if non_grasp_idxs:
                release_pos = np.mean(other_kps[non_grasp_idxs], axis=0)

        topdown_rotmat = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
        ])
        topdown_quat = T.mat2quat(topdown_rotmat)
        # Hover above the drop target (direct IK, no spline interpolation).
        self._drive_to_pose(release_pos + np.array([0.0, 0.0, 0.06]), topdown_quat,
                            gripper_action=self.env.get_gripper_close_action(), max_iters=80)
        # Drop.
        self.env.open_gripper()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default='pen', help='task to perform')
    parser.add_argument('--use_cached_query', action='store_true', help='instead of querying the VLM, use the cached query')
    parser.add_argument('--apply_disturbance', action='store_true', help='apply disturbance to test the robustness')
    parser.add_argument('--visualize', action='store_true', help='visualize each solution before executing (NOTE: this is blocking and needs to press "ESC" to continue)')
    parser.add_argument('--gui', action='store_true', help='show the OmniGibson viewer window instead of running headless')
    args = parser.parse_args()

    if args.apply_disturbance:
        assert args.task == 'pen' and args.use_cached_query, 'disturbance sequence is only defined for cached scenario'

    # ====================================
    # = pen task disturbance sequence
    # ====================================
    def stage1_disturbance_seq(env):
        """
        Move the pen in stage 0 when robot is trying to grasp the pen
        """
        pen = env.og_env.scene.object_registry("name", "pen_1")
        holder = env.og_env.scene.object_registry("name", "pencil_holder_1")
        # disturbance sequence
        pos0, orn0 = pen.get_position_orientation()
        pose0 = np.concatenate([pos0, orn0])
        pos1 = pos0 + np.array([-0.08, 0.0, 0.0])
        orn1 = T.quat_multiply(T.euler2quat(np.array([0, 0, np.pi/4])), orn0)
        pose1 = np.concatenate([pos1, orn1])
        pos2 = pos1 + np.array([0.10, 0.0, 0.0])
        orn2 = T.quat_multiply(T.euler2quat(np.array([0, 0, -np.pi/2])), orn1)
        pose2 = np.concatenate([pos2, orn2])
        control_points = np.array([pose0, pose1, pose2])
        pose_seq = spline_interpolate_poses(control_points, num_steps=25)
        def disturbance(counter):
            if counter < len(pose_seq):
                pose = pose_seq[counter]
                pos, orn = pose[:3], pose[3:]
                pen.set_position_orientation(pos, orn)
                counter += 1
        counter = 0
        while True:
            yield disturbance(counter)
            counter += 1
    
    def stage2_disturbance_seq(env):
        """
        Take the pen out of the gripper in stage 1 when robot is trying to reorient the pen
        """
        apply_disturbance = env.is_grasping()
        pen = env.og_env.scene.object_registry("name", "pen_1")
        holder = env.og_env.scene.object_registry("name", "pencil_holder_1")
        # disturbance sequence
        pos0, orn0 = pen.get_position_orientation()
        pose0 = np.concatenate([pos0, orn0])
        pose1 = np.array([-0.30, -0.15, 0.71, -0.7071068, 0, 0, 0.7071068])
        control_points = np.array([pose0, pose1])
        pose_seq = spline_interpolate_poses(control_points, num_steps=25)
        def disturbance(counter):
            if apply_disturbance:
                if counter < 20:
                    if counter > 15:
                        env.robot.release_grasp_immediately()  # force robot to release the pen
                    else:
                        pass  # do nothing for the other steps
                elif counter < len(pose_seq) + 20:
                    env.robot.release_grasp_immediately()  # force robot to release the pen
                    pose = pose_seq[counter - 20]
                    pos, orn = pose[:3], pose[3:]
                    pen.set_position_orientation(pos, orn)
                    counter += 1
        counter = 0
        while True:
            yield disturbance(counter)
            counter += 1
    
    def stage3_disturbance_seq(env):
        """
        Move the holder in stage 2 when robot is trying to drop the pen into the holder
        """
        pen = env.og_env.scene.object_registry("name", "pen_1")
        holder = env.og_env.scene.object_registry("name", "pencil_holder_1")
        # disturbance sequence
        pos0, orn0 = holder.get_position_orientation()
        pose0 = np.concatenate([pos0, orn0])
        pos1 = pos0 + np.array([-0.02, -0.15, 0.0])
        orn1 = orn0
        pose1 = np.concatenate([pos1, orn1])
        control_points = np.array([pose0, pose1])
        pose_seq = spline_interpolate_poses(control_points, num_steps=5)
        def disturbance(counter):
            if counter < len(pose_seq):
                pose = pose_seq[counter]
                pos, orn = pose[:3], pose[3:]
                holder.set_position_orientation(pos, orn)
                counter += 1
        counter = 0
        while True:
            yield disturbance(counter)
            counter += 1

    task_list = {
        'pen': {
            'scene_file': './configs/og_scene_file_red_pen.json',
            'instruction': 'reorient the red pen and drop it upright into the black pen holder',
            'rekep_program_dir': './vlm_query/pen',
            'disturbance_seq': {1: stage1_disturbance_seq, 2: stage2_disturbance_seq, 3: stage3_disturbance_seq},
            },
        'trash': {
            'scene_file': './configs/og_scene_file_trash.json',
            'instruction': (
                'Use the fixed SO-101 robot mounted on the table to pick up one piece of trash on the tabletop '
                'and place it into the trash bin.'
            ),
            'rekep_program_dir': './vlm_query/trash_cleanup',
            },
    }
    if args.task not in task_list:
        raise ValueError(f"Unknown task {args.task!r}. Available tasks: {sorted(task_list)}")
    task = task_list[args.task]
    scene_file = task['scene_file']
    instruction = task['instruction']
    main = None
    try:
        main = Main(scene_file, visualize=args.visualize)
        main.perform_task(instruction,
                        rekep_program_dir=task['rekep_program_dir'] if args.use_cached_query else None,
                        disturbance_seq=task.get('disturbance_seq', None) if args.apply_disturbance else None)
    except BaseException as exc:
        # Isaac/OG installs a global excepthook that swallows tracebacks once the Kit
        # app starts shutting down. Print to stderr ourselves so the user sees the
        # real failure (e.g. missing OPENAI_API_KEY) instead of a silent Shutting Down.
        import sys, traceback
        sys.stderr.write(f"\n{bcolors.FAIL}[main.py] perform_task failed: "
                         f"{type(exc).__name__}: {exc}{bcolors.ENDC}\n")
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if main is not None:
            import omnigibson as og
            og.shutdown()

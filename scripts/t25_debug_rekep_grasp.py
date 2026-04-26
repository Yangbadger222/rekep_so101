#!/usr/bin/env python3
"""T25 diagnostic: run cached trash stage 1 and print SO101 grasp geometry."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _as_np(value):
    import numpy as np
    import torch

    if torch.is_tensor(value):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def _fmt(value) -> str:
    return "[" + ", ".join(f"{float(v):.5f}" for v in _as_np(value).reshape(-1)) + "]"


def _segment_distance(point, start, end):
    import numpy as np

    point = _as_np(point)
    start = _as_np(start)
    end = _as_np(end)
    ray = end - start
    denom = float(np.dot(ray, ray))
    if denom <= 0:
        return float(np.linalg.norm(point - start)), 0.0, start
    alpha = float(np.clip(np.dot(point - start, ray) / denom, 0.0, 1.0))
    closest = start + alpha * ray
    return float(np.linalg.norm(point - closest)), alpha, closest


def _grasp_rays_world(robot):
    import torch as th
    import omnigibson.utils.transform_utils as OT

    arm = robot.default_arm
    identity = th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32)
    starts = []
    ends = []
    for start_point in robot.assisted_grasp_start_points[arm]:
        pos, quat = robot.links[start_point.link_name].get_position_orientation()
        start_world, _ = OT.pose_transform(pos, quat, start_point.position, identity)
        starts.append(start_world)
    for end_point in robot.assisted_grasp_end_points[arm]:
        pos, quat = robot.links[end_point.link_name].get_position_orientation()
        end_world, _ = OT.pose_transform(pos, quat, end_point.position, identity)
        ends.append(end_world)
    return [(start, end) for start in starts for end in ends]


def _print_grasp_probe(label, env, target_obj):
    import numpy as np
    import torch as th
    import omnigibson.utils.transform_utils as OT

    robot = env.robot
    ee_pose = env.get_ee_pose()
    arm_qpos = env.get_arm_joint_postions()
    obj_pos, obj_quat = target_obj.get_position_orientation()
    obj_pos = _as_np(obj_pos)
    print(f"T25_PROBE {label} arm_qpos={_fmt(arm_qpos)}", flush=True)
    print(f"T25_PROBE {label} ee_pos={_fmt(ee_pose[:3])} ee_quat={_fmt(ee_pose[3:])}", flush=True)
    print(f"T25_PROBE {label} obj={target_obj.name} pos={_fmt(obj_pos)} quat={_fmt(obj_quat)}", flush=True)
    for link_name in ["gripper_frame_link", "gripper_link", "moving_jaw_so101_v1_link"]:
        link_pos, link_quat = robot.links[link_name].get_position_orientation()
        obj_local, _ = OT.relative_pose_transform(
            th.as_tensor(obj_pos, dtype=th.float32),
            th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
            link_pos,
            link_quat,
        )
        print(f"T25_PROBE {label} link={link_name} pos={_fmt(link_pos)} quat={_fmt(link_quat)}", flush=True)
        print(f"T25_PROBE {label} obj_in_{link_name}={_fmt(obj_local)}", flush=True)
    best = (np.inf, -1, 0.0, None)
    for idx, (start, end) in enumerate(_grasp_rays_world(robot)):
        dist, alpha, closest = _segment_distance(obj_pos, start, end)
        if dist < best[0]:
            best = (dist, idx, alpha, closest)
        print(
            f"T25_PROBE {label} ray={idx} start={_fmt(start)} end={_fmt(end)} "
            f"dist_to_obj={dist:.5f} alpha={alpha:.3f} closest={_fmt(closest)}",
            flush=True,
        )
    try:
        contacts, contact_links = robot._find_gripper_contacts(arm=robot.default_arm)
        raycast = robot._find_gripper_raycast_collisions(arm=robot.default_arm)
        contact_links_str = {
            key: sorted(value)
            for key, value in contact_links.items()
        }
        print(
            f"T25_PROBE {label} contacts={sorted(contacts)} "
            f"contact_links={contact_links_str} raycast={sorted(raycast)}",
            flush=True,
        )
    except Exception as exc:
        print(f"T25_PROBE {label} contact_probe_error={type(exc).__name__}: {exc}", flush=True)
    in_hand = robot._ag_obj_in_hand[robot.default_arm]
    print(
        f"T25_PROBE {label} best_ray={best[1]} best_dist={best[0]:.5f} "
        f"best_alpha={best[2]:.3f} is_grasping={env.is_grasping(target_obj)} "
        f"in_hand={getattr(in_hand, 'name', None)}",
        flush=True,
    )


def _search_grasp_qpos(env, target_obj, samples: int, seed: int) -> None:
    import numpy as np
    import omnigibson as og
    import torch as th
    import transform_utils as T

    robot = env.robot
    rng = np.random.default_rng(seed)
    arm = robot.default_arm
    arm_idx = _as_np(robot.arm_control_idx[arm]).astype(int)
    qpos0 = robot.get_joint_positions().detach().clone()
    obj_pos0, obj_quat0 = target_obj.get_position_orientation()
    obj_pos0 = obj_pos0.detach().clone() if hasattr(obj_pos0, "detach") else obj_pos0
    obj_quat0 = obj_quat0.detach().clone() if hasattr(obj_quat0, "detach") else obj_quat0
    lower = _as_np(robot.joint_lower_limits)[arm_idx]
    upper = _as_np(robot.joint_upper_limits)[arm_idx]
    reset_arm = _as_np(robot.reset_joint_pos)[arm_idx]
    obj_pos = _as_np(target_obj.get_position_orientation()[0])

    print(
        f"T25_SEARCH samples={samples} seed={seed} obj={target_obj.name} obj_pos={_fmt(obj_pos)} "
        f"lower={_fmt(lower)} upper={_fmt(upper)} reset={_fmt(reset_arm)}",
        flush=True,
    )
    candidates = []
    for sample_idx in range(samples):
        if sample_idx == 0:
            arm_q = reset_arm
        else:
            arm_q = rng.uniform(lower, upper)
        qpos = qpos0.detach().clone()
        qpos[arm_idx] = th.as_tensor(arm_q, dtype=qpos.dtype)
        robot.set_joint_positions(qpos, drive=False)
        robot.keep_still()
        og.sim.step()
        target_obj.set_position_orientation(position=obj_pos0, orientation=obj_quat0)
        target_obj.keep_still()
        ee_pose = env.get_ee_pose()
        if np.any(ee_pose[:3] < env.bounds_min) or np.any(ee_pose[:3] > env.bounds_max):
            continue
        best_dist = np.inf
        best_alpha = 0.0
        for ray_idx, (start, end) in enumerate(_grasp_rays_world(robot)):
            dist, alpha, _ = _segment_distance(obj_pos, start, end)
            if dist < best_dist:
                best_dist = dist
                best_alpha = alpha
                best_ray = ray_idx
        rotmat = T.quat2mat(ee_pose[3:])
        eef_x = rotmat[:, 0]
        score = best_dist + 0.2 * abs(best_alpha - 0.5) + 0.02 * np.linalg.norm(arm_q - reset_arm)
        candidates.append((score, best_dist, best_alpha, best_ray, arm_q.copy(), ee_pose.copy(), eef_x.copy()))

    robot.set_joint_positions(qpos0, drive=False)
    robot.keep_still()
    og.sim.step()

    if not candidates:
        print("T25_SEARCH no_candidates_in_workspace", flush=True)
        return
    for rank, candidate in enumerate(sorted(candidates, key=lambda x: x[0])[:10]):
        score, best_dist, best_alpha, best_ray, arm_q, ee_pose, eef_x = candidate
        print(
            f"T25_SEARCH rank={rank} score={score:.5f} ray_dist={best_dist:.5f} "
            f"ray_alpha={best_alpha:.3f} ray={best_ray} q={_fmt(arm_q)} "
            f"ee_pos={_fmt(ee_pose[:3])} ee_quat={_fmt(ee_pose[3:])} eef_x={_fmt(eef_x)}",
            flush=True,
        )


def _parse_qpos(value: str):
    try:
        parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    if len(parts) != 5:
        raise argparse.ArgumentTypeError(f"expected 5 comma-separated arm joints, got {len(parts)}")
    return parts


def _try_arm_qpos(env, target_obj, arm_qpos) -> None:
    import omnigibson as og
    import torch as th

    robot = env.robot
    arm = robot.default_arm
    qpos = robot.get_joint_positions().detach().clone()
    obj_pos0, obj_quat0 = target_obj.get_position_orientation()
    obj_pos0 = obj_pos0.detach().clone() if hasattr(obj_pos0, "detach") else obj_pos0
    obj_quat0 = obj_quat0.detach().clone() if hasattr(obj_quat0, "detach") else obj_quat0
    qpos[_as_np(robot.arm_control_idx[arm]).astype(int)] = th.as_tensor(arm_qpos, dtype=qpos.dtype)
    qpos[_as_np(robot.gripper_control_idx[arm]).astype(int)] = th.as_tensor([-0.174533], dtype=qpos.dtype)
    robot.set_joint_positions(qpos, drive=False)
    robot.keep_still()
    for _ in range(10):
        og.sim.step()
    target_obj.set_position_orientation(position=obj_pos0, orientation=obj_quat0)
    target_obj.keep_still()
    _print_grasp_probe("try_qpos_open", env, target_obj)

    action = th.zeros(robot.action_dim, dtype=th.float32)
    action[robot.arm_action_idx[arm]] = qpos[_as_np(robot.arm_control_idx[arm]).astype(int)]
    action[robot.gripper_action_idx[arm]] = th.as_tensor([1.0], dtype=th.float32)
    for step in range(80):
        robot.apply_action(action)
        og.sim.step()
        if step in {0, 1, 2, 5, 10, 20, 40, 79} or env.is_grasping(target_obj):
            _print_grasp_probe(f"try_qpos_close_step_{step}", env, target_obj)
        if env.is_grasping(target_obj):
            break


def _try_arm_qpos_no_settle(env, target_obj, arm_qpos) -> None:
    import omnigibson as og
    import torch as th

    robot = env.robot
    arm = robot.default_arm
    qpos = robot.get_joint_positions().detach().clone()
    qpos[_as_np(robot.arm_control_idx[arm]).astype(int)] = th.as_tensor(arm_qpos, dtype=qpos.dtype)
    qpos[_as_np(robot.gripper_control_idx[arm]).astype(int)] = th.as_tensor([-0.174533], dtype=qpos.dtype)
    robot.set_joint_positions(qpos, drive=False)
    robot.keep_still()
    _print_grasp_probe("try_qpos_no_settle_open", env, target_obj)

    action = th.zeros(robot.action_dim, dtype=th.float32)
    action[robot.arm_action_idx[arm]] = th.as_tensor(arm_qpos, dtype=th.float32)
    action[robot.gripper_action_idx[arm]] = th.as_tensor([1.0], dtype=th.float32)
    for step in range(20):
        robot.apply_action(action)
        og.sim.step()
        if step in {0, 1, 2, 5, 10, 19} or env.is_grasping(target_obj):
            _print_grasp_probe(f"try_qpos_no_settle_close_step_{step}", env, target_obj)
        if env.is_grasping(target_obj):
            break


def _search_drop_qpos(env, target_obj, samples: int, seed: int) -> None:
    import numpy as np
    import omnigibson as og
    import torch as th
    import omnigibson.utils.transform_utils as OT

    robot = env.robot
    arm = robot.default_arm
    arm_idx = _as_np(robot.arm_control_idx[arm]).astype(int)
    qpos0 = robot.get_joint_positions().detach().clone()
    lower = _as_np(robot.joint_lower_limits)[arm_idx]
    upper = _as_np(robot.joint_upper_limits)[arm_idx]
    rng = np.random.default_rng(seed)

    obj_pos, obj_quat = target_obj.get_position_orientation()
    eef_pos, eef_quat = robot.eef_links[arm].get_position_orientation()
    obj_rel_pos, obj_rel_quat = OT.relative_pose_transform(obj_pos, obj_quat, eef_pos, eef_quat)
    target = np.array([-0.15, 0.15, 0.91], dtype=float)
    reset_arm = _as_np(robot.reset_joint_pos)[arm_idx]
    candidates = []
    print(
        f"T25_DROP_SEARCH samples={samples} seed={seed} obj_rel_pos={_fmt(obj_rel_pos)} "
        f"target={_fmt(target)} lower={_fmt(lower)} upper={_fmt(upper)}",
        flush=True,
    )
    seeds = [
        reset_arm,
        np.array([-1.2, 0.4, 0.2, 1.2, -0.3]),
        np.array([-1.4, 0.2, 0.5, 1.0, -0.4]),
        np.array([-1.0, 0.7, -0.2, 1.4, -0.3]),
        np.array([-1.5, 0.8, -0.4, 1.7, -0.3]),
    ]
    for sample_idx in range(samples + len(seeds)):
        if sample_idx < len(seeds):
            arm_q = np.clip(seeds[sample_idx], lower, upper)
        else:
            arm_q = rng.uniform(lower, upper)
        qpos = qpos0.detach().clone()
        qpos[arm_idx] = th.as_tensor(arm_q, dtype=qpos.dtype)
        robot.set_joint_positions(qpos, drive=False)
        robot.keep_still()
        ee_pose = env.get_ee_pose()
        if np.any(ee_pose[:3] < env.bounds_min - 0.05) or np.any(ee_pose[:3] > env.bounds_max + 0.10):
            continue
        pred_obj_pos, _ = OT.pose_transform(
            th.as_tensor(ee_pose[:3], dtype=th.float32),
            th.as_tensor(ee_pose[3:], dtype=th.float32),
            obj_rel_pos,
            obj_rel_quat,
        )
        pred_obj_pos = _as_np(pred_obj_pos)
        score = float(np.linalg.norm(pred_obj_pos - target) + 0.03 * np.linalg.norm(arm_q - reset_arm))
        candidates.append((score, float(np.linalg.norm(pred_obj_pos - target)), arm_q.copy(), ee_pose.copy(), pred_obj_pos))

    robot.set_joint_positions(qpos0, drive=False)
    robot.keep_still()
    og.sim.step()

    if not candidates:
        print("T25_DROP_SEARCH no_candidates", flush=True)
        return
    for rank, (score, dist, arm_q, ee_pose, pred_obj_pos) in enumerate(sorted(candidates, key=lambda x: x[0])[:12]):
        print(
            f"T25_DROP_SEARCH rank={rank} score={score:.5f} obj_dist={dist:.5f} "
            f"q={_fmt(arm_q)} ee_pos={_fmt(ee_pose[:3])} ee_quat={_fmt(ee_pose[3:])} "
            f"pred_obj={_fmt(pred_obj_pos)}",
            flush=True,
        )


def _try_drop_qpos(env, target_obj, arm_qpos) -> None:
    import omnigibson as og
    import torch as th

    robot = env.robot
    arm = robot.default_arm
    qpos = robot.get_joint_positions().detach().clone()
    qpos[_as_np(robot.arm_control_idx[arm]).astype(int)] = th.as_tensor(arm_qpos, dtype=qpos.dtype)
    qpos[_as_np(robot.gripper_control_idx[arm]).astype(int)] = th.as_tensor([1.74533], dtype=qpos.dtype)
    robot.set_joint_positions(qpos, drive=False)
    robot.keep_still()
    action = th.zeros(robot.action_dim, dtype=th.float32)
    action[robot.arm_action_idx[arm]] = th.as_tensor(arm_qpos, dtype=th.float32)
    action[robot.gripper_action_idx[arm]] = th.as_tensor([1.0], dtype=th.float32)
    for step in range(40):
        robot.apply_action(action)
        og.sim.step()
        if step in {0, 1, 5, 10, 20, 39}:
            _print_grasp_probe(f"try_drop_qpos_hold_step_{step}", env, target_obj)
    env.open_gripper()
    for step in range(60):
        og.sim.step()
        if step in {0, 1, 10, 30, 59}:
            _print_grasp_probe(f"try_drop_qpos_release_step_{step}", env, target_obj)


def _drive_arm_qpos(env, arm_qpos, steps: int = 180) -> None:
    import numpy as np

    action = env._blank_robot_action()
    action[env.arm_action_idx] = np.asarray(arm_qpos, dtype=float)
    action[env.gripper_action_idx] = env.get_gripper_open_action()
    for _ in range(steps):
        env._step(action)


def _try_motion_suite(env, target_obj) -> None:
    import numpy as np
    import omnigibson as og
    import torch as th

    robot = env.robot
    arm_idx = _as_np(robot.arm_control_idx[robot.default_arm]).astype(int)
    qpos0 = robot.get_joint_positions().detach().clone()
    obj_pos0, obj_quat0 = target_obj.get_position_orientation()
    obj_pos0 = obj_pos0.detach().clone() if hasattr(obj_pos0, "detach") else obj_pos0
    obj_quat0 = obj_quat0.detach().clone() if hasattr(obj_quat0, "detach") else obj_quat0
    final_q = np.array([-0.86698, 0.89339, -0.21821, 1.60090, -0.34938])
    suites = [
        ("direct", [final_q]),
        ("yaw_then_final", [
            np.array([-0.86698, -0.50000, 1.00000, 0.00000, -0.34938]),
            final_q,
        ]),
        ("folded_high_then_final", [
            np.array([-0.86698, -0.50000, 1.00000, 0.00000, -0.34938]),
            np.array([-0.86698, 0.05000, 0.65000, 0.65000, -0.34938]),
            final_q,
        ]),
        ("elbow_high_then_final", [
            np.array([-0.86698, -0.50000, 1.00000, 0.00000, -0.34938]),
            np.array([-0.86698, 0.30000, 0.30000, 1.05000, -0.34938]),
            final_q,
        ]),
        ("late_pan", [
            np.array([0.00000, 0.30000, 0.30000, 1.05000, -0.34938]),
            np.array([-0.86698, 0.30000, 0.30000, 1.05000, -0.34938]),
            final_q,
        ]),
    ]

    for name, waypoints in suites:
        qpos = qpos0.detach().clone()
        qpos[arm_idx] = qpos0[arm_idx]
        robot.set_joint_positions(qpos, drive=False)
        robot.keep_still()
        target_obj.set_position_orientation(position=obj_pos0, orientation=obj_quat0)
        target_obj.keep_still()
        for _ in range(10):
            og.sim.step()
        env.open_gripper()

        print(f"T25_MOTION name={name} start_obj={_fmt(obj_pos0)}", flush=True)
        for idx, waypoint in enumerate(waypoints):
            _drive_arm_qpos(env, waypoint)
            obj_pos = _as_np(target_obj.get_position_orientation()[0])
            drift = float(np.linalg.norm(obj_pos - _as_np(obj_pos0)))
            print(
                f"T25_MOTION name={name} waypoint={idx} q={_fmt(env.get_arm_joint_postions())} "
                f"ee={_fmt(env.get_ee_pose()[:3])} obj={_fmt(obj_pos)} drift={drift:.5f}",
                flush=True,
            )
            if drift > 0.02:
                break
        env.close_gripper()
        obj_pos = _as_np(target_obj.get_position_orientation()[0])
        drift = float(np.linalg.norm(obj_pos - _as_np(obj_pos0)))
        in_hand = robot._ag_obj_in_hand[robot.default_arm]
        print(
            f"T25_MOTION_RESULT name={name} drift={drift:.5f} "
            f"is_grasping={env.is_grasping(target_obj)} in_hand={getattr(in_hand, 'name', None)}",
            flush=True,
        )


def run(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    import json
    import numpy as np
    import omnigibson as og

    import transform_utils as T
    from main import Main
    from utils import get_callable_grasping_cost_fn, load_functions_from_txt

    main = None
    try:
        main = Main("./configs/og_scene_file_trash.json", visualize=False)
        main.env.reset()
        program_dir = Path("./vlm_query/trash_cleanup")
        with open(program_dir / "metadata.json", "r") as f:
            main.program_info = json.load(f)
        main.env.register_keypoints(main.program_info["init_keypoint_positions"])
        target_obj = main.env.get_object_by_keypoint(0)
        if args.try_qpos is not None:
            _try_arm_qpos(main.env, target_obj, args.try_qpos)
            return
        if args.try_qpos_no_settle is not None:
            _try_arm_qpos_no_settle(main.env, target_obj, args.try_qpos_no_settle)
            return
        if args.try_motion_suite:
            _try_motion_suite(main.env, target_obj)
            return
        if args.search_only:
            _search_grasp_qpos(main.env, target_obj, args.samples, args.seed)
            return
        main.constraint_fns = {}
        for stage in range(1, main.program_info["num_stages"] + 1):
            stage_dict = {}
            for constraint_type in ["subgoal", "path"]:
                load_path = program_dir / f"stage{stage}_{constraint_type}_constraints.txt"
                stage_dict[constraint_type] = load_functions_from_txt(
                    str(load_path), get_callable_grasping_cost_fn(main.env)
                )
            main.constraint_fns[stage] = stage_dict
        main.keypoint_movable_mask = np.zeros(main.program_info["num_keypoints"] + 1, dtype=bool)
        main.keypoint_movable_mask[0] = True
        main._update_stage(1)

        print(f"T25_TARGET keypoint0_obj={target_obj.name}", flush=True)
        _print_grasp_probe("after_reset", main.env, target_obj)

        scene_keypoints = main.env.get_keypoint_positions()
        main.keypoints = np.concatenate([[main.env.get_ee_pos()], scene_keypoints], axis=0)
        main.curr_ee_pose = main.env.get_ee_pose()
        main.curr_joint_pos = main.env.get_arm_joint_postions()
        main.sdf_voxels = main.env.get_sdf_voxels(main.config["sdf_voxel_size"])
        main.collision_points = main.env.get_collision_points()
        subgoal = main._get_next_subgoal(from_scratch=True)
        path = main._get_next_path(subgoal, from_scratch=True)
        print(f"T25_PLAN subgoal_pos={_fmt(subgoal[:3])} subgoal_quat={_fmt(subgoal[3:])}", flush=True)
        print(f"T25_PLAN path_len={len(path)} first={_fmt(path[0, :7])} last={_fmt(path[-1, :7])}", flush=True)

        if main.env.robot.__class__.__name__ == "SO101":
            print("T25_PLAN skip_stage1_task_space_path_for_so101=True", flush=True)
        else:
            for idx, action in enumerate(path.tolist()):
                main.env.execute_action(action, precise=(idx == len(path) - 1))
        _print_grasp_probe("before_grasp_push", main.env, target_obj)
        pre_pose = main.env.get_ee_pose()
        grasp_pose = pre_pose.copy()
        grasp_pose[:3] += T.quat2mat(pre_pose[3:]) @ np.array([main.config["grasp_depth"], 0.0, 0.0])
        print(f"T25_GRASP_PUSH depth={main.config['grasp_depth']:.5f} target={_fmt(grasp_pose)}", flush=True)
        main._execute_grasp_action()
        _print_grasp_probe("after_grasp_push", main.env, target_obj)
        if args.search_drop_qpos:
            _search_drop_qpos(main.env, target_obj, args.samples, args.seed)
            return
        if args.try_drop_qpos is not None:
            _try_drop_qpos(main.env, target_obj, args.try_drop_qpos)
            return
    finally:
        if main is not None:
            og.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run OmniGibson headlessly")
    parser.add_argument("--search-only", action="store_true", help="sample joint space and print reachable grasp-ray candidates")
    parser.add_argument("--samples", type=int, default=1000, help="number of joint samples for --search-only")
    parser.add_argument("--seed", type=int, default=0, help="random seed for --search-only")
    parser.add_argument("--try-qpos", type=_parse_qpos, help="set 5 arm joints, close gripper, and print grasp probes")
    parser.add_argument("--try-qpos-no-settle", type=_parse_qpos, help="set 5 arm joints and close immediately")
    parser.add_argument("--try-motion-suite", action="store_true", help="physically drive several scripted SO101 grasp approach waypoint suites")
    parser.add_argument("--search-drop-qpos", action="store_true", help="after grasp, sample qpos candidates that place the held object over the bin")
    parser.add_argument("--try-drop-qpos", type=_parse_qpos, help="after grasp, move to a candidate drop qpos and release")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()

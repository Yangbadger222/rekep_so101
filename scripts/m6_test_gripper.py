#!/usr/bin/env python3
"""M6 smoke test: verify SO101 gripper open/close and assisted grasp."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _as_list(value) -> list[float]:
    return [float(v) for v in value]


def _assert_finite(label: str, tensor) -> None:
    import torch as th

    if th.any(th.isnan(tensor)) or th.any(th.isinf(tensor)):
        raise RuntimeError(f"{label} contains NaN or inf: {_as_list(tensor)}")


def _gripper_qpos(robot):
    if os.environ.get("REKEP_DEBUG_STEPS") == "1":
        print("debug_gripper_qpos before_get_joint_positions", flush=True)
    joint_positions = robot.get_joint_positions()
    if os.environ.get("REKEP_DEBUG_STEPS") == "1":
        print("debug_gripper_qpos after_get_joint_positions", flush=True)
    return joint_positions[robot.gripper_control_idx[robot.default_arm]][0]


def _make_action(robot, gripper_command: float, arm_command: dict[int, float] | None = None):
    import torch as th

    action = th.zeros(robot.action_dim, dtype=th.float32)
    if arm_command:
        for idx, value in arm_command.items():
            action[idx] = value
    action[robot.gripper_action_idx[robot.default_arm]] = gripper_command
    return action


def _step_action(env, robot, action, steps: int) -> None:
    import omnigibson as og

    debug_steps = os.environ.get("REKEP_DEBUG_STEPS") == "1"
    for step in range(steps):
        try:
            if debug_steps:
                print(f"debug_step={step} before_apply", flush=True)
            robot.apply_action(action)
            if debug_steps:
                print(f"debug_step={step} after_apply before_sim_step", flush=True)
            og.sim.step()
            if debug_steps:
                print(f"debug_step={step} after_sim_step", flush=True)
        except SystemExit as exc:
            raise RuntimeError(f"sim step raised SystemExit({exc.code}) at local step {step}") from exc
        if debug_steps:
            print(f"debug_step={step} before_qpos_check", flush=True)
        qpos = robot.get_joint_positions()
        _assert_finite(f"joint positions after step {step}", qpos)
        if debug_steps:
            print(f"debug_step={step} after_qpos_check", flush=True)


def _grasp_ray_world(robot):
    return _grasp_rays_world(robot)[0]


def _grasp_rays_world(robot):
    import torch as th
    import omnigibson.utils.transform_utils as T

    arm = robot.default_arm
    identity = th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32)

    starts = []
    ends = []
    for start_point in robot.assisted_grasp_start_points[arm]:
        start_link_pos, start_link_quat = robot.links[start_point.link_name].get_position_orientation()
        start_world, _ = T.pose_transform(start_link_pos, start_link_quat, start_point.position, identity)
        starts.append(start_world)
    for end_point in robot.assisted_grasp_end_points[arm]:
        end_link_pos, end_link_quat = robot.links[end_point.link_name].get_position_orientation()
        end_world, _ = T.pose_transform(end_link_pos, end_link_quat, end_point.position, identity)
        ends.append(end_world)
    return [(start_world, end_world) for start_world in starts for end_world in ends]


def _debug_grasp_geometry(label: str, robot, cube=None) -> None:
    try:
        root_pos, root_quat = robot.get_position_orientation()
        eef_pos = robot.get_eef_position()
        eef_quat = robot.get_eef_orientation()
        gripper_pos, gripper_quat = robot.links["gripper_link"].get_position_orientation()
        jaw_pos, jaw_quat = robot.links["moving_jaw_so101_v1_link"].get_position_orientation()
        rays = _grasp_rays_world(robot)
        start_world, end_world = rays[0]
        ray_len = float((end_world - start_world).norm())
        qpos = robot.get_joint_positions()
        print(
            f"debug_geometry label={label} "
            f"root_pos={_as_list(root_pos)} root_quat={_as_list(root_quat)} "
            f"eef_pos={_as_list(eef_pos)} eef_quat={_as_list(eef_quat)} "
            f"gripper_pos={_as_list(gripper_pos)} gripper_quat={_as_list(gripper_quat)} "
            f"jaw_pos={_as_list(jaw_pos)} jaw_quat={_as_list(jaw_quat)} "
            f"ray_count={len(rays)} ray_start={_as_list(start_world)} ray_end={_as_list(end_world)} "
            f"ray_len={ray_len:.5f} "
            f"qpos={_as_list(qpos)}",
            flush=True,
        )
        if cube is not None:
            cube_pos, cube_quat = cube.get_position_orientation()
            print(
                f"debug_geometry label={label} cube_pos={_as_list(cube_pos)} cube_quat={_as_list(cube_quat)}",
                flush=True,
            )
        try:
            contacts, contact_links = robot._find_gripper_contacts(arm=robot.default_arm)
            raycast = robot._find_gripper_raycast_collisions(arm=robot.default_arm)
            print(
                f"debug_geometry label={label} contacts={sorted(contacts)} "
                f"contact_links={{{', '.join(f'{key}: {sorted(value)}' for key, value in contact_links.items())}}} "
                f"raycast={sorted(raycast)}",
                flush=True,
            )
        except Exception as exc:
            print(f"debug_geometry label={label} contact_probe_error={type(exc).__name__}: {exc}", flush=True)
    except Exception as exc:
        print(f"debug_geometry label={label} error={type(exc).__name__}: {exc}", flush=True)


def _debug_contact_step(label: str, robot, cube, state) -> None:
    try:
        from omnigibson.utils.usd_utils import RigidContactAPI

        cube_path = cube.root_link.prim_path
        finger_contacts, finger_contact_links = robot._find_gripper_contacts(arm=robot.default_arm)
        raycast = robot._find_gripper_raycast_collisions(arm=robot.default_arm)
        try:
            row_contacts = RigidContactAPI.get_contact_pairs(robot.scene.idx, row_prim_paths={cube_path})
        except ValueError:
            row_contacts = set()
        try:
            column_contacts = RigidContactAPI.get_contact_pairs(robot.scene.idx, column_prim_paths={cube_path})
        except ValueError:
            column_contacts = set()
        cube_pos = cube.get_position_orientation()[0]
        print(
            f"debug_contact label={label} state={state.name} cube_pos={_as_list(cube_pos)} "
            f"finger_contacts={sorted(finger_contacts)} "
            f"finger_links={{{', '.join(f'{key}: {sorted(value)}' for key, value in finger_contact_links.items())}}} "
            f"raycast={sorted(raycast)} row_contacts={sorted(row_contacts)} "
            f"column_contacts={sorted(column_contacts)}",
            flush=True,
        )
    except Exception as exc:
        print(f"debug_contact label={label} error={type(exc).__name__}: {exc}", flush=True)


def _place_cube(cube, position) -> None:
    import torch as th

    cube.disable_gravity()
    cube.set_position_orientation(
        position=position,
        orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
    )
    cube.keep_still()


def _build_env(args: argparse.Namespace):
    import omnigibson as og
    from omnigibson.macros import gm
    from omnigibson.robots import REGISTERED_ROBOTS

    from so101_robot import SO101

    gm.USE_GPU_DYNAMICS = args.use_gpu_dynamics
    gm.ENABLE_FLATCACHE = False

    if "SO101" not in REGISTERED_ROBOTS:
        raise RuntimeError("SO101 was not registered in OmniGibson's robot registry")

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "type": "SO101",
                "name": "so101",
                "position": [0.0, -0.15, 0.75],
                "obs_modalities": [],
                "grasping_mode": "assisted",
                "action_type": "continuous",
                "action_normalize": True,
            }
        ],
        "objects": [
            {
                "type": "PrimitiveObject",
                "name": "grasp_cube",
                "primitive_type": "Cube",
                "category": "trash",
                "size": args.object_size,
                "fixed_base": False,
                "visual_only": False,
                "load_config": {"mass": args.object_mass},
                "rgba": [1.0, 0.1, 0.1, 1.0],
                "position": [0.0, 0.0, 1.2],
            }
        ],
        "task": {"type": "DummyTask"},
        "env": {
            "action_frequency": 30,
            "physics_frequency": 120,
            "rendering_frequency": 30,
        },
    }

    env = og.Environment(configs=cfg)
    robot = env.robots[0]
    if not isinstance(robot, SO101):
        raise RuntimeError(f"expected SO101, got {type(robot).__name__}")
    cube = env.scene.object_registry("name", "grasp_cube")
    if cube is None:
        raise RuntimeError("failed to find grasp_cube in scene registry")
    return og, env, robot, cube


def _test_open_close(args: argparse.Namespace, env, robot) -> None:
    open_action = _make_action(robot, args.open_command)
    close_action = _make_action(robot, args.close_command)
    controller = robot.controllers[f"gripper_{robot.default_arm}"]
    open_qpos = float(controller._open_qpos[0])
    closed_qpos = float(controller._closed_qpos[0])
    max_open_err = 0.0
    max_close_err = 0.0

    print(
        f"phase=open_close cycles={args.cycles} open_qpos={open_qpos:.5f} closed_qpos={closed_qpos:.5f}",
        flush=True,
    )
    for cycle in range(args.cycles):
        _step_action(env, robot, open_action, args.steps_per_command)
        if os.environ.get("REKEP_DEBUG_STEPS") == "1":
            print(f"debug_cycle={cycle} after_open_step", flush=True)
        open_err = abs(float(_gripper_qpos(robot)) - open_qpos)
        if os.environ.get("REKEP_DEBUG_STEPS") == "1":
            print(f"debug_cycle={cycle} after_open_err", flush=True)
        max_open_err = max(max_open_err, open_err)
        if open_err > args.gripper_tolerance:
            raise RuntimeError(f"open cycle {cycle} err={open_err:.5f} > {args.gripper_tolerance:.5f}")

        _step_action(env, robot, close_action, args.steps_per_command)
        close_err = abs(float(_gripper_qpos(robot)) - closed_qpos)
        max_close_err = max(max_close_err, close_err)
        if close_err > args.gripper_tolerance:
            raise RuntimeError(f"close cycle {cycle} err={close_err:.5f} > {args.gripper_tolerance:.5f}")

        if cycle % args.log_every_cycles == 0 or cycle == args.cycles - 1:
            print(
                f"cycle={cycle:03d} open_err={open_err:.5f} close_err={close_err:.5f} "
                f"qpos={float(_gripper_qpos(robot)):.5f}",
                flush=True,
            )

    print(
        f"gripper_open_close: cycles={args.cycles} max_open_err={max_open_err:.5f} "
        f"max_close_err={max_close_err:.5f}",
        flush=True,
    )


def _try_assisted_grasp(args: argparse.Namespace, env, robot, cube):
    from omnigibson.controllers import IsGraspingState

    open_action = _make_action(robot, args.open_command)
    close_action = _make_action(robot, args.close_command)

    print(
        f"phase=assisted_grasp local_placements={args.grasp_local_placements} alphas={args.ray_alphas}",
        flush=True,
    )
    if robot.is_grasping(candidate_obj=cube) != IsGraspingState.FALSE:
        raise RuntimeError("expected is_grasping(False) before grasp attempt")

    if args.debug_geometry:
        _debug_grasp_geometry("before_attempts", robot, cube)

    for local_idx, local_xyz in enumerate(args.grasp_local_placements):
        _step_action(env, robot, open_action, args.pregrasp_open_steps)
        target_pos = _local_position_to_world(robot, "gripper_link", local_xyz)
        label = (
            f"source=local index={local_idx} "
            f"local=({local_xyz[0]:.3f},{local_xyz[1]:.3f},{local_xyz[2]:.3f})"
        )
        if _attempt_grasp_at_position(args, robot, cube, close_action, target_pos, label):
            return target_pos
        _step_action(env, robot, open_action, args.pregrasp_open_steps)

    for ray_idx in range(len(robot.assisted_grasp_start_points[robot.default_arm]) * len(robot.assisted_grasp_end_points[robot.default_arm])):
        _step_action(env, robot, open_action, args.pregrasp_open_steps)
        start_world, end_world = _grasp_rays_world(robot)[ray_idx]
        ray = end_world - start_world
        for alpha in args.ray_alphas:
            target_pos = start_world + alpha * ray
            label = f"source=ray ray={ray_idx} alpha={alpha:.2f}"
            if _attempt_grasp_at_position(args, robot, cube, close_action, target_pos, label):
                return target_pos
            _step_action(env, robot, open_action, args.pregrasp_open_steps)
    raise RuntimeError("assisted grasp did not trigger for any tested ray placement")


def _test_follow_and_release(args: argparse.Namespace, env, robot, cube) -> None:
    from omnigibson.controllers import IsGraspingState

    close_action = _make_action(robot, args.close_command)
    open_action = _make_action(robot, args.open_command)
    move_action = _make_action(robot, args.close_command, arm_command={0: args.arm_move_command})

    print("phase=follow_and_release", flush=True)
    if robot.is_grasping(candidate_obj=cube) != IsGraspingState.TRUE:
        raise RuntimeError("expected cube to be grasped before follow test")

    cube_start = cube.get_position_orientation()[0].clone()
    eef_start = robot.get_eef_position().clone()
    _step_action(env, robot, move_action, args.move_steps)
    _step_action(env, robot, close_action, args.steps_per_command)
    cube_after = cube.get_position_orientation()[0].clone()
    eef_after = robot.get_eef_position().clone()
    cube_motion = (cube_after - cube_start).norm().item()
    eef_motion = (eef_after - eef_start).norm().item()
    print(f"follow: eef_motion_m={eef_motion:.5f} cube_motion_m={cube_motion:.5f}", flush=True)
    if eef_motion < args.min_follow_motion:
        raise RuntimeError(f"EEF did not move enough for follow test: {eef_motion:.5f} m")
    if cube_motion < args.min_follow_motion:
        raise RuntimeError(f"cube did not follow grasp: {cube_motion:.5f} m")

    release_start_z = float(cube_after[2])
    cube.enable_gravity()
    _step_action(env, robot, open_action, args.release_steps)
    release_state = robot.is_grasping(candidate_obj=cube)
    if release_state != IsGraspingState.FALSE:
        raise RuntimeError(f"expected released grasp state FALSE, got {release_state.name}")
    release_end_z = float(cube.get_position_orientation()[0][2])
    drop = release_start_z - release_end_z
    print(f"release: start_z={release_start_z:.5f} end_z={release_end_z:.5f} drop_m={drop:.5f}", flush=True)
    if drop < args.min_release_drop:
        raise RuntimeError(f"cube did not drop enough after release: {drop:.5f} m")


def run(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    og = None
    env = None
    error = None
    try:
        og, env, robot, cube = _build_env(args)
        print(
            "SO101 gripper smoke: "
            f"action_dim={robot.action_dim} gripper_idx={_as_list(robot.gripper_action_idx[robot.default_arm])} "
            f"finger_links={robot.finger_link_names[robot.default_arm]}",
            flush=True,
        )
        og.sim.play()
        print("phase=sim_playing", flush=True)
        _test_open_close(args, env, robot)
        print("phase=open_close_done", flush=True)
        if args.debug_geometry:
            _debug_grasp_geometry("after_open_close", robot, cube)
        _try_assisted_grasp(args, env, robot, cube)
        _test_follow_and_release(args, env, robot, cube)
        print("PASS: SO101 gripper open/close and assisted grasp verified", flush=True)
    except BaseException as exc:
        error = exc
        raise
    finally:
        print("phase=cleanup", flush=True)
        if env is not None and og is not None and og.app is not None:
            try:
                og.shutdown()
            except BaseException as shutdown_error:
                if error is None:
                    raise
                print(f"phase=cleanup ignored_shutdown_error={type(shutdown_error).__name__}: {shutdown_error}", flush=True)


def _parse_alphas(value: str) -> list[float]:
    try:
        return [float(part.strip()) for part in value.split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _parse_local_placements(value: str) -> list[tuple[float, float, float]]:
    placements = []
    try:
        for raw_triplet in value.split(";"):
            if not raw_triplet.strip():
                continue
            triplet = tuple(float(part.strip()) for part in raw_triplet.split(","))
            if len(triplet) != 3:
                raise ValueError(f"expected 3 comma-separated values, got {raw_triplet!r}")
            placements.append(triplet)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return placements


def _local_position_to_world(robot, link_name: str, local_xyz: tuple[float, float, float]):
    import torch as th
    import omnigibson.utils.transform_utils as T

    identity = th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32)
    link_pos, link_quat = robot.links[link_name].get_position_orientation()
    local_pos = th.tensor(local_xyz, dtype=th.float32)
    world_pos, _ = T.pose_transform(link_pos, link_quat, local_pos, identity)
    return world_pos


def _attempt_grasp_at_position(
    args: argparse.Namespace,
    robot,
    cube,
    close_action,
    target_pos,
    label: str,
):
    import omnigibson as og
    from omnigibson.controllers import IsGraspingState

    _place_cube(cube, target_pos)
    if args.debug_geometry:
        _debug_grasp_geometry(f"after_place_{label}", robot, cube)

    state = IsGraspingState.FALSE
    for step in range(args.grasp_steps):
        try:
            robot.apply_action(close_action)
            og.sim.step()
        except SystemExit as exc:
            raise RuntimeError(f"sim step raised SystemExit({exc.code}) during grasp step {step}") from exc
        _assert_finite(f"joint positions after grasp step {step}", robot.get_joint_positions())
        state = robot.is_grasping(candidate_obj=cube)
        if args.debug_geometry and (
            step in {0, 1, 2, 5, 10, 20, args.grasp_steps - 1} or state == IsGraspingState.TRUE
        ):
            _debug_contact_step(f"{label}_step_{step}", robot, cube, state)
        if state == IsGraspingState.TRUE:
            print(f"grasp_triggered {label} step={step}", flush=True)
            break

    if args.debug_geometry:
        _debug_grasp_geometry(f"after_close_{label}", robot, cube)
    print(
        f"grasp_attempt {label} "
        f"cube_pos={_as_list(cube.get_position_orientation()[0])} is_grasping={state.name}",
        flush=True,
    )
    return state == IsGraspingState.TRUE


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run OmniGibson headlessly")
    parser.add_argument("--cycles", type=int, default=100, help="open/close cycles")
    parser.add_argument("--steps-per-command", type=int, default=50, help="steps for basic gripper commands")
    parser.add_argument("--pregrasp-open-steps", type=int, default=4, help="opening steps before each grasp placement")
    parser.add_argument("--grasp-steps", type=int, default=100, help="steps for assisted grasp closing")
    parser.add_argument("--move-steps", type=int, default=20, help="steps for moving while grasped")
    parser.add_argument("--release-steps", type=int, default=100, help="steps after opening to release")
    parser.add_argument("--log-every-cycles", type=int, default=20, help="open/close cycle logging interval")
    parser.add_argument("--open-command", type=float, default=-1.0, help="normalized gripper open command")
    parser.add_argument("--close-command", type=float, default=1.0, help="normalized gripper close command")
    parser.add_argument("--arm-move-command", type=float, default=1.0, help="shoulder_pan delta command while grasped")
    parser.add_argument("--object-size", type=float, default=0.02, help="test cube side length in meters")
    parser.add_argument("--object-mass", type=float, default=0.01, help="test cube mass in kg")
    parser.add_argument("--ray-alphas", type=_parse_alphas, default=[0.35, 0.5, 0.65], help="comma-separated grasp ray placements")
    parser.add_argument(
        "--grasp-local-placements",
        type=_parse_local_placements,
        default=[(0.010, 0.000, -0.030)],
        help="semicolon-separated gripper_link-local xyz placements before ray alpha fallback",
    )
    parser.add_argument("--gripper-tolerance", type=float, default=0.02, help="max gripper qpos error")
    parser.add_argument("--min-follow-motion", type=float, default=0.005, help="minimum object motion while grasped")
    parser.add_argument("--min-release-drop", type=float, default=0.01, help="minimum z drop after release")
    parser.add_argument("--use-gpu-dynamics", action="store_true", help="enable GPU dynamics")
    parser.add_argument("--debug-geometry", action="store_true", help="print gripper link, raycast, and contact diagnostics")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()

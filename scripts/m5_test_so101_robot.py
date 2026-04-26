#!/usr/bin/env python3
"""M5 smoke test: verify SO101 arm joint and EEF pose behavior."""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
EEF_CHAIN_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper_frame_joint",
]
URDF_PATH = REPO_ROOT / "assert" / "SO101" / "so101_new_calib.urdf"


def _as_list(value) -> list[float]:
    return [float(v) for v in value]


def _assert_finite(label: str, tensor) -> None:
    import torch as th

    if th.any(th.isnan(tensor)) or th.any(th.isinf(tensor)):
        raise RuntimeError(f"{label} contains NaN or inf: {_as_list(tensor)}")


def _assert_within_limits(robot, qpos, tolerance: float) -> None:
    lower, upper = robot.control_limits["position"]
    below = qpos < lower - tolerance
    above = qpos > upper + tolerance
    if bool(below.any() or above.any()):
        raise RuntimeError(
            "joint limit violation: "
            f"qpos={_as_list(qpos)} lower={_as_list(lower)} upper={_as_list(upper)}"
        )


def _reset_robot(robot, env, steps: int) -> None:
    import torch as th

    reset_qpos = th.as_tensor(robot.default_joint_pos, dtype=th.float32)
    robot.set_joint_positions(reset_qpos, drive=False)
    robot.keep_still()
    for _ in range(steps):
        env.step(reset_qpos)


def _drive_to_qpos(env, robot, target_qpos, steps: int) -> None:
    for step in range(steps):
        env.step(target_qpos)
        qpos = robot.get_joint_positions()
        _assert_finite(f"joint positions while driving target at step {step}", qpos)


def _parse_urdf_vec(value: str | None, default: tuple[float, float, float]) -> list[float]:
    if value is None:
        return list(default)
    return [float(part) for part in value.split()]


def _load_eef_fk_specs() -> list[dict]:
    joints = {joint.attrib["name"]: joint for joint in ET.parse(URDF_PATH).getroot().findall("joint")}
    specs = []
    for joint_name in EEF_CHAIN_JOINTS:
        if joint_name not in joints:
            raise RuntimeError(f"missing joint {joint_name!r} in {URDF_PATH}")
        joint = joints[joint_name]
        origin = joint.find("origin")
        axis = joint.find("axis")
        specs.append(
            {
                "name": joint_name,
                "type": joint.attrib.get("type", "fixed"),
                "xyz": _parse_urdf_vec(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
                "rpy": _parse_urdf_vec(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
                "axis": _parse_urdf_vec(axis.attrib.get("xyz") if axis is not None else None, (0.0, 0.0, 1.0)),
            }
        )
    return specs


def _hmat(position=None, rotation=None):
    import numpy as np

    mat = np.eye(4)
    if position is not None:
        mat[:3, 3] = position
    if rotation is not None:
        mat[:3, :3] = rotation
    return mat


def _pose_hmat(position, quat):
    import numpy as np
    from scipy.spatial.transform import Rotation

    return _hmat(
        position=np.asarray(_as_list(position), dtype=float),
        rotation=Rotation.from_quat(_as_list(quat)).as_matrix(),
    )


def _fk_base_to_eef(robot, fk_specs: list[dict]):
    import numpy as np
    from scipy.spatial.transform import Rotation

    qpos = robot.get_joint_positions()
    qpos_by_name = {name: float(qpos[robot.dof_names_ordered.index(name)]) for name in robot.dof_names_ordered}
    base_to_eef = np.eye(4)
    for spec in fk_specs:
        base_to_eef = base_to_eef @ _hmat(
            position=np.asarray(spec["xyz"], dtype=float),
            rotation=Rotation.from_euler("xyz", spec["rpy"]).as_matrix(),
        )
        if spec["type"] != "fixed":
            axis = np.asarray(spec["axis"], dtype=float)
            axis = axis / np.linalg.norm(axis)
            base_to_eef = base_to_eef @ _hmat(
                rotation=Rotation.from_rotvec(axis * qpos_by_name[spec["name"]]).as_matrix()
            )
    return base_to_eef


def _rotation_error_rad(expected_rotation, actual_rotation) -> float:
    from scipy.spatial.transform import Rotation

    return float((Rotation.from_matrix(expected_rotation).inv() * Rotation.from_matrix(actual_rotation)).magnitude())


def _eef_pose_error(robot, fk_specs: list[dict]) -> dict:
    import numpy as np

    base_pos, base_quat = robot.links["base_link"].get_position_orientation()
    expected = _pose_hmat(base_pos, base_quat) @ _fk_base_to_eef(robot, fk_specs)
    actual = _pose_hmat(robot.get_eef_position(), robot.get_eef_orientation())
    link_pos, link_quat = robot.links["gripper_frame_link"].get_position_orientation()
    direct_link = _pose_hmat(link_pos, link_quat)
    return {
        "expected": expected,
        "actual": actual,
        "direct_link": direct_link,
        "pos_err": float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
        "rot_err": _rotation_error_rad(expected[:3, :3], actual[:3, :3]),
        "link_pos_err": float(np.linalg.norm(actual[:3, 3] - direct_link[:3, 3])),
        "link_rot_err": _rotation_error_rad(direct_link[:3, :3], actual[:3, :3]),
        "quat_norm": float(np.linalg.norm(_as_list(robot.get_eef_orientation()))),
    }


def _format_vector(vector) -> str:
    return "[" + ", ".join(f"{float(value):+.5f}" for value in vector) + "]"


def _test_eef_pose(args: argparse.Namespace, env, robot) -> None:
    import numpy as np
    import torch as th

    print("phase=eef_pose", flush=True)
    fk_specs = _load_eef_fk_specs()

    robot_pos, robot_quat = robot.get_position_orientation()
    robot2world = _pose_hmat(robot_pos, robot_quat)
    world2robot = np.linalg.inv(robot2world)
    identity_err = float(np.linalg.norm(world2robot @ robot2world - np.eye(4)))
    print(
        f"world2robot_check identity_err={identity_err:.9f} "
        f"robot_pos={_format_vector(robot_pos)}",
        flush=True,
    )
    if identity_err > args.eef_transform_tolerance:
        raise RuntimeError(f"world2robot inverse error {identity_err:.9f} exceeded tolerance")

    targets = [
        ("default", th.tensor(robot.default_joint_pos, dtype=th.float32)),
        ("reach_left", th.tensor([-0.35, -0.35, 0.85, 0.25, -0.45, 0.5], dtype=th.float32)),
        ("reach_right", th.tensor([0.45, -0.75, 1.20, -0.30, 0.55, 0.5], dtype=th.float32)),
    ]

    for label, target in targets:
        _drive_to_qpos(env, robot, target, args.eef_settle_steps)
        errors = _eef_pose_error(robot, fk_specs)
        actual = errors["actual"]
        quat = robot.get_eef_orientation()
        print(
            f"eef_pose label={label} pos={_format_vector(actual[:3, 3])} "
            f"quat={_format_vector(quat)} quat_norm={errors['quat_norm']:.9f} "
            f"fk_pos_err_m={errors['pos_err']:.9f} fk_rot_err_rad={errors['rot_err']:.9f} "
            f"link_pos_err_m={errors['link_pos_err']:.9f} link_rot_err_rad={errors['link_rot_err']:.9f}",
            flush=True,
        )
        if abs(errors["quat_norm"] - 1.0) > args.eef_quat_tolerance:
            raise RuntimeError(f"EEF quaternion norm is not unit length: {errors['quat_norm']:.9f}")
        if errors["pos_err"] > args.eef_fk_pos_tolerance:
            raise RuntimeError(f"EEF FK position error {errors['pos_err']:.9f} exceeded tolerance")
        if errors["rot_err"] > args.eef_fk_rot_tolerance:
            raise RuntimeError(f"EEF FK rotation error {errors['rot_err']:.9f} exceeded tolerance")
        if errors["link_pos_err"] > args.eef_fk_pos_tolerance or errors["link_rot_err"] > args.eef_fk_rot_tolerance:
            raise RuntimeError("get_eef pose does not match gripper_frame_link pose")

    axes = _eef_pose_error(robot, fk_specs)["actual"][:3, :3]
    gripper_pos, gripper_quat = robot.links["gripper_link"].get_position_orientation()
    eef_pos, eef_quat = robot.links["gripper_frame_link"].get_position_orientation()
    gripper_to_eef = np.linalg.inv(_pose_hmat(gripper_pos, gripper_quat)) @ _pose_hmat(eef_pos, eef_quat)
    relative_axes = gripper_to_eef[:3, :3]
    print(
        "eef_axes_world "
        f"x={_format_vector(axes[:, 0])} y={_format_vector(axes[:, 1])} z={_format_vector(axes[:, 2])}",
        flush=True,
    )
    print(
        "eef_axes_in_gripper "
        f"x={_format_vector(relative_axes[:, 0])} "
        f"y={_format_vector(relative_axes[:, 1])} "
        f"z={_format_vector(relative_axes[:, 2])}",
        flush=True,
    )

    _drive_to_qpos(env, robot, targets[0][1], args.eef_settle_steps)
    prev_errors = _eef_pose_error(robot, fk_specs)
    prev_actual = prev_errors["actual"]
    max_step_pos = 0.0
    max_step_rot = 0.0
    max_fk_pos = prev_errors["pos_err"]
    max_fk_rot = prev_errors["rot_err"]
    for step in range(1, args.eef_track_steps + 1):
        alpha = step / args.eef_track_steps
        target = (1.0 - alpha) * targets[0][1] + alpha * targets[2][1]
        env.step(target)
        errors = _eef_pose_error(robot, fk_specs)
        actual = errors["actual"]
        max_step_pos = max(max_step_pos, float(np.linalg.norm(actual[:3, 3] - prev_actual[:3, 3])))
        max_step_rot = max(max_step_rot, _rotation_error_rad(prev_actual[:3, :3], actual[:3, :3]))
        max_fk_pos = max(max_fk_pos, errors["pos_err"])
        max_fk_rot = max(max_fk_rot, errors["rot_err"])
        prev_actual = actual

    print(
        f"eef_track steps={args.eef_track_steps} max_step_pos_m={max_step_pos:.9f} "
        f"max_step_rot_rad={max_step_rot:.9f} max_fk_pos_err_m={max_fk_pos:.9f} "
        f"max_fk_rot_err_rad={max_fk_rot:.9f}",
        flush=True,
    )
    if max_step_pos > args.max_eef_step_motion:
        raise RuntimeError(f"EEF step position jump {max_step_pos:.9f} exceeded tolerance")
    if max_step_rot > args.max_eef_step_rotation:
        raise RuntimeError(f"EEF step rotation jump {max_step_rot:.9f} exceeded tolerance")
    if max_fk_pos > args.eef_fk_pos_tolerance or max_fk_rot > args.eef_fk_rot_tolerance:
        raise RuntimeError("dynamic EEF pose diverged from URDF FK")


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
        "objects": [],
        "robots": [
            {
                "type": "SO101",
                "name": "so101",
                "position": [0.0, -0.15, 0.75],
                "obs_modalities": [],
                "grasping_mode": "assisted",
                "action_type": "continuous",
                "action_normalize": False,
                "controller_config": {
                    "arm_0": {
                        "name": "JointController",
                        "motor_type": "position",
                        "command_input_limits": None,
                        "command_output_limits": None,
                        "use_delta_commands": False,
                        "use_impedances": False,
                    },
                    "gripper_0": {
                        "name": "JointController",
                        "motor_type": "position",
                        "command_input_limits": None,
                        "command_output_limits": None,
                        "use_delta_commands": False,
                        "use_impedances": False,
                    },
                },
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
    if robot.action_dim != 6:
        raise RuntimeError(f"expected action_dim=6, got {robot.action_dim}")
    return og, env, robot


def run(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    import torch as th

    og = None
    env = None
    error = None
    try:
        og, env, robot = _build_env(args)
        arm_indices = [robot.dof_names_ordered.index(joint_name) for joint_name in ARM_JOINTS]
        print(
            "SO101 joint-control smoke: "
            f"root_link={robot.root_link_name} action_dim={robot.action_dim} dofs={robot.dof_names_ordered}",
            flush=True,
        )

        og.sim.play()
        if args.eef_only:
            _test_eef_pose(args, env, robot)
            print("PASS: SO101 EEF pose matches URDF FK and tracks smoothly", flush=True)
            return

        for joint_name, dof_idx in zip(ARM_JOINTS, arm_indices):
            for delta in (args.delta, -args.delta):
                _reset_robot(robot, env, args.settle_steps)
                start_qpos = robot.get_joint_positions().clone()
                target = start_qpos.clone()
                target[dof_idx] = target[dof_idx] + delta
                lower, upper = robot.control_limits["position"]
                target[dof_idx] = th.clamp(target[dof_idx], lower[dof_idx], upper[dof_idx])

                for _ in range(args.steps_per_target):
                    env.step(target)
                    qpos = robot.get_joint_positions()
                    _assert_finite("joint positions", qpos)
                    _assert_within_limits(robot, qpos, args.limit_tolerance)

                final_qpos = robot.get_joint_positions()
                error_rad = abs(float(final_qpos[dof_idx] - target[dof_idx]))
                print(
                    f"joint={joint_name:<13} delta={delta:+.3f} "
                    f"start={float(start_qpos[dof_idx]):+.4f} "
                    f"target={float(target[dof_idx]):+.4f} "
                    f"final={float(final_qpos[dof_idx]):+.4f} err={error_rad:.5f}",
                    flush=True,
                )
                if error_rad > args.tolerance:
                    raise RuntimeError(
                        f"{joint_name} failed to reach target: err={error_rad:.5f} > {args.tolerance:.5f}"
                    )

        _reset_robot(robot, env, args.settle_steps)
        hold_start = robot.get_joint_positions().clone()
        base_start = th.as_tensor(robot.get_position_orientation()[0], dtype=th.float32).clone()
        for step in range(args.hold_steps):
            env.step(hold_start)
            qpos = robot.get_joint_positions()
            _assert_finite("hold joint positions", qpos)
            _assert_within_limits(robot, qpos, args.limit_tolerance)
            if step % args.log_every == 0 or step == args.hold_steps - 1:
                base_pos = th.as_tensor(robot.get_position_orientation()[0], dtype=th.float32)
                joint_drift = th.max(th.abs(qpos[:5] - hold_start[:5])).item()
                base_drift = th.linalg.norm(base_pos - base_start).item()
                print(
                    f"hold_step={step:04d} joint_drift_rad={joint_drift:.6f} "
                    f"base_drift_m={base_drift:.6f}",
                    flush=True,
                )
                if joint_drift > args.max_hold_drift:
                    raise RuntimeError(
                        f"hold joint drift {joint_drift:.6f} exceeded {args.max_hold_drift:.6f} rad"
                    )
                if base_drift > args.max_base_drift:
                    raise RuntimeError(f"base drift {base_drift:.6f} exceeded {args.max_base_drift:.6f} m")

        if args.test_eef_pose:
            _test_eef_pose(args, env, robot)
            print("PASS: SO101 EEF pose matches URDF FK and tracks smoothly", flush=True)

        print("PASS: all SO101 arm joints reached +/- targets and held stable", flush=True)
    except BaseException as exc:
        error = exc
        raise
    finally:
        if env is not None and og is not None and og.app is not None:
            try:
                og.shutdown()
            except BaseException as shutdown_error:
                if error is None:
                    raise
                print(f"WARN: ignored shutdown error after failure: {shutdown_error}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run OmniGibson headlessly")
    parser.add_argument("--steps-per-target", type=int, default=100, help="control steps for each joint target")
    parser.add_argument("--settle-steps", type=int, default=10, help="settle steps after resetting qpos")
    parser.add_argument("--hold-steps", type=int, default=1000, help="continuous hold stability steps")
    parser.add_argument("--log-every", type=int, default=250, help="hold logging interval")
    parser.add_argument("--delta", type=float, default=0.5, help="joint target delta in radians")
    parser.add_argument("--tolerance", type=float, default=0.05, help="max final target error in radians")
    parser.add_argument("--limit-tolerance", type=float, default=0.002, help="allowed joint limit tolerance")
    parser.add_argument("--max-hold-drift", type=float, default=0.05, help="max arm drift while holding")
    parser.add_argument("--max-base-drift", type=float, default=0.001, help="max base drift while holding")
    parser.add_argument("--test-eef-pose", action="store_true", help="also run EEF pose / FK checks")
    parser.add_argument("--eef-only", action="store_true", help="skip joint sweep and only run EEF pose / FK checks")
    parser.add_argument("--eef-settle-steps", type=int, default=80, help="steps to settle each EEF pose target")
    parser.add_argument("--eef-track-steps", type=int, default=80, help="steps for dynamic EEF tracking check")
    parser.add_argument("--eef-fk-pos-tolerance", type=float, default=0.001, help="max EEF-vs-FK position error")
    parser.add_argument("--eef-fk-rot-tolerance", type=float, default=0.01, help="max EEF-vs-FK rotation error")
    parser.add_argument("--eef-quat-tolerance", type=float, default=0.001, help="max EEF quaternion norm error")
    parser.add_argument(
        "--eef-transform-tolerance",
        type=float,
        default=1e-6,
        help="max world2robot inverse identity error",
    )
    parser.add_argument("--max-eef-step-motion", type=float, default=0.02, help="max EEF step translation jump")
    parser.add_argument("--max-eef-step-rotation", type=float, default=0.15, help="max EEF step rotation jump")
    parser.add_argument("--use-gpu-dynamics", action="store_true", help="enable GPU dynamics")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()

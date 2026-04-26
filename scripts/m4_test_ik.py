#!/usr/bin/env python3
"""M4 smoke test: validate SO-101 Lula descriptor and IK solver."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SO101_ROOT = REPO_ROOT / "assert/SO101"
DESCRIPTOR_PATH = SO101_ROOT / "lula/so101_robot_descriptor.yaml"
URDF_PATH = SO101_ROOT / "so101_new_calib.urdf"
EEF_NAME = "gripper_frame_link"

RESET_JOINT_POS = np.array([0.0, -0.5, 1.0, 0.0, 0.0], dtype=float)
TARGET_JOINT_CONFIGS = (
    np.array([0.0, -0.5, 1.0, 0.0, 0.0], dtype=float),
    np.array([0.35, -0.65, 0.95, -0.25, 0.4], dtype=float),
    np.array([-0.35, -0.65, 0.95, -0.25, -0.4], dtype=float),
    np.array([0.65, -0.35, 0.75, 0.25, 0.8], dtype=float),
    np.array([-0.65, -0.35, 0.75, 0.25, -0.8], dtype=float),
    np.array([0.25, -1.0, 1.2, 0.35, 1.2], dtype=float),
    np.array([-0.25, -1.0, 1.2, 0.35, -1.2], dtype=float),
    np.array([0.0, -0.9, 1.35, -0.55, 1.6], dtype=float),
)


def _result_value(result, name: str):
    value = getattr(result, name)
    return value() if callable(value) else value


def _pose_to_homo(pose) -> np.ndarray:
    matrix = pose.matrix
    return np.array(matrix() if callable(matrix) else matrix, dtype=float).reshape(4, 4)


def _orientation_error(result) -> float:
    return max(
        abs(float(_result_value(result, "x_axis_orientation_error"))),
        abs(float(_result_value(result, "y_axis_orientation_error"))),
        abs(float(_result_value(result, "z_axis_orientation_error"))),
    )


def _make_empty_env(og):
    cfg = {
        "scene": {"type": "Scene"},
        "objects": [],
        "robots": [],
        "task": {"type": "DummyTask"},
        "env": {
            "action_frequency": 30,
            "physics_frequency": 120,
            "rendering_frequency": 30,
        },
    }
    return og.Environment(configs=cfg)


def _run(args: argparse.Namespace, og) -> None:
    from ik_solver import IKSolver

    solver = IKSolver(
        robot_description_path=str(DESCRIPTOR_PATH),
        robot_urdf_path=str(URDF_PATH),
        eef_name=EEF_NAME,
        reset_joint_pos=RESET_JOINT_POS,
        world2robot_homo=np.eye(4),
    )
    print("IKSolver initialized", flush=True)

    successes = 0
    for index, target_q in enumerate(TARGET_JOINT_CONFIGS):
        target_pose = _pose_to_homo(solver.kinematics.pose(target_q, EEF_NAME))
        result = solver.solve(
            target_pose,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=args.orientation_tolerance,
            max_iterations=args.max_iterations,
            initial_joint_pos=RESET_JOINT_POS,
        )
        success = bool(_result_value(result, "success"))
        pos_err = float(_result_value(result, "position_error"))
        ori_err = _orientation_error(result)
        passed = success and pos_err < args.position_tolerance and ori_err < args.orientation_tolerance
        successes += int(passed)

        solved_q = np.array(_result_value(result, "cspace_position"), dtype=float)
        target_pos = target_pose[:3, 3]
        print(
            "target={:02d} success={} pos_err_m={:.6f} ori_err_rad={:.6f} "
            "target_pos={} solved_q={}".format(
                index,
                passed,
                pos_err,
                ori_err,
                np.round(target_pos, 4).tolist(),
                np.round(solved_q, 4).tolist(),
            ),
            flush=True,
        )

    success_rate = successes / len(TARGET_JOINT_CONFIGS)
    print(f"success_rate={successes}/{len(TARGET_JOINT_CONFIGS)} ({success_rate:.1%})", flush=True)
    if success_rate < args.min_success_rate:
        raise RuntimeError(f"IK success rate {success_rate:.1%} below required {args.min_success_rate:.1%}")

    outside_target = np.eye(4)
    outside_target[:3, 3] = [1.0, 0.0, 0.4]
    outside = solver.solve(
        outside_target,
        position_tolerance=args.position_tolerance,
        orientation_tolerance=args.orientation_tolerance,
        max_iterations=args.max_iterations,
    )
    outside_success = bool(_result_value(outside, "success"))
    outside_pos_err = float(_result_value(outside, "position_error"))
    print(f"outside_target_success={outside_success} pos_err_m={outside_pos_err:.6f}", flush=True)
    if outside_success:
        raise RuntimeError("out-of-workspace IK target unexpectedly succeeded")

    print(
        "PASS: Lula descriptor loaded and IK solved reachable SO-101 FK targets "
        f"at >= {args.min_success_rate:.0%} success rate",
        flush=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--gui", dest="headless", action="store_false")
    parser.add_argument("--position-tolerance", type=float, default=0.005)
    parser.add_argument("--orientation-tolerance", type=float, default=0.1)
    parser.add_argument("--min-success-rate", type=float, default=0.8)
    parser.add_argument("--max-iterations", type=int, default=300)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not DESCRIPTOR_PATH.exists():
        raise FileNotFoundError(DESCRIPTOR_PATH)
    if not URDF_PATH.exists():
        raise FileNotFoundError(URDF_PATH)

    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    sys.path.insert(0, str(REPO_ROOT))

    import omnigibson as og
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = False

    error = None
    try:
        _make_empty_env(og)
        _run(args, og)
    except BaseException as exc:
        error = exc
        raise
    finally:
        if error is None and og.app is not None:
            try:
                og.shutdown()
            except SystemExit:
                pass


if __name__ == "__main__":
    main()

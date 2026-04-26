#!/usr/bin/env python3
"""M5 smoke test: load SO101 as an OmniGibson ManipulationRobot."""

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


def run(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    import torch as th

    import omnigibson as og
    from omnigibson.macros import gm
    from omnigibson.robots import REGISTERED_ROBOTS

    from so101_robot import SO101

    gm.USE_GPU_DYNAMICS = args.use_gpu_dynamics
    gm.ENABLE_FLATCACHE = False

    if "SO101" not in REGISTERED_ROBOTS:
        raise RuntimeError("SO101 was not registered in OmniGibson's robot registry")

    env = None
    error = None
    try:
        cfg = {
            "scene": {"type": "Scene"},
            "objects": [],
            "robots": [
                {
                    "type": "SO101",
                    "name": "so101",
                    "position": [0.0, -0.15, 0.75],
                    "obs_modalities": ["proprio"],
                    "grasping_mode": "assisted",
                    "action_type": "continuous",
                    "action_normalize": True,
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
            raise RuntimeError(f"expected env.robots[0] to be SO101, got {type(robot).__name__}")
        if robot.action_dim != 6:
            raise RuntimeError(f"expected action_dim=6, got {robot.action_dim}")
        if robot.grasping_mode != "assisted":
            raise RuntimeError(f"expected assisted grasping mode, got {robot.grasping_mode}")

        expected_arm_joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        if robot.arm_joint_names[robot.default_arm] != expected_arm_joints:
            raise RuntimeError(f"unexpected arm joints: {robot.arm_joint_names[robot.default_arm]}")
        if robot.finger_joint_names[robot.default_arm] != ["gripper"]:
            raise RuntimeError(f"unexpected finger joints: {robot.finger_joint_names[robot.default_arm]}")
        if robot.eef_link_names[robot.default_arm] != "gripper_frame_link":
            raise RuntimeError(f"unexpected eef link: {robot.eef_link_names[robot.default_arm]}")

        print(
            "SO101 robot: "
            f"type={type(robot).__name__} root_link={robot.root_link_name} action_dim={robot.action_dim} "
            f"controllers={list(robot.controllers.keys())} dofs={robot.dof_names_ordered}",
            flush=True,
        )
        print(
            "properties: "
            f"arm_joints={robot.arm_joint_names[robot.default_arm]} "
            f"finger_links={robot.finger_link_names[robot.default_arm]} "
            f"finger_joints={robot.finger_joint_names[robot.default_arm]}",
            flush=True,
        )

        og.sim.play()
        zero_action = th.zeros(robot.action_dim, dtype=th.float32)
        for step in range(args.steps):
            env.step(zero_action)
            if step % args.log_every == 0 or step == args.steps - 1:
                qpos = robot.get_joint_positions()
                pos = robot.get_position_orientation()[0]
                if th.any(th.isnan(qpos)) or th.any(th.isnan(pos)):
                    raise RuntimeError(f"NaN detected at step {step}")
                print(f"step={step:04d} pos={_as_list(pos)} qpos={_as_list(qpos)}", flush=True)

        print("PASS: SO101 ManipulationRobot loaded with action_dim=6 and assisted grasping", flush=True)
    except BaseException as exc:
        error = exc
        raise
    finally:
        if env is not None and og.app is not None:
            try:
                og.shutdown()
            except BaseException as shutdown_error:
                if error is None:
                    raise
                print(f"WARN: ignored shutdown error after failure: {shutdown_error}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run OmniGibson headlessly")
    parser.add_argument("--steps", type=int, default=10, help="number of simulation steps")
    parser.add_argument("--log-every", type=int, default=5, help="logging interval")
    parser.add_argument("--use-gpu-dynamics", action="store_true", help="enable GPU dynamics")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()

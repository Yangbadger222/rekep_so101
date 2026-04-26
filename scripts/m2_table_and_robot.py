#!/usr/bin/env python3
"""M3 smoke test: load a primitive table and fixed SO-101 USDObject."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SO101_USD = REPO_ROOT / "assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd"


def _parse_vec2(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected two comma-separated numbers, e.g. 0,-0.15")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _parse_vec3(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated numbers, e.g. 0.8,0.6,0.05")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _as_position(value) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _drift(a: list[float], b: list[float]) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def run(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    import omnigibson as og
    import torch as th
    from omnigibson.macros import gm
    from omnigibson.objects import PrimitiveObject, USDObject

    gm.USE_GPU_DYNAMICS = args.use_gpu_dynamics
    gm.ENABLE_FLATCACHE = False

    error = None
    try:
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
        env = og.Environment(configs=cfg)
        og.sim.stop()

        table_thickness = args.table_size[2]
        table_center_z = args.table_top_height - table_thickness / 2.0
        table_position = [0.0, 0.0, table_center_z]
        robot_position = [args.robot_xy[0], args.robot_xy[1], args.table_top_height]

        table = PrimitiveObject(
            name="table",
            primitive_type="Cube",
            category="table",
            fixed_base=True,
            visual_only=False,
            scale=args.table_size,
            rgba=(0.45, 0.45, 0.42, 1.0),
        )
        env.scene.add_object(table)
        table.set_position_orientation(
            position=th.tensor(table_position, dtype=th.float32),
            orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
        )

        so101 = USDObject(
            name="so101",
            usd_path=str(args.so101_usd),
            category="robot",
            fixed_base=True,
            visual_only=False,
            kinematic_only=False,
            self_collisions=False,
        )
        env.scene.add_object(so101)
        so101.set_position_orientation(
            position=th.tensor(robot_position, dtype=th.float32),
            orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
        )

        og.sim.play()
        table_start = _as_position(table.get_position_orientation()[0])
        robot_start = _as_position(so101.get_position_orientation()[0])
        print(
            "scene: "
            f"table_size={args.table_size} table_top_height={args.table_top_height:.3f} "
            f"table_pos={table_start} robot_pos={robot_start} root_link={so101.root_link_name}",
            flush=True,
        )

        for step in range(args.steps):
            og.sim.step()
            if step % args.log_every == 0 or step == args.steps - 1:
                table_pos = _as_position(table.get_position_orientation()[0])
                robot_pos = _as_position(so101.get_position_orientation()[0])
                table_drift = _drift(table_pos, table_start)
                robot_drift = _drift(robot_pos, robot_start)
                print(
                    f"step={step:04d} table_pos={table_pos} table_drift_m={table_drift:.6f} "
                    f"so101_pos={robot_pos} so101_drift_m={robot_drift:.6f}",
                    flush=True,
                )
                if table_drift > args.max_drift:
                    raise RuntimeError(
                        f"table drift {table_drift:.6f} m exceeded limit {args.max_drift:.6f} m"
                    )
                if robot_drift > args.max_drift:
                    raise RuntimeError(
                        f"SO-101 drift {robot_drift:.6f} m exceeded limit {args.max_drift:.6f} m"
                    )

        print(f"PASS: table + SO-101 stayed within {args.max_drift:.6f} m for {args.steps} steps", flush=True)
    except BaseException as exc:
        error = exc
    finally:
        if og.app is not None:
            try:
                og.shutdown()
            except SystemExit:
                if error is None:
                    raise

    if error is not None:
        raise error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--so101-usd", type=Path, default=DEFAULT_SO101_USD)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--log-every", type=int, default=100)
    parser.add_argument("--table-size", type=_parse_vec3, default=[0.8, 0.6, 0.05])
    parser.add_argument("--table-top-height", type=float, default=0.75)
    parser.add_argument("--robot-xy", type=_parse_vec2, default=[0.0, -0.15])
    parser.add_argument("--max-drift", type=float, default=1e-3)
    parser.add_argument("--use-gpu-dynamics", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.so101_usd = args.so101_usd.expanduser().resolve()
    if not args.so101_usd.exists():
        raise FileNotFoundError(args.so101_usd)
    run(args)


if __name__ == "__main__":
    main()

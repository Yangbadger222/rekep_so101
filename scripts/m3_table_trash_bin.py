#!/usr/bin/env python3
"""M3 smoke test: table, fixed SO-101, small trash, and an open-top bin."""

from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SO101_USD = REPO_ROOT / "assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd"


def _parse_vec3(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated numbers")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _as_position(value) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _drift(a: list[float], b: list[float]) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def _has_nan(values: list[float]) -> bool:
    return any(math.isnan(value) for value in values)


def _make_cube(PrimitiveObject, name, size, *, fixed_base, rgba, category="object"):
    scale = size if isinstance(size, list) else None
    cube_size = None if isinstance(size, list) else size
    return PrimitiveObject(
        name=name,
        primitive_type="Cube",
        category=category,
        fixed_base=fixed_base,
        visual_only=False,
        scale=scale,
        size=cube_size,
        rgba=rgba,
    )


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

        table_size = [0.8, 0.6, 0.05]
        table_top = args.table_top_height
        table_center_z = table_top - table_size[2] / 2.0

        table = _make_cube(
            PrimitiveObject,
            "table",
            table_size,
            fixed_base=True,
            rgba=(0.45, 0.45, 0.42, 1.0),
            category="table",
        )
        env.scene.add_object(table)
        table.set_position_orientation(
            position=th.tensor([0.0, 0.0, table_center_z], dtype=th.float32),
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
            position=th.tensor([0.0, -0.15, table_top], dtype=th.float32),
            orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
        )

        trash_specs = [
            ("trash_0", [0.15, 0.0, table_top + args.trash_size / 2.0 + args.drop_height], (1.0, 0.0, 0.0, 1.0)),
            ("trash_1", [0.10, 0.10, table_top + args.trash_size / 2.0 + args.drop_height], (1.0, 0.85, 0.0, 1.0)),
        ]
        trash_objects = []
        for name, position, rgba in trash_specs:
            trash = _make_cube(PrimitiveObject, name, args.trash_size, fixed_base=False, rgba=rgba, category="trash")
            env.scene.add_object(trash)
            trash.set_position_orientation(
                position=th.tensor(position, dtype=th.float32),
                orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
            )
            trash_objects.append(trash)

        bin_center = args.bin_center
        bin_outer = 0.14
        bin_wall = 0.01
        bin_height = 0.08
        bin_base = 0.01
        wall_z = table_top + bin_base + bin_height / 2.0
        base_z = table_top + bin_base / 2.0
        bin_parts = [
            ("trash_bin_base", [bin_outer, bin_outer, bin_base], [bin_center[0], bin_center[1], base_z]),
            (
                "trash_bin_wall_n",
                [bin_outer, bin_wall, bin_height],
                [bin_center[0], bin_center[1] + (bin_outer - bin_wall) / 2.0, wall_z],
            ),
            (
                "trash_bin_wall_s",
                [bin_outer, bin_wall, bin_height],
                [bin_center[0], bin_center[1] - (bin_outer - bin_wall) / 2.0, wall_z],
            ),
            (
                "trash_bin_wall_e",
                [bin_wall, bin_outer, bin_height],
                [bin_center[0] + (bin_outer - bin_wall) / 2.0, bin_center[1], wall_z],
            ),
            (
                "trash_bin_wall_w",
                [bin_wall, bin_outer, bin_height],
                [bin_center[0] - (bin_outer - bin_wall) / 2.0, bin_center[1], wall_z],
            ),
        ]
        bin_objects = []
        for name, size, position in bin_parts:
            part = _make_cube(
                PrimitiveObject,
                name,
                size,
                fixed_base=True,
                rgba=(0.0, 0.2, 1.0, 1.0),
                category="trash_bin",
            )
            env.scene.add_object(part)
            part.set_position_orientation(
                position=th.tensor(position, dtype=th.float32),
                orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
            )
            bin_objects.append(part)

        og.sim.play()
        table_start = _as_position(table.get_position_orientation()[0])
        robot_start = _as_position(so101.get_position_orientation()[0])
        trash_start = {obj.name: _as_position(obj.get_position_orientation()[0]) for obj in trash_objects}
        bin_start = {obj.name: _as_position(obj.get_position_orientation()[0]) for obj in bin_objects}
        print(
            "scene: "
            f"table_top={table_top:.3f} trash_size={args.trash_size:.3f} drop_height={args.drop_height:.3f} "
            f"bin_center={bin_center} root_link={so101.root_link_name}",
            flush=True,
        )

        step_times = []
        nan_count = 0
        for step in range(args.steps):
            start = time.perf_counter()
            og.sim.step()
            step_times.append(time.perf_counter() - start)

            table_pos = _as_position(table.get_position_orientation()[0])
            robot_pos = _as_position(so101.get_position_orientation()[0])
            trash_positions = {obj.name: _as_position(obj.get_position_orientation()[0]) for obj in trash_objects}
            bin_positions = {obj.name: _as_position(obj.get_position_orientation()[0]) for obj in bin_objects}
            all_positions = [table_pos, robot_pos, *trash_positions.values(), *bin_positions.values()]
            nan_count += sum(1 for pos in all_positions if _has_nan(pos))

            if step % args.log_every == 0 or step == args.steps - 1:
                table_drift = _drift(table_pos, table_start)
                robot_drift = _drift(robot_pos, robot_start)
                trash_report = " ".join(
                    f"{name}_pos={pos} {name}_offset_m={_drift(pos, trash_start[name]):.6f}"
                    for name, pos in trash_positions.items()
                )
                print(
                    f"step={step:04d} table_drift_m={table_drift:.6f} so101_drift_m={robot_drift:.6f} "
                    f"{trash_report}",
                    flush=True,
                )

        final_table = _as_position(table.get_position_orientation()[0])
        final_robot = _as_position(so101.get_position_orientation()[0])
        final_trash = {obj.name: _as_position(obj.get_position_orientation()[0]) for obj in trash_objects}
        final_bin = {obj.name: _as_position(obj.get_position_orientation()[0]) for obj in bin_objects}
        table_drift = _drift(final_table, table_start)
        robot_drift = _drift(final_robot, robot_start)
        max_bin_drift = max(_drift(final_bin[name], start) for name, start in bin_start.items())
        avg_step_ms = sum(step_times) / len(step_times) * 1000.0
        max_step_ms = max(step_times) * 1000.0
        expected_trash_z = table_top + args.trash_size / 2.0

        print("metrics:", flush=True)
        print(f"  so101_drift_mm={robot_drift * 1000.0:.3f}", flush=True)
        print(f"  table_drift_mm={table_drift * 1000.0:.3f}", flush=True)
        print(f"  max_bin_drift_mm={max_bin_drift * 1000.0:.3f}", flush=True)
        for name, pos in final_trash.items():
            offset = _drift(pos, trash_start[name])
            print(f"  {name}_final_pos={pos}", flush=True)
            print(f"  {name}_final_offset_mm={offset * 1000.0:.3f}", flush=True)
            print(f"  {name}_height_above_table_mm={(pos[2] - table_top) * 1000.0:.3f}", flush=True)
            if pos[2] < expected_trash_z - args.trash_z_tolerance:
                raise RuntimeError(f"{name} appears to penetrate the table: z={pos[2]:.6f}")
            if pos[2] > expected_trash_z + args.trash_settle_tolerance:
                raise RuntimeError(f"{name} did not settle near the table: z={pos[2]:.6f}")
        print(f"  avg_step_time_ms={avg_step_ms:.3f}", flush=True)
        print(f"  max_step_time_ms={max_step_ms:.3f}", flush=True)
        print(f"  nan_count={nan_count}", flush=True)

        if table_drift > args.max_fixed_drift:
            raise RuntimeError(f"table drift {table_drift:.6f} m exceeded limit {args.max_fixed_drift:.6f}")
        if robot_drift > args.max_fixed_drift:
            raise RuntimeError(f"SO-101 drift {robot_drift:.6f} m exceeded limit {args.max_fixed_drift:.6f}")
        if max_bin_drift > args.max_fixed_drift:
            raise RuntimeError(f"trash bin drift {max_bin_drift:.6f} m exceeded limit {args.max_fixed_drift:.6f}")
        if nan_count:
            raise RuntimeError(f"encountered {nan_count} NaN position samples")

        print("PASS: table + SO-101 + trash + bin scene stayed stable", flush=True)
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
    parser.add_argument("--table-top-height", type=float, default=0.75)
    parser.add_argument("--trash-size", type=float, default=0.025)
    parser.add_argument("--drop-height", type=float, default=0.015)
    parser.add_argument("--bin-center", type=_parse_vec3, default=[-0.15, 0.15, 0.0])
    parser.add_argument("--max-fixed-drift", type=float, default=1e-3)
    parser.add_argument("--trash-z-tolerance", type=float, default=0.005)
    parser.add_argument("--trash-settle-tolerance", type=float, default=0.03)
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

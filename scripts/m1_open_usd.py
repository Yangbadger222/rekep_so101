#!/usr/bin/env python3
"""M1 smoke test: load SO-101 USD in OmniGibson / Isaac and step physics."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_USD_PATH = REPO_ROOT / "assert/SO101/so101_new_calib/so101_new_calib.usd"
DEFAULT_USDOBJECT_USD_PATH = REPO_ROOT / "assert/SO101/so101_new_calib/so101_new_calib_og_usdobject.usd"
EXPECTED_ROOT_ASSETS = (
    "configuration/so101_new_calib_base.usd",
    "configuration/so101_new_calib_physics.usd",
    "configuration/so101_new_calib_robot.usd",
    "configuration/so101_new_calib_sensor.usd",
)


def _parse_vec3(value: str) -> list[float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated numbers, e.g. 0,0,0.5")
    try:
        return [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _validate_metadata(usd_path: Path) -> None:
    try:
        from pxr import Sdf, Usd, UsdGeom
    except ImportError as exc:
        raise RuntimeError(
            "USD Python bindings are required for --metadata-only. "
            "Run inside an Isaac/OmniGibson Python environment or install usd-core."
        ) from exc

    layer = Sdf.Layer.FindOrOpen(str(usd_path))
    if layer is None:
        raise RuntimeError(f"failed to open USD layer: {usd_path}")

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"failed to open USD stage: {usd_path}")

    root_text = layer.ExportToString()
    missing_assets = [asset for asset in EXPECTED_ROOT_ASSETS if asset not in root_text]
    if missing_assets:
        raise RuntimeError(f"missing root USD asset references: {missing_assets}")

    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    default_prim = stage.GetDefaultPrim()
    if meters_per_unit != 1.0:
        raise RuntimeError(f"expected metersPerUnit=1.0, got {meters_per_unit}")
    if up_axis != UsdGeom.Tokens.z:
        raise RuntimeError(f"expected Z-up stage, got {up_axis}")
    if not default_prim:
        raise RuntimeError("stage has no defaultPrim")

    print(f"USD: {usd_path}", flush=True)
    print(f"defaultPrim: {default_prim.GetPath()}", flush=True)
    print(f"metersPerUnit: {meters_per_unit}", flush=True)
    print(f"upAxis: {up_axis}", flush=True)
    print("root asset references:", flush=True)
    for asset in EXPECTED_ROOT_ASSETS:
        print(f"  OK {asset}", flush=True)


def _as_xyz(value) -> list[float]:
    return [float(value[0]), float(value[1]), float(value[2])]


def _get_world_position(prim) -> list[float]:
    import omnigibson.lazy as lazy

    cache = lazy.pxr.UsdGeom.XformCache()
    transform = cache.GetLocalToWorldTransform(prim)
    return _as_xyz(transform.ExtractTranslation())


def _drift(a: list[float], b: list[float]) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def _run_raw_stage(args: argparse.Namespace, usd_path: Path, og) -> None:
    import omnigibson.lazy as lazy
    from omnigibson.utils.usd_utils import add_asset_to_stage

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
    root_path = "/World/scene_0/so101"
    root_prim = add_asset_to_stage(asset_path=str(usd_path), prim_path=root_path)

    xform_api = lazy.pxr.UsdGeom.XformCommonAPI(root_prim)
    xform_api.SetTranslate(lazy.pxr.Gf.Vec3d(*args.position))
    if args.scale != 1.0:
        xform_api.SetScale(lazy.pxr.Gf.Vec3f(args.scale, args.scale, args.scale))

    base_prim = lazy.omni.isaac.core.utils.prims.get_prim_at_path(f"{root_path}/base_link")
    if not base_prim:
        raise RuntimeError(f"failed to find base_link under {root_path}")

    og.sim.play()
    start_pos = _get_world_position(base_prim)
    for step in range(args.steps):
        og.sim.step()
        if step % args.log_every == 0 or step == args.steps - 1:
            pos = _get_world_position(base_prim)
            drift = _drift(pos, start_pos)
            print(f"step={step:04d} base_link_pos={pos} drift_m={drift:.6f}", flush=True)
            if drift > args.max_drift:
                raise RuntimeError(f"SO-101 drift {drift:.6f} m exceeded limit {args.max_drift:.6f} m")

    print(f"PASS: raw USD stage stayed within {args.max_drift:.6f} m for {args.steps} steps", flush=True)


def _run_usdobject(args: argparse.Namespace, usd_path: Path, og) -> None:
    import torch as th
    from omnigibson.objects import USDObject

    print(f"Loading SO-101 as USDObject from {usd_path}", flush=True)
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
    print("Empty OmniGibson environment created", flush=True)
    og.sim.stop()
    print("Simulation stopped for USDObject import", flush=True)
    so101 = USDObject(
        name="so101",
        usd_path=str(usd_path),
        category="robot",
        fixed_base=True,
        visual_only=False,
        kinematic_only=False,
        self_collisions=False,
        scale=args.scale,
    )
    print("USDObject constructed", flush=True)
    try:
        env.scene.add_object(so101)
    except BaseException as exc:
        print(f"USDObject add failed: {type(exc).__name__}: {exc}", flush=True)
        raise
    print("USDObject added to OmniGibson scene", flush=True)
    so101.set_position_orientation(
        position=th.tensor(args.position, dtype=th.float32),
        orientation=th.tensor([0.0, 0.0, 0.0, 1.0], dtype=th.float32),
    )

    og.sim.play()
    start_pos = th.as_tensor(so101.get_position_orientation()[0], dtype=th.float32)
    print(
        f"USDObject: usd_path={usd_path} root_link={so101.root_link_name} start_pos={start_pos.tolist()}",
        flush=True,
    )

    for step in range(args.steps):
        og.sim.step()
        if step % args.log_every == 0 or step == args.steps - 1:
            pos = th.as_tensor(so101.get_position_orientation()[0], dtype=th.float32)
            drift = th.linalg.norm(pos - start_pos).item()
            print(f"step={step:04d} pos={pos.tolist()} drift_m={drift:.6f}", flush=True)
            if drift > args.max_drift:
                raise RuntimeError(f"SO-101 drift {drift:.6f} m exceeded limit {args.max_drift:.6f} m")

    print(f"PASS: SO-101 USDObject stayed within {args.max_drift:.6f} m for {args.steps} steps", flush=True)


def _run_omnigibson(args: argparse.Namespace, usd_path: Path) -> None:
    if args.headless:
        os.environ["OMNIGIBSON_HEADLESS"] = "1"

    import omnigibson as og
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = args.use_gpu_dynamics
    gm.ENABLE_FLATCACHE = False

    error = None
    try:
        print(f"load_mode={args.load_mode} usd_path={usd_path}", flush=True)
        if args.load_mode == "raw":
            _run_raw_stage(args, usd_path, og)
        else:
            _run_usdobject(args, usd_path, og)
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
    parser.add_argument(
        "--usd-path",
        type=Path,
        default=None,
        help="USD path to load. Defaults to the source USD for raw/metadata and the OG wrapper for usdobject.",
    )
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--metadata-only", action="store_true")
    parser.add_argument(
        "--load-mode",
        choices=("raw", "usdobject"),
        default="usdobject",
        help="usdobject validates OG USDObject loading; raw validates direct Isaac stage loading.",
    )
    parser.add_argument("--position", type=_parse_vec3, default=[0.0, 0.0, 0.5])
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--max-drift", type=float, default=1e-3)
    parser.add_argument("--log-every", type=int, default=100)
    parser.add_argument("--use-gpu-dynamics", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    default_usd_path = (
        DEFAULT_SOURCE_USD_PATH if args.metadata_only or args.load_mode == "raw" else DEFAULT_USDOBJECT_USD_PATH
    )
    usd_path = (args.usd_path or default_usd_path).expanduser().resolve()
    if not usd_path.exists():
        raise FileNotFoundError(usd_path)

    if args.metadata_only:
        _validate_metadata(usd_path)
        return

    _run_omnigibson(args, usd_path)


if __name__ == "__main__":
    main()

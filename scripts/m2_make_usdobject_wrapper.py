#!/usr/bin/env python3
"""Generate the SO-101 USDObject compatibility wrapper."""

from __future__ import annotations

import argparse
from pathlib import Path

try:
    from pxr import Sdf, Usd, UsdGeom
except ImportError as exc:  # pragma: no cover - exercised only without USD installed.
    raise SystemExit(
        "Missing pxr USD bindings. Install usd-core or run with PYTHONPATH pointing "
        "to a usd-core target directory, e.g. PYTHONPATH=/tmp/codex-usd-core."
    ) from exc


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO_ROOT / "assert/SO101/so101_new_calib"
SOURCE_USD = ASSET_DIR / "so101_new_calib.usd"
PHYSICS_USD = ASSET_DIR / "configuration/so101_new_calib_physics.usd"
WRAPPER_USD = ASSET_DIR / "so101_new_calib_og_usdobject.usd"
ROOT_PATH = "/so101_new_calib"

JOINT_PARENT_LINKS = {
    "shoulder_pan": "base_link",
    "shoulder_lift": "shoulder_link",
    "elbow_flex": "upper_arm_link",
    "wrist_flex": "lower_arm_link",
    "wrist_roll": "wrist_link",
    "gripper": "gripper_link",
    "gripper_frame_joint": "gripper_link",
}


def generate_wrapper(source_usd: Path, physics_usd: Path, output_usd: Path) -> None:
    if output_usd.exists():
        output_usd.unlink()

    stage = Usd.Stage.CreateNew(str(output_usd))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    root = stage.DefinePrim(ROOT_PATH, "Xform")
    stage.SetDefaultPrim(root)
    root.GetReferences().AddReference(source_usd.name, ROOT_PATH)

    stage.OverridePrim(f"{ROOT_PATH}/root_joint").SetActive(False)
    stage.OverridePrim(f"{ROOT_PATH}/joints").SetActive(False)
    stage.GetRootLayer().Save()

    wrapper_layer = Sdf.Layer.FindOrOpen(str(output_usd))
    physics_layer = Sdf.Layer.FindOrOpen(str(physics_usd))
    if wrapper_layer is None:
        raise RuntimeError(f"failed to open generated wrapper layer: {output_usd}")
    if physics_layer is None:
        raise RuntimeError(f"failed to open physics layer: {physics_usd}")

    for link_name in sorted(set(JOINT_PARENT_LINKS.values())):
        link_spec = Sdf.CreatePrimInLayer(wrapper_layer, f"{ROOT_PATH}/{link_name}")
        link_spec.specifier = Sdf.SpecifierOver

    for joint_name, parent_link in JOINT_PARENT_LINKS.items():
        source_path = Sdf.Path(f"{ROOT_PATH}/joints/{joint_name}")
        target_path = Sdf.Path(f"{ROOT_PATH}/{parent_link}/{joint_name}")
        if not physics_layer.GetPrimAtPath(source_path):
            raise RuntimeError(f"missing source joint spec: {source_path}")
        if not Sdf.CopySpec(physics_layer, source_path, wrapper_layer, target_path):
            raise RuntimeError(f"failed to copy {source_path} to {target_path}")

    wrapper_layer.Save()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-usd", type=Path, default=SOURCE_USD)
    parser.add_argument("--physics-usd", type=Path, default=PHYSICS_USD)
    parser.add_argument("--output-usd", type=Path, default=WRAPPER_USD)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    generate_wrapper(args.source_usd.resolve(), args.physics_usd.resolve(), args.output_usd.resolve())
    print(f"Wrote USDObject wrapper: {args.output_usd.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Validate SO-101 USD collision configuration."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    from pxr import Usd, UsdPhysics
except ImportError as exc:  # pragma: no cover - exercised only without USD installed.
    raise SystemExit(
        "Missing pxr USD bindings. Install usd-core or run with PYTHONPATH pointing "
        "to a usd-core target directory, e.g. PYTHONPATH=/tmp/codex-usd-core."
    ) from exc


DEFAULT_USD = Path("assert/SO101/so101_new_calib/so101_new_calib.usd")
ROOT_PATH = "/so101_new_calib"

REQUIRED_COLLISION_LINKS = (
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "moving_jaw_so101_v1_link",
)

NO_COLLISION_LINKS = ("gripper_frame_link",)
ALLOWED_APPROXIMATIONS = {"convexHull", "convexDecomposition"}


@dataclass(frozen=True)
class Collider:
    path: str
    approximation: str | None
    enabled: bool | None
    has_mesh_collision_api: bool
    has_mesh_descendant: bool


def _prim_range_for_collision_container(prim: Usd.Prim) -> Usd.PrimRange:
    """Traverse a collision container, expanding instance prototypes when needed."""

    if prim.IsInstance():
        prototype = prim.GetPrototype()
        if not prototype:
            raise ValueError(f"{prim.GetPath()} is an instance without a prototype")
        return Usd.PrimRange(prototype)
    return Usd.PrimRange(prim)


def _has_mesh_descendant(prim: Usd.Prim) -> bool:
    for child in Usd.PrimRange(prim):
        if child != prim and child.GetTypeName() == "Mesh":
            return True
    return False


def _collect_colliders(container: Usd.Prim) -> list[Collider]:
    colliders: list[Collider] = []
    for prim in _prim_range_for_collision_container(container):
        schemas = set(prim.GetAppliedSchemas())
        if "PhysicsCollisionAPI" not in schemas:
            continue

        collision_api = UsdPhysics.CollisionAPI(prim)
        enabled_attr = collision_api.GetCollisionEnabledAttr()
        enabled = enabled_attr.Get() if enabled_attr else None
        approximation_attr = prim.GetAttribute("physics:approximation")
        approximation = approximation_attr.Get() if approximation_attr else None

        colliders.append(
            Collider(
                path=str(prim.GetPath()),
                approximation=approximation,
                enabled=enabled,
                has_mesh_collision_api="PhysicsMeshCollisionAPI" in schemas,
                has_mesh_descendant=_has_mesh_descendant(prim),
            )
        )
    return colliders


def _validate_colliders(link_name: str, colliders: list[Collider]) -> list[str]:
    errors: list[str] = []
    if not colliders:
        return [f"{link_name}: missing collider"]

    for collider in colliders:
        if collider.enabled is False:
            errors.append(f"{link_name}: disabled collider {collider.path}")
        if not collider.has_mesh_collision_api:
            errors.append(f"{link_name}: missing PhysicsMeshCollisionAPI on {collider.path}")
        if collider.approximation not in ALLOWED_APPROXIMATIONS:
            errors.append(
                f"{link_name}: unsupported approximation {collider.approximation!r} "
                f"on {collider.path}"
            )
        if not collider.has_mesh_descendant:
            errors.append(f"{link_name}: collider {collider.path} has no Mesh descendant")
    return errors


def check_colliders(usd_path: Path, root_path: str, *, detail: bool) -> int:
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        print(f"FAIL: could not open USD stage: {usd_path}", file=sys.stderr)
        return 1

    errors: list[str] = []
    reports: list[tuple[str, list[Collider]]] = []

    for link_name in REQUIRED_COLLISION_LINKS:
        container_path = f"{root_path}/{link_name}/collisions"
        container = stage.GetPrimAtPath(container_path)
        if not container or not container.IsValid():
            errors.append(f"{link_name}: missing collisions container {container_path}")
            reports.append((link_name, []))
            continue

        colliders = _collect_colliders(container)
        reports.append((link_name, colliders))
        errors.extend(_validate_colliders(link_name, colliders))

    for link_name in NO_COLLISION_LINKS:
        container = stage.GetPrimAtPath(f"{root_path}/{link_name}/collisions")
        colliders = _collect_colliders(container) if container and container.IsValid() else []
        reports.append((link_name, colliders))
        if colliders:
            errors.append(f"{link_name}: expected no collider, found {len(colliders)}")

    print(f"USD: {usd_path}")
    for link_name, colliders in reports:
        if link_name in NO_COLLISION_LINKS and not colliders:
            print(f"{link_name}: 0 colliders (expected dummy EE frame)")
            continue

        approximations = sorted({str(c.approximation) for c in colliders})
        print(
            f"{link_name}: {len(colliders)} colliders, "
            f"approximations={','.join(approximations) if approximations else 'none'}"
        )
        if detail:
            for collider in colliders:
                print(f"  - {collider.path}")

    if errors:
        print("FAIL: collider validation errors:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print("PASS: SO-101 collision configuration is complete and uses convex mesh colliders")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD, help="SO-101 root USD path")
    parser.add_argument("--root", default=ROOT_PATH, help="SO-101 root prim path")
    parser.add_argument("--detail", action="store_true", help="Print individual collider prims")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return check_colliders(args.usd, args.root, detail=args.detail)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""M1 static check for SO-101 URDF physical parameters."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF_PATH = REPO_ROOT / "assert/SO101/so101_new_calib.urdf"
ASSET_ROOT = REPO_ROOT / "assert/SO101"

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
EXPECTED_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}
EXPECTED_CHAIN = (
    ("shoulder_pan", "base_link", "shoulder_link"),
    ("shoulder_lift", "shoulder_link", "upper_arm_link"),
    ("elbow_flex", "upper_arm_link", "lower_arm_link"),
    ("wrist_flex", "lower_arm_link", "wrist_link"),
    ("wrist_roll", "wrist_link", "gripper_link"),
    ("gripper_frame_joint", "gripper_link", "gripper_frame_link"),
)
DUMMY_LINKS = {"gripper_frame_link"}


def _load_urdf(path: Path) -> ET.Element:
    if not path.exists():
        raise FileNotFoundError(path)
    return ET.parse(path).getroot()


def _child_text(element: ET.Element, child: str, attr: str) -> str:
    node = element.find(child)
    if node is None or attr not in node.attrib:
        raise RuntimeError(f"missing {child} @{attr} under {element.tag} {element.attrib}")
    return node.attrib[attr]


def _collect(root: ET.Element) -> tuple[dict[str, ET.Element], dict[str, ET.Element]]:
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    return links, joints


def _check_joint_names_and_limits(joints: dict[str, ET.Element], tol: float) -> None:
    missing = sorted(set(EXPECTED_LIMITS) - set(joints))
    if missing:
        raise RuntimeError(f"missing expected joints: {missing}")

    for joint_name, (expected_lower, expected_upper) in EXPECTED_LIMITS.items():
        joint = joints[joint_name]
        limit = joint.find("limit")
        if limit is None:
            raise RuntimeError(f"missing limit for joint {joint_name}")
        lower = float(limit.attrib["lower"])
        upper = float(limit.attrib["upper"])
        effort = float(limit.attrib["effort"])
        velocity = float(limit.attrib["velocity"])
        if not math.isclose(lower, expected_lower, abs_tol=tol):
            raise RuntimeError(f"{joint_name} lower expected {expected_lower}, got {lower}")
        if not math.isclose(upper, expected_upper, abs_tol=tol):
            raise RuntimeError(f"{joint_name} upper expected {expected_upper}, got {upper}")
        if effort <= 0 or velocity <= 0:
            raise RuntimeError(f"{joint_name} effort/velocity must be positive")


def _check_chain(joints: dict[str, ET.Element]) -> None:
    for joint_name, parent, child in EXPECTED_CHAIN:
        joint = joints[joint_name]
        actual_parent = _child_text(joint, "parent", "link")
        actual_child = _child_text(joint, "child", "link")
        if (actual_parent, actual_child) != (parent, child):
            raise RuntimeError(
                f"{joint_name} expected {parent}->{child}, got {actual_parent}->{actual_child}"
            )


def _check_masses_and_inertias(links: dict[str, ET.Element]) -> tuple[float, list[str]]:
    total_mass = 0.0
    dummy_notes = []
    for link_name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            raise RuntimeError(f"missing inertial for link {link_name}")
        mass = float(_child_text(inertial, "mass", "value"))
        inertia = inertial.find("inertia")
        if inertia is None:
            raise RuntimeError(f"missing inertia for link {link_name}")
        diag = [float(inertia.attrib[key]) for key in ("ixx", "iyy", "izz")]
        total_mass += mass
        if link_name in DUMMY_LINKS:
            if mass > 1e-6 or any(value != 0.0 for value in diag):
                raise RuntimeError(f"dummy link {link_name} should stay tiny and inertialess")
            dummy_notes.append(link_name)
            continue
        if mass <= 0:
            raise RuntimeError(f"link {link_name} has non-positive mass {mass}")
        if any(value <= 0 for value in diag):
            raise RuntimeError(f"link {link_name} has invalid diagonal inertia {diag}")
    return total_mass, dummy_notes


def _check_meshes(root: ET.Element, asset_root: Path) -> tuple[int, int]:
    mesh_paths = [mesh.attrib["filename"] for mesh in root.findall(".//mesh")]
    missing = [path for path in mesh_paths if not (asset_root / path).exists()]
    if missing:
        raise RuntimeError(f"missing mesh files: {missing}")
    return len(mesh_paths), len(set(mesh_paths))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf-path", type=Path, default=DEFAULT_URDF_PATH)
    parser.add_argument("--asset-root", type=Path, default=ASSET_ROOT)
    parser.add_argument("--limit-tol", type=float, default=1e-5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = _load_urdf(args.urdf_path)
    links, joints = _collect(root)

    _check_joint_names_and_limits(joints, args.limit_tol)
    _check_chain(joints)
    total_mass, dummy_links = _check_masses_and_inertias(links)
    mesh_ref_count, unique_mesh_count = _check_meshes(root, args.asset_root)

    print(f"arm_joints: {', '.join(ARM_JOINTS)}")
    print("gripper_joint: gripper (revolute)")
    print("ee_link: gripper_frame_link")
    print("finger_link: moving_jaw_so101_v1_link")
    print(f"links: {len(links)}")
    print(f"joints: {len(joints)}")
    print(f"total_mass_kg: {total_mass:.9f}")
    print(f"mesh_refs: {mesh_ref_count} ({unique_mesh_count} unique)")
    if dummy_links:
        print(f"dummy_links_with_zero_inertia: {', '.join(dummy_links)}")
    print("PASS: SO-101 URDF physical parameters are internally consistent")


if __name__ == "__main__":
    main()

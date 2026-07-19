#!/usr/bin/env python3

import argparse
import copy
import hashlib
import math
import pathlib
import sys
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PACKAGE_ROOT / "models" / "x500_arm2x_control.urdf"
CONTROL_LINKS = ("base_link", "link_1", "link_2", "link_3", "link_4")
CONTROL_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4")
LUMPED_CHILDREN = (
    ("link_5", "joint_5"),
    ("gripper_left", "joint_gripper_left"),
    ("gripper_right", "joint_gripper_right"),
)
JAW_JOINTS = ("joint_gripper_left", "joint_gripper_right")
EE_LINK = "ee_link"
EE_JOINT = "ee_fixed_joint"

Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
IDENTITY: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
ZERO_MATRIX: Matrix3 = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
)


def _named_element(
    root: ET.Element, tag: str, name: str
) -> ET.Element:
    matches = [
        element
        for element in root.findall(tag)
        if element.get("name") == name
    ]
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one {tag} named {name!r}, found {len(matches)}."
        )
    return matches[0]


def _required_child(element: ET.Element, tag: str, owner: str) -> ET.Element:
    child = element.find(tag)
    if child is None:
        raise ValueError(f"{owner} is missing required <{tag}> data.")
    return child


def _vector_attribute(
    element: ET.Element, attribute: str, owner: str
) -> Vector3:
    raw_value = element.get(attribute)
    if raw_value is None:
        raise ValueError(f"{owner} is missing the {attribute!r} attribute.")
    try:
        values = tuple(float(item) for item in raw_value.split())
    except ValueError as exc:
        raise ValueError(
            f"{owner} has a non-numeric {attribute!r} attribute."
        ) from exc
    if len(values) != 3:
        raise ValueError(f"{owner} {attribute!r} must contain three values.")
    return values


def _add_vector(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))


def _subtract_vector(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] - right[index] for index in range(3))


def _scale_vector(scale: float, vector: Vector3) -> Vector3:
    return tuple(scale * value for value in vector)


def _add_matrix(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(left[row][column] + right[row][column] for column in range(3))
        for row in range(3)
    )


def _scale_matrix(scale: float, matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(scale * matrix[row][column] for column in range(3))
        for row in range(3)
    )


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(matrix[column][row] for column in range(3))
        for row in range(3)
    )


def _multiply_matrix(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _multiply_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][index] * vector[index] for index in range(3))
        for row in range(3)
    )


def _rotation_from_rpy(rpy: Vector3) -> Matrix3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _origin_pose(element: ET.Element, owner: str) -> tuple[Vector3, Matrix3]:
    origin = _required_child(element, "origin", owner)
    xyz = _vector_attribute(origin, "xyz", f"origin for {owner}")
    rpy = _vector_attribute(origin, "rpy", f"origin for {owner}")
    return xyz, _rotation_from_rpy(rpy)


def _inertia_matrix(inertia: ET.Element, owner: str) -> Matrix3:
    def value(name: str) -> float:
        raw_value = inertia.get(name)
        if raw_value is None:
            raise ValueError(f"{owner} is missing inertia {name!r}.")
        try:
            return float(raw_value)
        except ValueError as exc:
            raise ValueError(
                f"{owner} inertia {name!r} is not numeric."
            ) from exc

    ixx, ixy, ixz = value("ixx"), value("ixy"), value("ixz")
    iyy, iyz, izz = value("iyy"), value("iyz"), value("izz")
    return (
        (ixx, ixy, ixz),
        (ixy, iyy, iyz),
        (ixz, iyz, izz),
    )


def _format_number(value: float) -> str:
    if abs(value) < 1.0e-15:
        return "0"
    return format(value, ".12g")


def _format_vector(vector: Vector3) -> str:
    return " ".join(_format_number(value) for value in vector)


def _component_in_link4(
    root: ET.Element, link_name: str, joint_name: str | None
) -> tuple[float, Vector3, Matrix3]:
    link = _named_element(root, "link", link_name)
    inertial = _required_child(link, "inertial", f"link {link_name!r}")

    mass_element = _required_child(
        inertial, "mass", f"inertial for link {link_name!r}"
    )
    raw_mass = mass_element.get("value")
    if raw_mass is None:
        raise ValueError(f"Link {link_name!r} mass is missing its value.")
    try:
        mass = float(raw_mass)
    except ValueError as exc:
        raise ValueError(f"Link {link_name!r} mass is not numeric.") from exc

    if joint_name is None:
        link_translation, link_rotation = (0.0, 0.0, 0.0), IDENTITY
    else:
        joint = _named_element(root, "joint", joint_name)
        link_translation, link_rotation = _origin_pose(
            joint, f"joint {joint_name!r}"
        )

    inertial_translation, inertial_rotation = _origin_pose(
        inertial, f"inertial for link {link_name!r}"
    )
    center = _add_vector(
        link_translation,
        _multiply_vector(link_rotation, inertial_translation),
    )
    rotation = _multiply_matrix(link_rotation, inertial_rotation)
    source_tensor = _inertia_matrix(
        _required_child(
            inertial, "inertia", f"inertial for link {link_name!r}"
        ),
        f"link {link_name!r}",
    )
    rotated_tensor = _multiply_matrix(
        _multiply_matrix(rotation, source_tensor),
        _transpose(rotation),
    )
    return mass, center, rotated_tensor


def _parallel_axis(displacement: Vector3) -> Matrix3:
    squared_norm = sum(value * value for value in displacement)
    return tuple(
        tuple(
            (squared_norm if row == column else 0.0)
            - displacement[row] * displacement[column]
            for column in range(3)
        )
        for row in range(3)
    )


def _lumped_link4_inertial(root: ET.Element) -> ET.Element:
    # Omitted gripper joints are rigidly folded into link_4 at q=0.
    components = [
        _component_in_link4(root, CONTROL_LINKS[-1], None),
        *(
            _component_in_link4(root, link_name, joint_name)
            for link_name, joint_name in LUMPED_CHILDREN
        ),
    ]
    total_mass = sum(mass for mass, _, _ in components)
    weighted_center = (0.0, 0.0, 0.0)
    for mass, center, _ in components:
        weighted_center = _add_vector(
            weighted_center, _scale_vector(mass, center)
        )
    center = _scale_vector(1.0 / total_mass, weighted_center)

    tensor = ZERO_MATRIX
    for mass, component_center, component_tensor in components:
        displacement = _subtract_vector(component_center, center)
        tensor = _add_matrix(
            tensor,
            _add_matrix(
                component_tensor,
                _scale_matrix(mass, _parallel_axis(displacement)),
            ),
        )
    tensor = tuple(
        tuple(
            0.5 * (tensor[row][column] + tensor[column][row])
            for column in range(3)
        )
        for row in range(3)
    )

    inertial = ET.Element("inertial")
    ET.SubElement(
        inertial,
        "origin",
        {"xyz": _format_vector(center), "rpy": "0 0 0"},
    )
    ET.SubElement(inertial, "mass", {"value": _format_number(total_mass)})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": _format_number(tensor[0][0]),
            "ixy": _format_number(tensor[0][1]),
            "ixz": _format_number(tensor[0][2]),
            "iyy": _format_number(tensor[1][1]),
            "iyz": _format_number(tensor[1][2]),
            "izz": _format_number(tensor[2][2]),
        },
    )
    return inertial


def _jaw_center(root: ET.Element) -> Vector3:
    positions = [
        _origin_pose(
            _named_element(root, "joint", joint_name),
            f"joint {joint_name!r}",
        )[0]
        for joint_name in JAW_JOINTS
    ]
    return _scale_vector(0.5, _add_vector(positions[0], positions[1]))


def _validate_source(root: ET.Element) -> None:
    if root.tag != "robot":
        raise ValueError(f"Expected <robot> root, found <{root.tag}>.")

    lumped_link_names = tuple(link_name for link_name, _ in LUMPED_CHILDREN)
    for link_name in CONTROL_LINKS + lumped_link_names:
        link = _named_element(root, "link", link_name)
        inertial = _required_child(link, "inertial", f"link {link_name!r}")
        for tag in ("origin", "mass", "inertia"):
            _required_child(inertial, tag, f"inertial for link {link_name!r}")

    expected_parent = CONTROL_LINKS[0]
    for joint_name, expected_child in zip(CONTROL_JOINTS, CONTROL_LINKS[1:]):
        joint = _named_element(root, "joint", joint_name)
        if joint.get("type") != "revolute":
            raise ValueError(f"Joint {joint_name!r} must be revolute.")

        parent = _required_child(
            joint, "parent", f"joint {joint_name!r}"
        ).get("link")
        child = _required_child(
            joint, "child", f"joint {joint_name!r}"
        ).get("link")
        if (parent, child) != (expected_parent, expected_child):
            raise ValueError(
                f"Joint {joint_name!r} must connect {expected_parent!r} to "
                f"{expected_child!r}, found {parent!r} to {child!r}."
            )
        for tag in ("origin", "axis", "limit"):
            _required_child(joint, tag, f"joint {joint_name!r}")
        expected_parent = expected_child

    for child_link, joint_name in LUMPED_CHILDREN:
        joint = _named_element(root, "joint", joint_name)
        parent = _required_child(
            joint, "parent", f"joint {joint_name!r}"
        ).get("link")
        child = _required_child(
            joint, "child", f"joint {joint_name!r}"
        ).get("link")
        if (parent, child) != (CONTROL_LINKS[-1], child_link):
            raise ValueError(
                f"Joint {joint_name!r} must connect {CONTROL_LINKS[-1]!r} "
                f"to {child_link!r}, found {parent!r} to {child!r}."
            )
        _required_child(joint, "origin", f"joint {joint_name!r}")


def build_control_model(
    source_root: ET.Element, source_sha256: str
) -> ET.Element:
    _validate_source(source_root)

    source_name = source_root.get("name")
    if not source_name:
        raise ValueError("Source robot is missing its name.")
    control_root = ET.Element("robot", {"name": f"{source_name}_control"})
    control_root.append(
        ET.Comment(f" Source URDF SHA256: {source_sha256} ")
    )

    base_source = _named_element(source_root, "link", CONTROL_LINKS[0])
    base = ET.SubElement(control_root, "link", {"name": CONTROL_LINKS[0]})
    base.append(
        copy.deepcopy(_required_child(base_source, "inertial", "base_link"))
    )

    for link_name, joint_name in zip(CONTROL_LINKS[1:], CONTROL_JOINTS):
        link_source = _named_element(source_root, "link", link_name)
        link = ET.SubElement(control_root, "link", {"name": link_name})
        if link_name == CONTROL_LINKS[-1]:
            link.append(_lumped_link4_inertial(source_root))
        else:
            link.append(
                copy.deepcopy(
                    _required_child(link_source, "inertial", link_name)
                )
            )
        source_joint = _named_element(source_root, "joint", joint_name)
        control_root.append(copy.deepcopy(source_joint))

    ET.SubElement(control_root, "link", {"name": EE_LINK})
    fixed_joint = ET.SubElement(
        control_root, "joint", {"name": EE_JOINT, "type": "fixed"}
    )
    ET.SubElement(
        fixed_joint,
        "origin",
        {"xyz": _format_vector(_jaw_center(source_root)), "rpy": "0 0 0"},
    )
    ET.SubElement(fixed_joint, "parent", {"link": CONTROL_LINKS[-1]})
    ET.SubElement(fixed_joint, "child", {"link": EE_LINK})

    return control_root


def generate_control_urdf(
    source_path: pathlib.Path, output_path: pathlib.Path
) -> pathlib.Path:
    source_bytes = source_path.read_bytes()
    source_sha256 = hashlib.sha256(source_bytes).hexdigest()
    source_root = ET.fromstring(source_bytes)
    control_root = build_control_model(source_root, source_sha256)
    ET.indent(control_root, space="  ")

    serialized = ET.tostring(
        control_root,
        encoding="utf-8",
        xml_declaration=True,
        short_empty_elements=True,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(serialized + b"\n")
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the reduced mesh-free x500_arm2x control URDF."
    )
    parser.add_argument(
        "source",
        type=pathlib.Path,
        help="Path to the full x500_arm2x URDF",
    )
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=DEFAULT_OUTPUT,
        help=f"Output path (default: {DEFAULT_OUTPUT})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        generated = generate_control_urdf(args.source, args.output)
    except (ET.ParseError, OSError, ValueError) as exc:
        print(f"Control URDF generation failed: {exc}", file=sys.stderr)
        return 2

    print(generated)
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Read-only capture of the current South/West top-rope pose.

Run in Blender's Text Editor AFTER manually placing the turnbuckle and point
empties. This does not run another rig script, change a control, or save the
.blend. It writes one timestamped JSON beside the .blend in SW_Rope_Diagnostics.
Run it again at another turnbuckle pose to provide a second relationship sample.
"""

import bpy
import json
import re
from datetime import datetime
from pathlib import Path
from mathutils import Vector


MASTER_NAME = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
CURVES = {"S": "ROPE_SOUTH_TOP_CURVE", "W": "ROPE_WEST_TOP_CURVE"}
POINT_NAME = re.compile(r"^(BASE|CTRL|OFFSET)_Rope_TR_[SW]_Curve_P\d\d$")
ANCHOR_NAME = re.compile(r"^CTRL_Rope Point_TR_[SW]_")


def numbers(values):
    return [float(value) for value in values]


def matrix_values(matrix):
    return [numbers(row) for row in matrix]


def json_value(value):
    if isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "items"):
        return {str(key): json_value(item) for key, item in value.items()}
    if hasattr(value, "to_list"):
        return [json_value(item) for item in value.to_list()]
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    return str(value)


def drivers(obj):
    animation = obj.animation_data
    if not animation:
        return []
    result = []
    for fcurve in animation.drivers:
        variables = []
        for variable in fcurve.driver.variables:
            variables.append({
                "name": variable.name,
                "type": variable.type,
                "targets": [
                    {"id": target.id.name if target.id else None,
                     "data_path": target.data_path,
                     "transform_type": target.transform_type,
                     "transform_space": target.transform_space}
                    for target in variable.targets
                ],
            })
        result.append({"data_path": fcurve.data_path,
                       "array_index": fcurve.array_index,
                       "expression": fcurve.driver.expression,
                       "variables": variables})
    return result


def object_record(name):
    obj = bpy.data.objects.get(name)
    if obj is None:
        return {"name": name, "missing": True}
    world_location, world_rotation, world_scale = obj.matrix_world.decompose()
    return {
        "name": obj.name,
        "type": obj.type,
        "parent": obj.parent.name if obj.parent else None,
        "parent_type": obj.parent_type,
        "parent_bone": obj.parent_bone,
        "matrix_parent_inverse": matrix_values(obj.matrix_parent_inverse),
        "location": numbers(obj.location),
        "rotation_mode": obj.rotation_mode,
        "rotation_euler_rad": numbers(obj.rotation_euler),
        "rotation_euler_deg": numbers([angle * 180.0 / 3.141592653589793
                                        for angle in obj.rotation_euler]),
        "rotation_quaternion": numbers(obj.rotation_quaternion),
        "scale": numbers(obj.scale),
        "delta_location": numbers(obj.delta_location),
        "delta_rotation_euler_rad": numbers(obj.delta_rotation_euler),
        "delta_scale": numbers(obj.delta_scale),
        "matrix_world": matrix_values(obj.matrix_world),
        "world_location": numbers(world_location),
        "world_rotation_quaternion": numbers(world_rotation),
        "world_scale": numbers(world_scale),
        "custom_properties": {key: json_value(obj[key])
                              for key in obj.keys() if key != "_RNA_UI"},
        "constraints": [
            {"name": constraint.name, "type": constraint.type,
             "target": constraint.target.name if getattr(constraint, "target", None) else None,
             "influence": float(constraint.influence), "mute": bool(constraint.mute)}
            for constraint in obj.constraints
        ],
        "drivers": drivers(obj),
    }


def curve_record(name, depsgraph):
    curve = bpy.data.objects.get(name)
    if curve is None or curve.type != "CURVE":
        return {"name": name, "missing_or_wrong_type": True}
    splines = []
    for spline in curve.data.splines:
        entry = {"type": spline.type, "cyclic": bool(spline.use_cyclic_u)}
        if spline.type == "BEZIER":
            entry["points"] = [
                {"index": index,
                 "co_local": numbers(point.co),
                 "co_world_before_hooks": numbers(curve.matrix_world @ point.co),
                 "handle_left_local": numbers(point.handle_left),
                 "handle_left_world_before_hooks": numbers(curve.matrix_world @ point.handle_left),
                 "handle_left_type": point.handle_left_type,
                 "handle_right_local": numbers(point.handle_right),
                 "handle_right_world_before_hooks": numbers(curve.matrix_world @ point.handle_right),
                 "handle_right_type": point.handle_right_type,
                 "tilt_rad": float(point.tilt), "radius": float(point.radius)}
                for index, point in enumerate(spline.bezier_points)
            ]
        splines.append(entry)
    result = {
        "name": name,
        "matrix_world": matrix_values(curve.matrix_world),
        "splines_before_hooks": splines,
        "modifiers": [
            {"name": modifier.name, "type": modifier.type,
             "object": modifier.object.name if modifier.type == "HOOK" and modifier.object else None,
             "vertex_indices": list(modifier.vertex_indices) if modifier.type == "HOOK" else None,
             "strength": float(modifier.strength) if modifier.type == "HOOK" else None}
            for modifier in curve.modifiers
        ],
    }
    evaluated = curve.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        result["evaluated_centerline_world"] = [
            numbers(evaluated.matrix_world @ vertex.co) for vertex in mesh.vertices
        ]
    finally:
        evaluated.to_mesh_clear()
    return result


def main():
    now = datetime.now().astimezone()
    project_file = Path(bpy.data.filepath) if bpy.data.filepath else None
    if project_file:
        destination = project_file.parent / "SW_Rope_Diagnostics"
    else:
        destination = Path.home() / "Downloads" / "SW_Rope_Diagnostics"
    destination.mkdir(parents=True, exist_ok=True)
    output = destination / ("SW_Top_Rope_Pose_" + now.strftime("%Y%m%d_%H%M%S_%f") + ".json")
    names = [MASTER_NAME]
    for side in "SW":
        for index in range(10):
            for level in ("BASE", "CTRL", "OFFSET"):
                names.append(f"{level}_Rope_TR_{side}_Curve_P{index:02d}")
    names.extend(sorted(obj.name for obj in bpy.data.objects
                        if ANCHOR_NAME.match(obj.name)))
    names = list(dict.fromkeys(names))
    depsgraph = bpy.context.evaluated_depsgraph_get()
    objects = {name: object_record(name) for name in names}
    west = objects.get("BASE_Rope_TR_W_Curve_P09")
    south = objects.get("BASE_Rope_TR_S_Curve_P00")
    junction_gap = None
    if west and south and not west.get("missing") and not south.get("missing"):
        junction_gap = (Vector(west["world_location"]) -
                        Vector(south["world_location"])).length
    report = {
        "description": "Current artist-adjusted SW turnbuckle and top-rope point pose; no rig edits",
        "captured_at": now.isoformat(),
        "blender_version": bpy.app.version_string,
        "blend_file": str(project_file) if project_file else None,
        "scene": bpy.context.scene.name,
        "frame": bpy.context.scene.frame_current,
        "active_object": bpy.context.view_layer.objects.active.name
                         if bpy.context.view_layer.objects.active else None,
        "object_records": objects,
        "curves": {side: curve_record(name, depsgraph)
                   for side, name in CURVES.items()},
        "junction_base_gap_m": junction_gap,
        "missing_point_controls": [name for name, record in objects.items()
                                   if POINT_NAME.match(name) and record.get("missing")],
    }
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print("SW_ROPE_POSE_EXPORTED::" + str(output))
    print("JUNCTION_BASE_GAP_M::" + str(junction_gap))
    print("No objects, curves, bones, or .blend files were changed.")


if __name__ == "__main__":
    main()

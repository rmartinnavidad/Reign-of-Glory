"""Read-only South/West rope seam capture at the CURRENT Blender pose.

Open this file in Blender's Text Editor and press Run Script once. Do not move
the turnbuckle first. It does not install/re-run a rig, move anything, or save
the .blend. It exports a timestamped JSON to SW_Rope_Diagnostics beside the
open .blend and prints the full filename in Blender's console.
"""

import bpy
import json
import math
from datetime import datetime
from pathlib import Path
from mathutils import Vector


MASTER = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
ROPE = "MESH_Ring Ropes"
ARMATURE = "BONES_ROPES"
CURVES = ("ROPE_SOUTH_TOP_CURVE", "ROPE_WEST_TOP_CURVE")
EYE_CONTROLS = ("OFFSET_Rope_TR_S_Curve_P00", "OFFSET_Rope_TR_W_Curve_P09")
NEAR_EYE_M = 1.25


def xyz(vector):
    return [round(float(v), 7) for v in vector]


def object_pose(obj):
    if obj is None:
        return None
    return {
        "name": obj.name,
        "type": obj.type,
        "parent": obj.parent.name if obj.parent else None,
        "location_local_m": xyz(obj.location),
        "location_world_m": xyz(obj.matrix_world.translation),
        "rotation_euler_deg": xyz(math.degrees(v) for v in obj.rotation_euler),
        "scale": xyz(obj.scale),
        "delta_location_m": xyz(obj.delta_location),
        "custom_properties": {
            key: list(value) if hasattr(value, "to_list") else value
            for key, value in obj.items()
            if key.startswith(("CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W | Off",
                               "CTRL_Rope_TR_", "Cable ", "Broken Rope"))
            and isinstance(value, (int, float, str, bool, list, tuple))
        },
    }


def weights(obj, vertex):
    result = []
    for item in vertex.groups:
        if item.group < len(obj.vertex_groups):
            result.append({"bone_or_group": obj.vertex_groups[item.group].name,
                           "weight": round(float(item.weight), 5)})
    return sorted(result, key=lambda row: -row["weight"])


def capture_curve(obj, depsgraph):
    if obj is None or obj.type != "CURVE":
        return {"missing": True}
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        samples = [evaluated.matrix_world @ vertex.co for vertex in mesh.vertices]
    finally:
        evaluated.to_mesh_clear()
    bends = []
    for i in range(1, len(samples)-1):
        a, b = samples[i]-samples[i-1], samples[i+1]-samples[i]
        if a.length > 1e-7 and b.length > 1e-7:
            bends.append({"sample": i, "angle_deg": round(math.degrees(a.angle(b)), 4),
                          "world_m": xyz(samples[i])})
    return {
        "samples_world_m": [xyz(point) for point in samples],
        "worst_sample_bends": sorted(bends, key=lambda row: -row["angle_deg"])[:20],
        "spline_point_counts": [len(spline.bezier_points) if spline.type == "BEZIER"
                                else len(spline.points) for spline in obj.data.splines],
        "hook_targets": [
            {"modifier": modifier.name,
             "object": modifier.object.name if modifier.object else None,
             "vertex_indices": list(modifier.vertex_indices)}
            for modifier in obj.modifiers if modifier.type == "HOOK"
        ],
    }


def main():
    master = bpy.data.objects.get(MASTER)
    rope = bpy.data.objects.get(ROPE)
    if master is None or rope is None or rope.type != "MESH":
        raise RuntimeError("Turnbuckle master or original rope mesh missing; nothing changed")

    depsgraph = bpy.context.evaluated_depsgraph_get()
    eye_objects = [bpy.data.objects.get(name) for name in EYE_CONTROLS]
    eye = (eye_objects[0].matrix_world.translation.copy() if eye_objects[0]
           else master.matrix_world.translation.copy())
    original = rope.data
    evaluated = rope.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        same_topology = (len(original.vertices) == len(mesh.vertices)
                         and len(original.edges) == len(mesh.edges)
                         and all(tuple(a.vertices) == tuple(b.vertices)
                                 for a, b in zip(original.edges, mesh.edges)))
        edge_rows = []
        if same_topology:
            rest = [rope.matrix_world @ vertex.co for vertex in original.vertices]
            current = [evaluated.matrix_world @ vertex.co for vertex in mesh.vertices]
            for edge in original.edges:
                a, b = edge.vertices
                rest_length = (rest[a]-rest[b]).length
                now_length = (current[a]-current[b]).length
                distance = min((current[a]-eye).length, (current[b]-eye).length)
                edge_rows.append({
                    "edge_index": edge.index,
                    "vertex_indices": [a, b],
                    "rest_length_m": round(rest_length, 7),
                    "evaluated_length_m": round(now_length, 7),
                    "stretch_ratio": round(now_length/max(rest_length, 1e-8), 5),
                    "distance_to_sw_eye_m": round(distance, 5),
                    "rest_endpoints_world_m": [xyz(rest[a]), xyz(rest[b])],
                    "evaluated_endpoints_world_m": [xyz(current[a]), xyz(current[b])],
                    "weights": [weights(rope, original.vertices[a]),
                                weights(rope, original.vertices[b])],
                })
        near = [row for row in edge_rows if row["distance_to_sw_eye_m"] <= NEAR_EYE_M]
        near.sort(key=lambda row: (-row["stretch_ratio"], -row["evaluated_length_m"]))
        global_worst = sorted(edge_rows, key=lambda row: -row["stretch_ratio"])[:30]
        # Keep all suspicious local edges, plus the worst 100 even if below
        # threshold, so a clean pose still gives a useful comparison.
        local_edges = [row for row in near if row["stretch_ratio"] >= 1.5
                       or row["evaluated_length_m"] >= 0.08]
        selected = {row["edge_index"]: row for row in near[:100] + local_edges}
        armature = bpy.data.objects.get(ARMATURE)
        bones = {}
        if armature and armature.type == "ARMATURE":
            implicated = {
                group["bone_or_group"]
                for row in selected.values() for endpoint in row["weights"]
                for group in endpoint
            }
            for name in sorted(implicated):
                bone = armature.pose.bones.get(name)
                if bone is not None:
                    bones[name] = {
                        "head_world_m": xyz(armature.matrix_world @ bone.head),
                        "tail_world_m": xyz(armature.matrix_world @ bone.tail),
                        "head_to_eye_m": round((armature.matrix_world @ bone.head-eye).length, 6),
                        "tail_to_eye_m": round((armature.matrix_world @ bone.tail-eye).length, 6),
                        "constraints": [
                            {"name": c.name, "type": c.type,
                             "target": c.target.name if getattr(c, "target", None) else None,
                             "chain_count": c.chain_count if c.type == "SPLINE_IK" else None}
                            for c in bone.constraints
                        ],
                    }
        curves = {name: capture_curve(bpy.data.objects.get(name), depsgraph)
                  for name in CURVES}
        south = curves[CURVES[0]].get("samples_world_m", [])
        west = curves[CURVES[1]].get("samples_world_m", [])
        curve_gap = ((Vector(south[0])-Vector(west[-1])).length
                     if south and west else None)
        join_angle = None
        if len(south) >= 2 and len(west) >= 2:
            incoming = Vector(west[-1])-Vector(west[-2])
            outgoing = Vector(south[1])-Vector(south[0])
            if incoming.length > 1e-7 and outgoing.length > 1e-7:
                join_angle = math.degrees(incoming.angle(outgoing))
        pose = {
            "master": object_pose(master),
            "eye_controls": {name: object_pose(obj)
                             for name, obj in zip(EYE_CONTROLS, eye_objects)},
            "point_controls": {
                name: object_pose(bpy.data.objects.get(name))
                for side in "SW" for index in range(10)
                for name in (f"BASE_Rope_TR_{side}_Curve_P{index:02d}",
                             f"CTRL_Rope_TR_{side}_Curve_P{index:02d}",
                             f"OFFSET_Rope_TR_{side}_Curve_P{index:02d}")
            },
        }
        report = {
            "description": "Read-only evaluated SW rope seam diagnosis at current Blender pose",
            "captured_at": datetime.now().astimezone().isoformat(),
            "blend_file": bpy.data.filepath,
            "frame": bpy.context.scene.frame_current,
            "blender_version": bpy.app.version_string,
            "pose": pose,
            "sw_eye_world_m": xyz(eye),
            "curve_end_gap_m": round(curve_gap, 7) if curve_gap is not None else None,
            "curve_join_angle_deg": round(join_angle, 4) if join_angle is not None else None,
            "curves": curves,
            "rope_modifiers": [{"name": m.name, "type": m.type,
                                "show_viewport": bool(m.show_viewport)} for m in rope.modifiers],
            "rope_original_vertex_count": len(original.vertices),
            "rope_evaluated_vertex_count": len(mesh.vertices),
            "rope_same_topology": same_topology,
            "near_eye_radius_m": NEAR_EYE_M,
            "near_eye_edge_count": len(near),
            "near_eye_worst_edges": list(selected.values()),
            "global_worst_edges": global_worst,
            "implicated_pose_bones": bones,
            "warning": None if same_topology else
                       "A modifier changed mesh topology, so rest/evaluated edge indices cannot safely be compared.",
        }
    finally:
        evaluated.to_mesh_clear()

    now = datetime.now().astimezone()
    folder = ((Path(bpy.data.filepath).parent if bpy.data.filepath else
               Path.home() / "Downloads") / "SW_Rope_Diagnostics")
    folder.mkdir(parents=True, exist_ok=True)
    output = folder / ("SW_Rope_Seams_" + now.strftime("%Y%m%d_%H%M%S_%f") + ".json")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print("SW_ROPE_SEAMS_EXPORTED::" + str(output))
    print("SW_ROPE_NEAR_EYE_WORST_RATIO::" +
          str(near[0]["stretch_ratio"] if near else None))
    print("SW_ROPE_CURVE_GAP_M::" + str(report["curve_end_gap_m"]))
    print("No Blender objects, rig data, or .blend files were changed.")


if __name__ == "__main__":
    main()

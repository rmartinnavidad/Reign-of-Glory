"""Build rounded, flexible-looking SW top ropes from the clean four-point scene.

Run once in Blender 5.0 on the clean four-point wrestling ring scene.
The script changes only the open scene in memory. It does not create or save
any .blend file. Save manually only if you like the result.
It requires the original v018 script in Downloads/Rope_Rig_Versions.

This intentionally does NOT import v029-v032: those scripts replace the v018
Hook chain and were observed to break live curve deformation. The ten curve
points and v018 Hook motion are preserved. The existing SW top mesh sections
are non-destructively masked; smooth curve-generated tubes become the visible
South/West top ropes. No extra artist point empties are created.
"""

import bpy
import math
import runpy
from pathlib import Path
from mathutils import Vector

V18 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\Add_Turnbuckle_Unit_Hybrid_v018_CanvasSafe_Cable.py")
MASTER = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
CURVES = ("ROPE_SOUTH_TOP_CURVE", "ROPE_WEST_TOP_CURVE")
FLOOR_NAME = "LSO Mat Collision - Actual Spline IK"
GN_NAME = "LSO Canvas Safety • Preserve Hooks"


def stop(message):
    raise RuntimeError("SW top v035 stopped safely: " + message)


def evaluated_points(name):
    obj = bpy.data.objects[name]
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    try:
        return [evaluated.matrix_world @ vertex.co for vertex in mesh.vertices]
    finally:
        evaluated.to_mesh_clear()


def max_turn(points):
    result = 0.0
    for a, b, c in zip(points, points[1:], points[2:]):
        incoming, outgoing = b - a, c - b
        if incoming.length > 1e-8 and outgoing.length > 1e-8:
            result = max(result, math.degrees(incoming.angle(outgoing)))
    return result


def rope_mesh_minimums():
    obj = bpy.data.objects["MESH_Ring Ropes"]
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    try:
        result = {}
        for side, ids in (("S", range(160, 171)), ("W", range(88, 99))):
            groups = {obj.vertex_groups[f"Bone.{n:03d}"].index for n in ids}
            indices = [v.index for v in obj.data.vertices
                       if any(g.group in groups and g.weight > 1e-6 for g in v.groups)]
            if not indices:
                stop(f"{side} rope mesh has no mapped vertices")
            result[side] = min((evaluated.matrix_world @ mesh.vertices[i].co).z
                               for i in indices)
        return result
    finally:
        evaluated.to_mesh_clear()


def soften_canvas_contact(curve):
    modifier = curve.modifiers.get(GN_NAME)
    if modifier is None or modifier.type != "NODES":
        stop(f"{curve.name} lacks the v018 canvas modifier")
    group = modifier.node_group
    # The v018 group resamples the Hook-deformed curve and clamps Z to the mat.
    # Smooth ONLY this post-Hook floor correction; no Hook or rope mesh changes.
    nodes, links = group.nodes, group.links
    combine = next(node for node in nodes if node.bl_idname == "ShaderNodeCombineXYZ")
    set_position = next(node for node in nodes if node.bl_idname == "GeometryNodeSetPosition")
    for link in list(set_position.inputs["Position"].links):
        links.remove(link)
    blur = nodes.new("GeometryNodeBlurAttribute")
    blur.name = "LSO v035 Smooth Floor Contact"
    blur.data_type = "FLOAT_VECTOR"
    blur.inputs["Iterations"].default_value = 8
    links.new(combine.outputs["Vector"], blur.inputs["Value"])
    parameter = nodes.new("GeometryNodeSplineParameter")
    multiply_pi = nodes.new("ShaderNodeMath")
    multiply_pi.operation = "MULTIPLY"
    multiply_pi.inputs[1].default_value = math.pi
    sine = nodes.new("ShaderNodeMath")
    sine.operation = "SINE"
    envelope = nodes.new("ShaderNodeMath")
    envelope.operation = "MULTIPLY"
    links.new(parameter.outputs["Factor"], multiply_pi.inputs[0])
    links.new(multiply_pi.outputs[0], sine.inputs[0])
    links.new(sine.outputs[0], envelope.inputs[0])
    links.new(sine.outputs[0], envelope.inputs[1])
    mix = nodes.new("ShaderNodeMix")
    mix.data_type = "VECTOR"
    links.new(envelope.outputs[0], mix.inputs["Factor"])
    links.new(combine.outputs["Vector"], mix.inputs["A"])
    links.new(blur.outputs["Value"], mix.inputs["B"])
    links.new(mix.outputs["Result"], set_position.inputs["Position"])
    group.update_tag()


def make_curve_visible_rope(curve, rope_material):
    """Use the post-Hook, floor-corrected path for actual visible geometry."""
    modifier = curve.modifiers.get(GN_NAME)
    group = modifier.node_group
    nodes, links = group.nodes, group.links
    resample = next(node for node in nodes if node.bl_idname == "GeometryNodeResampleCurve")
    resample.inputs["Count"].default_value = 256
    blur = nodes.get("LSO v035 Smooth Floor Contact")
    blur.inputs["Iterations"].default_value = 96
    set_position = next(node for node in nodes if node.bl_idname == "GeometryNodeSetPosition")
    output = next(node for node in nodes if node.bl_idname == "NodeGroupOutput")
    for link in list(output.inputs["Geometry"].links):
        links.remove(link)
    profile = nodes.new("GeometryNodeCurvePrimitiveCircle")
    profile.name = "LSO Round Rope Profile"
    profile.inputs["Radius"].default_value = .018
    profile.inputs["Resolution"].default_value = 16
    tube = nodes.new("GeometryNodeCurveToMesh")
    tube.name = "LSO Flexible Visible Rope"
    tube.inputs["Fill Caps"].default_value = True
    links.new(set_position.outputs["Geometry"], tube.inputs["Curve"])
    links.new(profile.outputs["Curve"], tube.inputs["Profile Curve"])
    links.new(tube.outputs["Mesh"], output.inputs["Geometry"])
    curve.data.bevel_depth = 0.0
    curve.hide_set(False)
    curve.hide_render = False
    if rope_material is not None and rope_material not in list(curve.data.materials):
        curve.data.materials.append(rope_material)
    group.update_tag()


def mask_old_top_rope_mesh():
    """Keep all other ropes; hide only the old SW top sections at modifier end."""
    rope = bpy.data.objects["MESH_Ring Ropes"]
    group_ids = {rope.vertex_groups[f"Bone.{number:03d}"].index
                 for begin, end in ((160, 171), (88, 99))
                 for number in range(begin, end)}
    keep_indices = [vertex.index for vertex in rope.data.vertices
                    if not any(weight.group in group_ids and weight.weight > 1e-6
                               for weight in vertex.groups)]
    if len(keep_indices) != 12700 or len(rope.data.vertices) != 15250:
        stop("rope mesh grouping differs from verified capture; no masking")
    keep = rope.vertex_groups.new(name="LSO Keep Other Ropes - SW Top Rounded")
    keep.add(keep_indices, 1.0, "REPLACE")
    mask = rope.modifiers.new("LSO Replace Only Old SW Top Mesh", "MASK")
    mask.mode = "VERTEX_GROUP"
    mask.vertex_group = keep.name
    return len(keep_indices)


def verify_visible_rope(name, floor):
    vertices = evaluated_points(name)
    if len(vertices) != 4096:
        stop(f"{name} did not become the expected 256-ring tube")
    centers = [sum(vertices[i:i + 16], Vector()) / 16
               for i in range(0, len(vertices), 16)]
    minimum = min(p.z for p in vertices)
    bend = max_turn(centers)
    print(f"V035 VISIBLE {name}: min_z={minimum:.6f}, worst_centerline_turn={bend:.2f} deg")
    if minimum < floor - .04 or bend > 6.0:
        stop(f"{name} visible rope intersects canvas or remains sharply bent")


def test_master_motion(master, name):
    before = evaluated_points(name)
    old = master.location.copy()
    master.location.x += .10
    bpy.context.view_layer.update()
    after = evaluated_points(name)
    master.location = old
    bpy.context.view_layer.update()
    if len(before) != len(after):
        stop(f"{name} changed vertex count during motion test")
    movement = max((a - b).length for a, b in zip(before, after))
    print(f"V035 MOTION {name}: {movement:.6f} m")
    if movement < .02:
        stop(f"{name} did not follow the turnbuckle")


def build():
    if not V18.is_file():
        stop(f"missing v018 source: {V18}")
    if bpy.data.objects.get(MASTER):
        stop("turnbuckle master already exists; open the clean four-point source Blend")
    for name in CURVES:
        obj = bpy.data.objects.get(name)
        if obj is None or obj.type != "CURVE" or len(obj.data.splines[0].bezier_points) != 4:
            stop(f"{name} is not the clean four-point curve")

    v18 = runpy.run_path(str(V18), run_name="__main__")
    master = bpy.data.objects[MASTER]
    floor = float(v18["canvas_floor"])
    for name in ("CTRL_Rope Point_TR_S_mid", "CTRL_Rope Point_TR_W_mid",
                 "CTRL_Rope Point_TR_N_mid", "CTRL_Rope Point_TR_E_mid"):
        obj = bpy.data.objects.get(name)
        if obj is None:
            stop(f"missing midpoint {name}")
        constraint = obj.constraints.get(FLOOR_NAME) or obj.constraints.new("LIMIT_LOCATION")
        constraint.name = FLOOR_NAME
        constraint.owner_space = "WORLD"
        constraint.use_min_z = True
        constraint.min_z = floor
        constraint.influence = 1.0
    for name in CURVES:
        curve = bpy.data.objects[name]
        if len(curve.data.splines[0].bezier_points) != 10:
            stop(f"{name} did not become ten-point")
        for point in curve.data.splines[0].bezier_points:
            point.handle_left_type = "AUTO"
            point.handle_right_type = "AUTO"
        soften_canvas_contact(curve)

    bpy.context.view_layer.update()
    for name in CURVES:
        test_master_motion(master, name)
    offset_props = v18["MASTER_OFFSET_LOCATION_PROPERTIES"]
    master[offset_props[0]] = -1.27  # -50 in
    master[offset_props[2]] = 4.445  # +175 in
    v18["hybrid_apply_master_properties"](master)
    bpy.context.view_layer.update()
    mesh_floor = rope_mesh_minimums()
    print(f"V035 FLOOR mesh_min_z={mesh_floor}; canvas_floor={floor:.6f}")
    if min(mesh_floor.values()) < floor - .04:
        stop("rope mesh penetrates the canvas at the floor test pose")
    for name in CURVES:
        points = evaluated_points(name)
        minimum, turn = min(p.z for p in points), max_turn(points)
        print(f"V035 CURVE {name}: min_z={minimum:.6f}, worst_turn={turn:.2f} deg")
        if minimum < floor - .01 or turn > 20:
            stop(f"{name} failed the floor or smoothness test")

    rope_mesh = bpy.data.objects["MESH_Ring Ropes"]
    material = rope_mesh.data.materials[0] if rope_mesh.data.materials else None
    for name in CURVES:
        make_curve_visible_rope(bpy.data.objects[name], material)
    kept = mask_old_top_rope_mesh()
    bpy.context.view_layer.update()
    evaluated_rope = rope_mesh.evaluated_get(bpy.context.evaluated_depsgraph_get())
    masked_mesh = evaluated_rope.to_mesh()
    try:
        print(f"V035 OTHER ROPES: {len(masked_mesh.vertices)} mesh vertices preserved")
        if len(masked_mesh.vertices) != kept:
            stop("other rope mesh changed unexpectedly")
    finally:
        evaluated_rope.to_mesh_clear()
    for name in CURVES:
        verify_visible_rope(name, floor)
        test_master_motion(master, name)

    master[offset_props[0]] = 0.0
    master[offset_props[2]] = 0.0
    v18["hybrid_apply_master_properties"](master)
    bpy.context.view_layer.update()
    bpy.context.scene["LSO SW TOP VERSION"] = "v035: v018 Hooks, ten points, rounded tube, spline floor"
    print("V035 APPLIED IN MEMORY. No Blend file was saved. Save manually if approved.")


if __name__ == "__main__":
    build()

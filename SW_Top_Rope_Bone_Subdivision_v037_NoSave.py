"""One-run South/West top-rope controls and deform-chain refinement.

Run this file by itself in Blender's Text Editor. A four-point starting scene
is upgraded to the ten-point v036 controls automatically. It then inserts one
deform bone per existing bone (11 -> 22 per side), transfers
the existing vertex weights across each pair, and doubles each Spline IK
chain count. Existing curves, Hooks, artist controls, and properties are left
untouched. This script does not save the .blend. Reruns are harmless.
"""

import bpy
import runpy
from pathlib import Path
from mathutils import Vector

ARMATURE = "BONES_ROPES"
MESH = "MESH_Ring Ropes"
SIDES = {"S": "ROPE_SOUTH_TOP_CURVE", "W": "ROPE_WEST_TOP_CURVE"}
MARKER = "LSO_SW_TOP_SPLINE_BONES_V037"
V36 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\SW_Top_South_Point_Controls_v036_NoSave.py")
V18 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\Add_Turnbuckle_Unit_Hybrid_v018_CanvasSafe_Cable.py")


def ensure_point_controls():
    south = bpy.data.objects.get(SIDES["S"])
    west = bpy.data.objects.get(SIDES["W"])
    if not south or not west or south.type != "CURVE" or west.type != "CURVE":
        raise RuntimeError("South/West top curves are missing. No changes made.")
    counts = [len(obj.data.splines[0].bezier_points) for obj in (south, west)]
    if counts == [10, 10] and south.get("LSO South Point Controls v036") and west.get("LSO West Point Controls v036"):
        return
    if counts not in ([4, 4], [10, 10]):
        raise RuntimeError(f"Unexpected South/West control-point counts {counts}. No changes made.")
    if not V36.is_file():
        raise RuntimeError(f"Missing bundled setup dependency: {V36}. No changes made.")
    if counts == [4, 4] and bpy.data.objects.get("CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"):
        if not V18.is_file():
            raise RuntimeError(f"Missing base setup dependency: {V18}. No changes made.")
        runpy.run_path(str(V18), run_name="__main__")
    runpy.run_path(str(V36), run_name="__main__")
    south = bpy.data.objects.get(SIDES["S"])
    west = bpy.data.objects.get(SIDES["W"])
    if not south or not west or any(len(obj.data.splines[0].bezier_points) != 10 for obj in (south, west)):
        raise RuntimeError("Automatic ten-point setup did not complete; no bone subdivision was attempted.")


def chain_for(curve_name, armature):
    hits = [(bone, constraint)
            for bone in armature.pose.bones
            for constraint in bone.constraints
            if constraint.type == "SPLINE_IK" and constraint.target
            and constraint.target.name == curve_name]
    if len(hits) != 1:
        raise RuntimeError(f"Expected one Spline IK tip for {curve_name}; found {len(hits)}. No changes made.")
    tip, constraint = hits[0]
    bones = []
    current = tip.bone
    for _ in range(constraint.chain_count):
        if current is None:
            raise RuntimeError(f"Incomplete {curve_name} bone chain. No changes made.")
        bones.append(current)
        current = current.parent
    if len(bones) != 11:
        raise RuntimeError(f"Expected 11 original bones for {curve_name}; found {len(bones)}. No changes made.")
    return tip.name, constraint, list(reversed(bones))


def main():
    ensure_point_controls()
    armature = bpy.data.objects.get(ARMATURE)
    mesh = bpy.data.objects.get(MESH)
    if not armature or armature.type != "ARMATURE" or not mesh or mesh.type != "MESH":
        raise RuntimeError("Missing original rope armature or mesh. No changes made.")
    if armature.get(MARKER):
        print("SW TOP v037 already installed; nothing changed.")
        return
    if not any(m.type == "ARMATURE" and m.object == armature for m in mesh.modifiers):
        raise RuntimeError("Rope mesh is not deformed by BONES_ROPES. No changes made.")
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    specs = {}
    all_bones = set()
    for side, curve_name in SIDES.items():
        curve = bpy.data.objects.get(curve_name)
        if not curve or curve.type != "CURVE" or len(curve.data.splines) != 1:
            raise RuntimeError(f"Missing single-spline {curve_name}. No changes made.")
        if len(curve.data.splines[0].bezier_points) != 10:
            raise RuntimeError(f"Run v036 first: {curve_name} does not have ten editable points. No changes made.")
        tip, constraint, chain = chain_for(curve_name, armature)
        if any(b.name in all_bones for b in chain):
            raise RuntimeError("South and West chains overlap. No changes made.")
        all_bones.update(b.name for b in chain)
        if any(b.name not in mesh.vertex_groups for b in chain):
            raise RuntimeError(f"{curve_name} has a deform bone without a matching vertex group. No changes made.")
        if any(f"LSO_{side}_TOP_HALF_{i:02d}" in armature.data.bones or
               f"LSO_{side}_TOP_HALF_{i:02d}" in mesh.vertex_groups
               for i in range(11)):
            raise RuntimeError("Subdivision names already exist but marker is absent. No changes made.")
        specs[side] = (tip, constraint, chain)

    # Save coordinates before changing the rest geometry; the curve is not edited.
    old = {}
    for side, (_tip, _constraint, chain) in specs.items():
        for i, bone in enumerate(chain):
            old[bone.name] = (side, i, bone.head_local.copy(), bone.tail_local.copy(),
                              mesh.vertex_groups[bone.name].index)
    weighted_sides = set()
    old_group_sides = {info[4]: info[0] for info in old.values()}
    for vert in mesh.data.vertices:
        weighted_sides.update(old_group_sides[item.group] for item in vert.groups
                              if item.group in old_group_sides and item.weight > 0)
    if weighted_sides != set(SIDES):
        raise RuntimeError("A South or West top chain has no weighted mesh vertices. No changes made.")
    old_active = bpy.context.view_layer.objects.active
    old_selection = list(bpy.context.selected_objects)
    try:
        for obj in old_selection:
            obj.select_set(False)
        armature.select_set(True)
        bpy.context.view_layer.objects.active = armature
        bpy.ops.object.mode_set(mode="EDIT")
        for side, (_tip, _constraint, chain) in specs.items():
            for i, bone in enumerate(chain):
                edit_bone = armature.data.edit_bones[bone.name]
                head, tail = old[bone.name][2:4]
                mid = (head + tail) * .5
                parent = edit_bone.parent
                was_connected = edit_bone.use_connect
                old_roll = edit_bone.roll
                old_deform = edit_bone.use_deform
                half = armature.data.edit_bones.new(f"LSO_{side}_TOP_HALF_{i:02d}")
                half.head = head
                half.tail = mid
                half.roll = old_roll
                half.use_deform = old_deform
                half.parent = parent
                half.use_connect = bool(parent and was_connected)
                edit_bone.use_connect = False
                edit_bone.head = mid
                edit_bone.parent = half
                edit_bone.use_connect = True
        bpy.ops.object.mode_set(mode="OBJECT")

        for side, (tip_name, _constraint, _chain) in specs.items():
            constraint = next(c for c in armature.pose.bones[tip_name].constraints
                              if c.type == "SPLINE_IK" and c.target.name == SIDES[side])
            constraint.chain_count = 22

        new_indices = {}
        for side in SIDES:
            for i in range(11):
                name = f"LSO_{side}_TOP_HALF_{i:02d}"
                new_indices[(side, i)] = mesh.vertex_groups.new(name=name).index

        # Preserve all unrelated weights. A smooth handoff around each original
        # bone midpoint avoids a hard deformation seam between the two halves.
        to_armature = armature.matrix_world.inverted() @ mesh.matrix_world
        by_index = {info[4]: info for info in old.values()}
        counts = {side: 0 for side in SIDES}
        for vert in mesh.data.vertices:
            relevant = [(item.group, item.weight, by_index[item.group])
                        for item in vert.groups if item.group in by_index]
            if not relevant:
                continue
            co = to_armature @ vert.co
            for index, weight, (side, i, head, tail, _group_index) in relevant:
                axis = tail - head
                t = max(0.0, min(1.0, (co - head).dot(axis) / axis.length_squared))
                u = max(0.0, min(1.0, (t - .30) / .40))
                fraction_to_original = u * u * (3.0 - 2.0 * u)
                mesh.vertex_groups[index].add([vert.index],
                                              weight * fraction_to_original, "REPLACE")
                mesh.vertex_groups[new_indices[(side, i)]].add(
                    [vert.index], weight * (1.0 - fraction_to_original), "REPLACE")
                counts[side] += 1
        mesh.data.update()
        armature[MARKER] = True
        bpy.context.view_layer.update()
        for side, (tip_name, _constraint, _chain) in specs.items():
            if counts[side] == 0:
                raise RuntimeError(f"No rope vertices were weighted for {side}; inspect this scene before saving.")
            print(f"SW TOP v037: {side} chain 11 -> 22 bones; {counts[side]} existing vertex weights split; tip {tip_name} retained.")
        print("All curves, Hook controls, custom properties and other rope chains retained. No .blend was saved.")
    finally:
        if armature.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        armature.select_set(False)
        for obj in old_selection:
            if obj.name in bpy.data.objects:
                obj.select_set(True)
        if old_active and old_active.name in bpy.data.objects:
            bpy.context.view_layer.objects.active = old_active


if __name__ == "__main__":
    main()

"""Repair the v040/v041 regression, preserving the v038 rope rig.

Run in Blender's Text Editor. This does not save the .blend. It removes ONLY
the v040 pickup-follow constraints and the original v041 floor constraints.
It does not replace the rope mesh, curve, Hook targets, or artist offsets.
If starting from a clean four-point scene, it installs v038 internally.
"""

import bpy
import runpy
from pathlib import Path

V038 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\SW_Top_West_CapturedShape_v038_NoSave.py")
MASTER = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
V040_PREFIX = "LSO SW Pickup Follow v040"
OLD_V041_LIMIT = "LSO SW Rope Canvas Contact v041"
CURVES = {"W": "ROPE_WEST_TOP_CURVE", "S": "ROPE_SOUTH_TOP_CURVE"}


def main():
    mesh = bpy.data.objects.get("MESH_Ring Ropes")
    if mesh is None or mesh.type != "MESH":
        raise RuntimeError("Original MESH_Ring Ropes is missing; nothing changed")
    counts = []
    for name in CURVES.values():
        curve = bpy.data.objects.get(name)
        if curve is None or curve.type != "CURVE" or len(curve.data.splines) != 1:
            raise RuntimeError(f"Original curve {name} is missing; nothing changed")
        counts.append(len(curve.data.splines[0].bezier_points))
    if counts == [4, 4]:
        if not V038.is_file():
            raise RuntimeError("v038 setup is missing; nothing changed")
        runpy.run_path(str(V038), run_name="__main__")
    elif counts != [10, 10]:
        raise RuntimeError(f"Unexpected curve point counts {counts}; nothing changed")
    master = bpy.data.objects.get(MASTER)
    if master is None or master.type != "EMPTY":
        raise RuntimeError("v038 setup did not create the SW master Empty; nothing else changed")
    if not master.get("LSO_WEST_XMINUS50_Z175_ARTIST_PROFILE_V038"):
        if not V038.is_file():
            raise RuntimeError("v038 setup is missing; nothing changed")
        runpy.run_path(str(V038), run_name="__main__")

    bases = []
    offsets = []
    for side, curve_name in CURVES.items():
        curve = bpy.data.objects[curve_name]
        for i in range(10):
            base = bpy.data.objects.get(f"BASE_Rope_TR_{side}_Curve_P{i:02d}")
            ctrl = bpy.data.objects.get(f"CTRL_Rope_TR_{side}_Curve_P{i:02d}")
            offset = bpy.data.objects.get(f"OFFSET_Rope_TR_{side}_Curve_P{i:02d}")
            hook = curve.modifiers.get(f"LSO {'West' if side == 'W' else 'South'} Artist P{i:02d}")
            if (base is None or ctrl is None or offset is None
                    or ctrl.parent != base or offset.parent != ctrl
                    or hook is None or hook.type != "HOOK" or hook.object != offset):
                raise RuntimeError(f"{side} P{i:02d} artist Hook chain is incomplete; no constraints removed")
            bases.append(base)
            offsets.append(offset)

    removed_follow = 0
    removed_floor = 0
    for base in bases:
        for constraint in tuple(base.constraints):
            if constraint.name.startswith(V040_PREFIX):
                base.constraints.remove(constraint)
                removed_follow += 1
    for offset in offsets:
        constraint = offset.constraints.get(OLD_V041_LIMIT)
        if constraint:
            offset.constraints.remove(constraint)
            removed_floor += 1
        for key in ("LSO_V041_CANVAS_TOP_M", "LSO_V041_HOOK_REACH_M"):
            if key in offset:
                del offset[key]
    bpy.context.view_layer.update()
    print(f"SW v041 repair: restored v038 Hook motion and canvas contact; "
          f"removed {removed_follow} v040 follow and {removed_floor} old v041 floor constraints. "
          "MESH_Ring Ropes retained. No .blend saved.")


if __name__ == "__main__":
    main()

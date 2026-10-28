"""Make the EXISTING South/West top-rope controls follow the picked-up SW unit.

Run in Blender's Text Editor. This only adds Child Of constraints to existing
curve-control bases. It creates no rope object and never edits MESH_Ring Ropes,
its shape keys, the curves, or their Hooks. Run again with REMOVE_TRAVEL=True
to remove only these constraints. This script never saves the .blend.
"""

import bpy
import math
import runpy
from pathlib import Path

REMOVE_TRAVEL = False
MASTER_NAME = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
CURVES = {"W": "ROPE_WEST_TOP_CURVE", "S": "ROPE_SOUTH_TOP_CURVE"}
CONSTRAINT_PREFIX = "LSO SW Pickup Follow v040"
V38 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\SW_Top_West_CapturedShape_v038_NoSave.py")


def get_rig():
    master = bpy.data.objects.get(MASTER_NAME)
    mesh = bpy.data.objects.get("MESH_Ring Ropes")
    if master is None or master.type != "EMPTY" or mesh is None or mesh.type != "MESH":
        raise RuntimeError("SW master or original MESH_Ring Ropes is missing")
    items = []
    for side, curve_name in CURVES.items():
        curve = bpy.data.objects.get(curve_name)
        if curve is None or curve.type != "CURVE" or len(curve.data.splines) != 1:
            raise RuntimeError(f"Missing existing {curve_name}")
        if len(curve.data.splines[0].bezier_points) != 10:
            raise RuntimeError(f"{curve_name} is not the v038 ten-point curve")
        for i in range(1, 8):
            base = bpy.data.objects.get(f"BASE_Rope_TR_{side}_Curve_P{i:02d}")
            ctrl = bpy.data.objects.get(f"CTRL_Rope_TR_{side}_Curve_P{i:02d}")
            hook = curve.modifiers.get(f"LSO {'West' if side == 'W' else 'South'} Artist P{i:02d}")
            if base is None or ctrl is None or ctrl.parent != base or hook is None or hook.type != "HOOK":
                raise RuntimeError(f"{side} P{i:02d} original Hook chain is incomplete")
            items.append((side, i, base))
    return master, mesh, items


def main():
    if not REMOVE_TRAVEL:
        master = bpy.data.objects.get(MASTER_NAME)
        if master is None or not master.get("LSO_WEST_XMINUS50_Z175_ARTIST_PROFILE_V038"):
            if not V38.is_file():
                raise RuntimeError("Required v038 setup script is missing")
            runpy.run_path(str(V38), run_name="__main__")
    master, mesh, items = get_rig()
    if mesh.get("LSO_SW_MESH_ROPE_V039") or bpy.data.objects.get("SW_TOP_CONTINUOUS_CABLE"):
        raise RuntimeError("Rejected v039 rope geometry is present; restore v038 first")
    if REMOVE_TRAVEL:
        removed = 0
        for _side, _i, base in items:
            for constraint in tuple(base.constraints):
                if constraint.name.startswith(CONSTRAINT_PREFIX):
                    base.constraints.remove(constraint)
                    removed += 1
        bpy.context.view_layer.update()
        print(f"SW v040 removed {removed} pickup constraints; original rope and curves retained. No .blend saved.")
        return
    if all(any(c.name.startswith(CONSTRAINT_PREFIX) for c in base.constraints)
           for _side, _i, base in items):
        print("SW v040 already installed; no changes made.")
        return
    if any(any(c.name.startswith(CONSTRAINT_PREFIX) for c in base.constraints)
           for _side, _i, base in items):
        raise RuntimeError("Partial v040 installation detected; no changes made")
    bpy.context.view_layer.update()
    inverse = master.matrix_world.inverted().copy()
    for side, i, base in items:
        t = i / 8.0
        proximity = t if side == "W" else 1.0 - t
        influence = 0.72 * math.sin(math.pi * t) * proximity
        constraint = base.constraints.new("CHILD_OF")
        constraint.name = f"{CONSTRAINT_PREFIX} {side} P{i:02d}"
        constraint.target = master
        constraint.inverse_matrix = inverse
        constraint.influence = influence
    bpy.context.view_layer.update()
    print("SW v040 installed on existing South/West curve controls; MESH_Ring Ropes unchanged. No .blend saved.")


if __name__ == "__main__":
    main()

"""Install the captured West top-rope shape without changing rope skinning.

Run once in Blender's Text Editor. If the saved file has four-point top
curves, this runs the required existing v036 setup internally. It applies
the artist's exported West offsets only as the SW turnbuckle approaches
X=-50 in, Z=175 in. The intact pose and deform bones are left untouched.
This script does not save the .blend and is safe to rerun.
"""

import bpy
import re
import runpy
from pathlib import Path
from mathutils import Euler, Vector

BASE = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions")
V18 = BASE / "Add_Turnbuckle_Unit_Hybrid_v018_CanvasSafe_Cable.py"
V36 = BASE / "SW_Top_South_Point_Controls_v036_NoSave.py"
MARKER = "LSO_WEST_XMINUS50_Z175_ARTIST_PROFILE_V038"
OFFSETS = {
    (1, "Y"): -0.2540000081062317, (1, "Z"): -0.07620000094175339,
    (2, "X"): 0.1777999997138977,
    (4, "X"): 0.6858000159263611, (4, "Y"): -0.3555999994277954,
    (5, "X"): -0.6858000159263611, (5, "Y"): -0.20319999754428864,
    (7, "X"): -0.02539999969303608, (7, "Y"): -0.6603999733924866,
}


def setup():
    names = ("ROPE_SOUTH_TOP_CURVE", "ROPE_WEST_TOP_CURVE")
    curves = [bpy.data.objects.get(name) for name in names]
    if any(obj is None or obj.type != "CURVE" or len(obj.data.splines) != 1
           for obj in curves):
        raise RuntimeError("Missing South/West top Bezier curves; no changes made.")
    counts = [len(obj.data.splines[0].bezier_points) for obj in curves]
    if counts == [10, 10] and all(obj.get(f"LSO {'South' if i == 0 else 'West'} Point Controls v036")
                                  for i, obj in enumerate(curves)):
        return
    if counts != [4, 4]:
        raise RuntimeError(f"Unexpected top-curve point counts {counts}; no changes made.")
    if not V18.is_file() or not V36.is_file():
        raise RuntimeError("Required v018 or v036 setup file is missing; no changes made.")
    if bpy.data.objects.get("CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"):
        runpy.run_path(str(V18), run_name="__main__")
    runpy.run_path(str(V36), run_name="__main__")
    if any(len(bpy.data.objects[name].data.splines[0].bezier_points) != 10 for name in names):
        raise RuntimeError("Ten-point setup did not complete; artist offsets were not applied.")


def main():
    setup()
    master = bpy.data.objects.get("CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W")
    if master is None or master.type != "EMPTY":
        raise RuntimeError("SW master Empty missing; no artist offsets applied.")
    if master.get(MARKER):
        print("SW v038 already installed; no changes made.")
        return
    controls = {}
    for i in sorted({i for i, _ in OFFSETS}):
        base = bpy.data.objects.get(f"BASE_Rope_TR_W_Curve_P{i:02d}")
        ctrl = bpy.data.objects.get(f"CTRL_Rope_TR_W_Curve_P{i:02d}")
        offset = bpy.data.objects.get(f"OFFSET_Rope_TR_W_Curve_P{i:02d}")
        if not all((base, ctrl, offset)) or ctrl.parent != base or offset.parent != ctrl:
            raise RuntimeError(f"West point P{i:02d} is not in its v036 hierarchy; no changes made.")
        if not base.animation_data or sum(fc.data_path == "location" for fc in base.animation_data.drivers) != 3:
            raise RuntimeError(f"West point P{i:02d} has incomplete drivers; no changes made.")
        controls[i] = (base, ctrl, offset)
    property_names = {(i, axis): f"CTRL_Rope_TR_W_Curve_P{i:02d} | Offset Loc {axis}"
                      for i, axis in OFFSETS}
    if any(name not in master for name in property_names.values()):
        raise RuntimeError("West artist controls are missing; no changes made.")
    current = {key: float(master[name]) for key, name in property_names.items()}
    all_zero = all(abs(v) < 1e-5 for v in current.values())
    all_captured = all(abs(current[key] - target) < 1e-5 for key, target in OFFSETS.items())
    if not (all_zero or all_captured):
        raise RuntimeError("Current West offsets are custom edits; refusing to overwrite them.")
    rest = Vector(tuple(float(master[f"SW_TR_TAUT_CONTROL_{a}"]) for a in "XYZ"))
    rotation = Euler(tuple(float(master[f"SW_TR_DEFAULT_ROTATION_{a}"]) for a in "XYZ"), "XYZ")
    reference = rest + rotation.to_matrix() @ Vector((-1.27, 0.0, 4.445))
    saved_loc, saved_rot = master.location.copy(), master.rotation_euler.copy()
    try:
        master.location = reference
        master.rotation_euler = rotation
        bpy.context.view_layer.update()
        by_point = {}
        for (i, axis), value in OFFSETS.items():
            _base, ctrl, _offset = controls[i]
            local = Vector((0.0, 0.0, 0.0))
            local["XYZ".index(axis)] = value
            by_point[i] = by_point.get(i, Vector()) + ctrl.matrix_world.to_3x3() @ local
    finally:
        master.location = saved_loc
        master.rotation_euler = saved_rot
        bpy.context.view_layer.update()
    edits = []
    for i, delta in by_point.items():
        base, _ctrl, _offset = controls[i]
        for axis in range(3):
            fc = next(fc for fc in base.animation_data.drivers
                      if fc.data_path == "location" and fc.array_index == axis)
            original = fc.driver.expression
            match = re.search(r"\+\(([-+0-9.eE]+)\)\*\(max\(0,min\(1,.*\)\)\)$", original)
            if match is None:
                raise RuntimeError(f"West P{i:02d} driver is not the expected v036 form; no artist offsets applied.")
            replacement = (original[:match.start(1)]
                           + f"{float(match.group(1)) + delta[axis]:.12g}"
                           + original[match.end(1):])
            if len(replacement) > 255:
                raise RuntimeError("A Blender driver would exceed 255 characters; no artist offsets applied.")
            edits.append((fc.driver, replacement))
    for driver, expression in edits:
        driver.expression = expression
    if all_captured:
        for key, name in property_names.items():
            master[name] = 0.0
            controls[key[0]][2].location["XYZ".index(key[1])] = 0.0
    for base, _ctrl, _offset in controls.values():
        base.update_tag(refresh={"OBJECT"})
    master[MARKER] = True
    bpy.context.view_layer.update()
    print("SW v038: captured West shape installed at X=-50in, Z=175in; rest pose and skinning untouched. No .blend saved.")


if __name__ == "__main__":
    main()

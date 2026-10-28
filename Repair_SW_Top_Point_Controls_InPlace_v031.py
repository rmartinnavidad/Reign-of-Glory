"""Repair v029's doubled South/West top-curve controls in the open scene.

Load this file into Blender's Text Editor and run it once. A unique full .blend
backup is saved before any changes. This does not require a scene version marker.
It checks the actual curve/control hierarchy and refuses ambiguous states.
"""

import bpy
from datetime import datetime
from pathlib import Path
from uuid import uuid4
from mathutils import Matrix, Vector


MARKER = "LSO v031 SW Point Parent Repair"
BACKUP_DIR = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\Reset_Snapshots")
CREATE_SNAPSHOT = globals().get("CREATE_SNAPSHOT", True)
STATE_TOLERANCE = 0.10  # metres; allows small hand edits without losing them


def evaluated_point_world(curve, index):
    evaluated = curve.evaluated_get(bpy.context.evaluated_depsgraph_get())
    return evaluated.matrix_world @ evaluated.data.splines[0].bezier_points[index].co


def repair():
    scene = bpy.context.scene
    if MARKER in scene or "LSO v030 SW Point Parent Repair" in scene:
        print("V031_ALREADY_REPAIRED::no changes")
        return

    pairs = []
    for side, curve_name in (("S", "ROPE_SOUTH_TOP_CURVE"),
                             ("W", "ROPE_WEST_TOP_CURVE")):
        curve = bpy.data.objects.get(curve_name)
        if (curve is None or curve.type != "CURVE" or
                len(curve.data.splines) != 1 or
                len(curve.data.splines[0].bezier_points) != 10):
            raise RuntimeError(f"{curve_name} is not the expected ten-point curve; no changes made")
        for index in range(10):
            name = f"CTRL_Rope_TR_{side}_Curve_P{index:02d}"
            control = bpy.data.objects.get(name)
            proxy_name = f"TECH_SW_TR_{side}_PointFollow_P{index:02d}"
            if (control is None or control.parent is None or
                    control.parent.name != proxy_name or
                    "LSO Control Origin World XYZ" not in control):
                raise RuntimeError(f"{name} lacks the expected curve-follow parent; no changes made")
            try:
                origin = Vector(control["LSO Control Origin World XYZ"])
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"{name} has an invalid stored origin; no changes made") from exc
            if len(origin) != 3 or origin.length < 0.5:
                raise RuntimeError(f"{name} has an unexpected stored origin; no changes made")
            pairs.append((curve, index, control, origin))

    # The v029 bug duplicated each original world coordinate into the child's
    # local Location. Fixed controls have local Location near zero instead.
    # Check every point so an already-fixed or mixed scene cannot be damaged.
    broken = [p for p in pairs if (p[2].location - p[3]).length <= STATE_TOLERANCE]
    fixed = [p for p in pairs if p[2].location.length <= STATE_TOLERANCE]
    if len(fixed) == len(pairs):
        print("V031_ALREADY_ALIGNED::all 20 controls are already in the fixed state; no changes")
        return
    if len(broken) != len(pairs):
        raise RuntimeError(
            f"Mixed or artist-edited control state ({len(broken)}/20 doubled, "
            f"{len(fixed)}/20 already fixed). No changes made."
        )

    snapshot = None
    if CREATE_SNAPSHOT:
        BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        snapshot = BACKUP_DIR / f"Before_v031_{stamp}_{uuid4().hex[:8]}.blend"
        result = bpy.ops.wm.save_as_mainfile(
            filepath=str(snapshot), copy=True, compress=False)
        if "FINISHED" not in result or not snapshot.is_file():
            raise RuntimeError("Could not make a backup; no controls were changed")

    originals = [(control, control.location.copy(),
                  control.matrix_parent_inverse.copy())
                 for _, _, control, _ in pairs]
    try:
        for curve, index, control, origin in pairs:
            control.location = control.location - origin
            control.matrix_parent_inverse = Matrix.Identity(4)
            control.update_tag(refresh={"OBJECT"})
            curve.data.update_tag()
            curve.update_tag(refresh={"OBJECT", "DATA"})
        bpy.context.view_layer.update()

        gaps = [(control.matrix_world.translation -
                 evaluated_point_world(curve, index)).length
                for curve, index, control, _ in pairs]
        if max(gaps) > 0.025:
            raise RuntimeError(
                f"Controls still differ from their curves by {max(gaps):.4f} m")

        for side, index in (("S", 2), ("W", 0)):
            curve = bpy.data.objects[
                f"ROPE_{'SOUTH' if side == 'S' else 'WEST'}_TOP_CURVE"]
            control = bpy.data.objects[f"CTRL_Rope_TR_{side}_Curve_P{index:02d}"]
            before = evaluated_point_world(curve, index)
            control.location.x += 0.02
            control.update_tag(refresh={"OBJECT"})
            bpy.context.view_layer.update()
            moved = (evaluated_point_world(curve, index) - before).length
            control.location.x -= 0.02
            control.update_tag(refresh={"OBJECT"})
            bpy.context.view_layer.update()
            if moved < 0.01:
                raise RuntimeError(f"{control.name} did not move its curve point")

    except Exception:
        for control, location, inverse in originals:
            control.location = location
            control.matrix_parent_inverse = inverse
            control.update_tag(refresh={"OBJECT"})
        for curve, _, _, _ in pairs:
            curve.data.update_tag()
        bpy.context.view_layer.update()
        raise

    scene[MARKER] = str(snapshot) if snapshot else "tested without snapshot"
    print(f"V031_REPAIR_COMPLETE::20 controls aligned; max gap {max(gaps):.5f} m; "
          f"backup={snapshot}")


repair()

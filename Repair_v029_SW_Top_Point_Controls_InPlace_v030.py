"""Repair the doubled SW top-rope point controls left by v029, in place.

Run once in the open v029 scene. It saves a unique full .blend copy first,
then repairs only the 20 South/West top-curve point control transforms.
It keeps each control's manual offset and does not change the rope rig,
turnbuckle, curves, Hooks, bones, or mesh directly. It does not overwrite
the open .blend. Future fresh installs should use the corrected v029 file.
"""

import bpy
from datetime import datetime
from pathlib import Path
from uuid import uuid4
from mathutils import Matrix, Vector


MARKER = "LSO v030 SW Point Parent Repair"
BACKUP_DIR = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\Reset_Snapshots")
CREATE_SNAPSHOT = globals().get("CREATE_SNAPSHOT", True)


def evaluated_point_world(curve, index):
    evaluated = curve.evaluated_get(bpy.context.evaluated_depsgraph_get())
    return evaluated.matrix_world @ evaluated.data.splines[0].bezier_points[index].co


def repair():
    scene = bpy.context.scene
    if MARKER in scene:
        print("V030_ALREADY_REPAIRED::no changes")
        return
    if "LSO v029 Reset Snapshot Path" not in scene:
        raise RuntimeError("This scene was not built by v029; no changes made")

    pairs = []
    for side, curve_name in (("S", "ROPE_SOUTH_TOP_CURVE"),
                             ("W", "ROPE_WEST_TOP_CURVE")):
        curve = bpy.data.objects.get(curve_name)
        if (curve is None or curve.type != "CURVE" or
                len(curve.data.splines) != 1 or
                len(curve.data.splines[0].bezier_points) != 10):
            raise RuntimeError(f"{curve_name} is not the expected ten-point curve")
        for index in range(10):
            name = f"CTRL_Rope_TR_{side}_Curve_P{index:02d}"
            control = bpy.data.objects.get(name)
            proxy_name = f"TECH_SW_TR_{side}_PointFollow_P{index:02d}"
            if (control is None or control.parent is None or
                    control.parent.name != proxy_name or
                    "LSO Control Origin World XYZ" not in control):
                raise RuntimeError(f"{name} is not in the captured v029 state")
            pairs.append((curve, index, control))

    snapshot = None
    if CREATE_SNAPSHOT:
        BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        snapshot = BACKUP_DIR / f"Before_v030_{stamp}_{uuid4().hex[:8]}.blend"
        result = bpy.ops.wm.save_as_mainfile(
            filepath=str(snapshot), copy=True, compress=False)
        if "FINISHED" not in result or not snapshot.is_file():
            raise RuntimeError("Could not make a backup; no controls were changed")

    originals = [(control, control.location.copy(),
                  control.matrix_parent_inverse.copy())
                 for _, _, control in pairs]
    try:
        for curve, index, control in pairs:
            origin = Vector(control["LSO Control Origin World XYZ"])
            # The broken v029 stored the original world coordinate as the
            # child's local coordinate as well. Subtract only that duplicate;
            # any artist-created local offset remains intact.
            control.location = control.location - origin
            control.matrix_parent_inverse = Matrix.Identity(4)
            control.update_tag(refresh={"OBJECT"})
            curve.data.update_tag()
            curve.update_tag(refresh={"OBJECT", "DATA"})
        bpy.context.view_layer.update()

        gaps = []
        for curve, index, control in pairs:
            gap = (control.matrix_world.translation -
                   evaluated_point_world(curve, index)).length
            gaps.append(gap)
        if max(gaps) > 0.025:
            raise RuntimeError(
                f"Point controls still differ from their curves by {max(gaps):.4f} m")

        for side, index in (("S", 2), ("W", 0)):
            curve = bpy.data.objects[f"ROPE_{'SOUTH' if side == 'S' else 'WEST'}_TOP_CURVE"]
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
        for curve, _, _ in pairs:
            curve.data.update_tag()
        bpy.context.view_layer.update()
        raise

    scene[MARKER] = str(snapshot) if snapshot else "tested without snapshot"
    print(f"V030_REPAIR_COMPLETE::20 controls aligned; max gap {max(gaps):.5f} m; "
          f"snapshot={snapshot}")


repair()

"""Build the driven SW top-rope curves from the original four-point scene.

This runs the corrected v029 four-to-ten-point installer once, then checks that
all 20 new point controls are present and coupled to the evaluated curves.
It does not run v031, which only repairs an already-installed ten-point rig.
"""

import bpy
import runpy
from pathlib import Path


INSTALLER = Path(
    r"C:\Users\rmart\Downloads\Rope_Rig_Versions"
    r"\Add_Turnbuckle_Unit_Hybrid_v029_DrivenExtraPoints_ApplyOrReset.py"
)


def evaluated_point_world(curve, index):
    dg = bpy.context.evaluated_depsgraph_get()
    evaluated = curve.evaluated_get(dg)
    return evaluated.matrix_world @ evaluated.data.splines[0].bezier_points[index].co


def run():
    if not INSTALLER.is_file():
        raise RuntimeError(f"Required corrected v029 installer is missing: {INSTALLER}")
    if "LSO v029 Reset Snapshot Path" in bpy.context.scene:
        raise RuntimeError(
            "This scene has a v029 reset marker. Do not rerun v029 here: "
            "it would reset the scene. No changes made."
        )

    for side, curve_name in (("S", "ROPE_SOUTH_TOP_CURVE"),
                             ("W", "ROPE_WEST_TOP_CURVE")):
        curve = bpy.data.objects.get(curve_name)
        if (curve is None or curve.type != "CURVE" or
                len(curve.data.splines) != 1 or
                curve.data.splines[0].type != "BEZIER" or
                len(curve.data.splines[0].bezier_points) != 4):
            raise RuntimeError(
                f"{curve_name} must start with one four-point Bezier spline; "
                "no changes made."
            )
        for index in range(10):
            name = f"CTRL_Rope_TR_{side}_Curve_P{index:02d}"
            if bpy.data.objects.get(name) is not None:
                raise RuntimeError(
                    f"{name} already exists in this four-point scene. "
                    "This is a mixed saved state; no changes made."
                )

    # v029 creates a unique pre-install .blend snapshot and restores it if
    # installation raises an exception. It also creates the 10-point splines.
    runpy.run_path(str(INSTALLER), run_name="__main__")
    bpy.context.view_layer.update()

    for side, curve_name in (("S", "ROPE_SOUTH_TOP_CURVE"),
                             ("W", "ROPE_WEST_TOP_CURVE")):
        curve = bpy.data.objects[curve_name]
        points = curve.data.splines[0].bezier_points
        if len(points) != 10:
            raise RuntimeError(f"{curve_name} did not become a ten-point curve")
        for index in range(10):
            control = bpy.data.objects.get(
                f"CTRL_Rope_TR_{side}_Curve_P{index:02d}")
            if control is None:
                raise RuntimeError(f"Missing {side} point control P{index:02d}")
            gap = (control.matrix_world.translation -
                   evaluated_point_world(curve, index)).length
            if gap > 0.025:
                raise RuntimeError(
                    f"{control.name} is {gap:.4f} m from its curve point")

    print("V032_BUILD_COMPLETE::South and West each have 10 points; "
          "all 20 controls align with their curve points")


run()

"""Add one Object Mode control for the southwest top turnbuckle unit.

Creates exactly one master empty for this unit:

    CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W
        -> BONES_Turnbuckles / TB_TR_S_W_ROOT
            -> existing jaw, body, rope-jaw and hardware hierarchy
        -> CTRL_Turnbuckle_Pad_TR_S_W_IK (same rigid delta)

TB_TR_S_W_FOAM remains attached to TB_TR_S_W_BODY with its mounting offset.
Both CTRL_Rope Point_TR_S_end_r and CTRL_Rope Point_TR_W_end_l remain locked
directly to the rope-side eyehook. The west tangent control retains its offset.

The foam mesh is already bone-parented through WRAP_Turnbuckle_Foam_TR_S_W,
and the sleeve is parented to that foam mesh, so both objects follow the foam
bone without extra object constraints or double transforms.

Move or rotate the empty in Object Mode. Running this script again reuses the
same empty and constraints; it does not create duplicates.

The south and west top-rope spans are calibrated from the visible current
position. Moving the corner inward creates available bend/sag; pulling it back
to the calibrated position straightens it again.
"""

import bpy
import time
from mathutils import Vector


ARMATURE_NAME = "BONES_Turnbuckles"
ROOT_BONE_NAME = "TB_TR_S_W_ROOT"
OLD_CONTROLLED_BONE_NAME = "TB_TR_S_W_JAW_POST"
BODY_BONE_NAME = "TB_TR_S_W_BODY"
FOAM_BONE_NAME = "TB_TR_S_W_FOAM"
EMPTY_NAME = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
ROOT_CONSTRAINT_NAME = "Object_Mode_Whole_Unit_Root_Control"
OLD_CONSTRAINT_NAME = "Object_Mode_Jaw_Post_Control"
FOAM_CONSTRAINT_NAME = "Object_Mode_Whole_Unit_Foam_Follow"
PAD_IK_NAME = "CTRL_Turnbuckle_Pad_TR_S_W_IK"
PAD_IK_BASE_LOCATION = (-1.886641502380371, 1.8772505521774292, 2.559016704559326)
PAD_FOLLOW_CONSTRAINT_NAME = "Object_Mode_Whole_Unit_Pad_Follow"
ROPE_EYEHOOK_NAME = "Ring Hardware_Turnbuckle_Eyehook_Rope_TR_S_W"
ROPE_ENDPOINT_LINKS = (
    ("CTRL_Rope Point_TR_S_end_r", "PERMANENT_SW_ENDPOINT_SOURCE"),
    ("CTRL_Rope Point_TR_W_end_l", "PERMANENT_SW_WEST_ENDPOINT_SOURCE"),
)
WEST_TANGENT_NAME = "CTRL_Rope Point_TR_W_end_l_ctrl"
WEST_TANGENT_CONSTRAINT_NAME = "PERMANENT_SW_TURNBUCKLE_REALTIME"

ROPE_CALIBRATION_VERSION = 9
ROPE_CALIBRATION_PROPERTY = "SW_TR_FIXED_ROPE_CALIBRATION_VERSION"
FIXED_REACH_HANDLER_KEY = "SW_TR_FIXED_ROPE_REACH_HANDLER"
RECOIL_STATE_KEY = "SW_TR_TOP_ROPE_RECOIL_STATE"
RECOIL_TIMER_KEY = "SW_TR_TOP_ROPE_RECOIL_TIMER"
RECOIL_STRENGTH_PROPERTY = "Top Rope Recoil Strength"
RECOIL_DAMPING_PROPERTY = "Top Rope Recoil Damping"
LEGACY_BROKEN_MODE_PROPERTY = "Broken Turnbuckle Weapon Mode"
BROKEN_FOLLOW_PROPERTY = "Broken Rope Corner Follow"
BROKEN_MAX_PULL_PROPERTY = "Broken Rope Maximum Corner Pull"
BROKEN_WIGGLE_PROPERTY = "Broken Rope Wiggle"
CORNER_RESPONSE_VERSION_PROPERTY = "SW_TR_CORNER_RESPONSE_VERSION"
CORNER_RESPONSE_VERSION = 9
# Extra lateral flex, measured in rope-space metres per metre of endpoint
# motion. The captured lateral bow is preserved as a fixed offset; newly
# available slack is sent downward by gravity instead of magnifying that bow.
ROPE_WIGGLE_RESPONSE = 0.08
INWARD_FULL_SAG_DISTANCE = 0.3048  # 12 inches
# Blender evaluates the endpoint constraint chain and midpoint driver in
# separate dependency-graph stages. Keep a small internal reserve so a frame of
# evaluation tolerance can never make the visible rope longer than its captured
# physical length. The stored/reportable rope length itself remains unchanged.
ROPE_LENGTH_SAFETY_INSET = 0.020  # 2 cm over a roughly 5.12 m rope span

TOP_ROPE_FLEX_VERSION = 3
TOP_ROPE_FLEX_FRACTIONS = (1.0 / 6.0, 1.0 / 3.0, 2.0 / 3.0, 5.0 / 6.0)
TOP_ROPE_FLEX_CONFIGS = (
    ("S", "ROPE_SOUTH_TOP_CURVE", "CTRL_Rope Point_TR_S_end_r",
     "CTRL_Rope Point_TR_S_mid", "CTRL_Rope Point_TR_S_end_l",
     "CTRL_Rope Point_TR_S_end_l_ctrl", (
         ((-2.566390, 2.608885, 2.548229), (-3.683158, 2.608885, 2.548229), (-1.449623, 2.608885, 2.548229)),
         ((0.294099, 2.608885, 2.548229), (-0.822658, 2.613818, 2.548229), (1.199927, 2.604883, 2.548229)),
         ((2.614218, 2.588387, 2.548229), (1.708390, 2.592388, 2.548229), (3.199996, 2.585799, 2.548229)),
         ((4.114643, 2.588387, 2.548229), (3.528860, 2.588387, 2.548229), (4.700426, 2.588387, 2.548229)),
     )),
    ("W", "ROPE_WEST_TOP_CURVE", "CTRL_Rope Point_TR_W_end_r",
     "CTRL_Rope Point_TR_W_mid", "CTRL_Rope Point_TR_W_end_l",
     "CTRL_Rope Point_TR_W_end_l_ctrl", (
         ((-2.478252, 4.085457, 2.515155), (-2.479623, 5.200018, 2.511582), (-2.476880, 2.970896, 2.518727)),
         ((-2.474739, 1.230621, 2.524304), (-2.465266, 2.345115, 2.515707), (-2.482356, 0.334467, 2.531218)),
         ((-2.516582, -1.064387, 2.552361), (-2.508414, -0.168227, 2.546884), (-2.521914, -1.649313, 2.555937)),
         ((-2.516582, -2.562707, 2.552361), (-2.516582, -1.977746, 2.552361), (-2.516582, -3.147668, 2.552361)),
     )),
    ("N", "ROPE_NORTH_TOP_CURVE", "CTRL_Rope Point_TR_N_end_r",
     "CTRL_Rope Point_TR_N_mid", "CTRL_Rope Point_TR_N_end_l",
     "CTRL_Rope Point_TR_N_end_l_ctrl", (
         ((2.600851, -2.546337, 2.522466), (3.501987, -2.524391, 2.521809), (1.699715, -2.568283, 2.523124)),
         ((0.292681, -2.602550, 2.524150), (1.194034, -2.593965, 2.519928), (-0.822603, -2.613172, 2.529374)),
         ((-2.564023, -2.587406, 2.548826), (-1.448825, -2.605105, 2.553470), (-3.170528, -2.577780, 2.546299)),
         ((-4.116968, -2.546338, 2.522466), (-3.510680, -2.562371, 2.532757), (-4.723256, -2.530305, 2.512176)),
     )),
    ("E", "ROPE_EAST_TOP_CURVE", "CTRL_Rope Point_TR_E_end_r",
     "CTRL_Rope Point_TR_E_mid", "CTRL_Rope Point_TR_E_end_l",
     "CTRL_Rope Point_TR_E_end_l_ctrl", (
         ((2.636738, 2.704089, 2.527290), (2.636414, 3.644546, 2.527691), (2.637062, 1.763633, 2.526889)),
         ((2.637568, 0.295203, 2.526263), (2.640844, 1.235654, 2.526999), (2.632988, -1.019332, 2.525235)),
         ((2.612949, -3.071776, 2.522432), (2.617700, -1.757241, 2.523221), (2.611539, -3.461793, 2.522198)),
         ((2.613031, -4.070771, 2.522371), (2.612999, -3.680752, 2.522395), (2.613064, -4.460790, 2.522347)),
     )),
)

# At rest these tangent helpers intentionally retain the captured artwork.
# Once the southwest master is lifted, each helper blends onto the straight
# continuation from midpoint through endpoint. This prevents the endpoint from
# becoming a sharp elbow while keeping the original four-point curves.
UPWARD_TANGENT_CONFIGS = (
    {
        "label": "SOUTH_TOP_FAR_END",
        "midpoint": "CTRL_Rope Point_TR_S_mid",
        "endpoint": "CTRL_Rope Point_TR_S_end_l",
        "tangent": "CTRL_Rope Point_TR_S_end_l_ctrl",
        "length": 1.480871701,
    },
    {
        "label": "EAST_TOP_FAR_END",
        "midpoint": "CTRL_Rope Point_TR_E_mid",
        "endpoint": "CTRL_Rope Point_TR_E_end_l",
        "tangent": "CTRL_Rope Point_TR_E_end_l_ctrl",
        "length": 1.418836320,
    },
)
UPWARD_TANGENT_CONSTRAINT = "SW_TR_UPWARD_STRAIGHT_TANGENT"

UPWARD_MIDPOINT_TILT_CONFIGS = (
    ("CTRL_Rope Point_TR_E_mid", "CTRL_Rope Point_TR_E_end_l"),
    ("CTRL_Rope Point_TR_N_mid", "CTRL_Rope Point_TR_N_end_l"),
)
UPWARD_MIDPOINT_TILT_CONSTRAINT = "SW_TR_UPWARD_MIDPOINT_TILT"

POSITIVE_X_MIDPOINT_TUNING = {
    "CTRL_Rope Point_TR_N_mid": (
        "Positive X North Mid X",
        "Positive X North Mid Y",
        "Positive X North Mid Z",
    ),
    "CTRL_Rope Point_TR_E_mid": (
        "Positive X East Mid X",
        "Positive X East Mid Y",
        "Positive X East Mid Z",
    ),
}


RECOIL_TARGETS = {
    "SE": {
        "object": "CTRL_Turnbuckle_Pad_TR_S_E_IK",
        "direction": (1.0, 0.0, -0.18),
        "gain": 0.85,
        "delay": 0.015,
        "maximum": 0.040,
    },
    "NW": {
        "object": "CTRL_Turnbuckle_Pad_TR_N_W_IK",
        "direction": (0.0, -1.0, -0.18),
        "gain": 0.75,
        "delay": 0.025,
        "maximum": 0.040,
    },
    "NE": {
        "object": "CTRL_Turnbuckle_Pad_TR_N_E_IK",
        "direction": (0.70710678, -0.70710678, -0.12),
        "gain": 0.38,
        "delay": 0.055,
        "maximum": 0.022,
    },
}

RECOIL_MIDPOINTS = {
    "N": {
        "object": "CTRL_Rope Point_TR_N_mid",
        "direction": (0.0, -0.45, -0.89),
        "sources": (("NW", 0.55), ("NE", 0.35)),
    },
    "E": {
        "object": "CTRL_Rope Point_TR_E_mid",
        "direction": (0.45, 0.0, -0.89),
        "sources": (("SE", 0.55), ("NE", 0.35)),
    },
}
RECOIL_MIDPOINT_CONSTRAINT = "SW_TR_PROPAGATED_RECOIL"

# Stable, one-way response weights for the three still-attached top corners.
# Adjacent corners receive more of the pull; the diagonal corner receives less.
BROKEN_CORNER_FOLLOW = {
    "CTRL_Turnbuckle_Pad_TR_S_E_IK": (1.00, 0.35, 0.70),
    "CTRL_Turnbuckle_Pad_TR_N_W_IK": (0.45, 1.00, 0.70),
    "CTRL_Turnbuckle_Pad_TR_N_E_IK": (0.30, 0.30, 0.40),
}

# Baseline positions come from the supplied ring-hardware capture before this
# corner is moved. They define each rope's fixed rest length and slight normal
# sag/bow. When the moving endpoint travels inward, the midpoint driver converts
# the lost chord distance into additional vertical sag.
SAG_CONFIGS = (
    {
        "label": "SOUTH_TOP",
        "midpoint": "CTRL_Rope Point_TR_S_mid",
        "end_a": "CTRL_Rope Point_TR_S_end_r",
        "end_b": "CTRL_Rope Point_TR_S_end_l",
        "curve": "ROPE_SOUTH_TOP_CURVE",
        "flex_axis": "Y",
        "base_a": (-2.5625712871551514, 2.5591719150543213, 2.5480053424835205),
        "base_b": (2.562570810317993, 2.5591721534729004, 2.548007011413574),
        "base_mid": (1.7462298274040222e-10, 2.6330463886260986, 2.5264062881469727),
    },
    {
        "label": "WEST_TOP",
        "midpoint": "CTRL_Rope Point_TR_W_mid",
        "end_a": "CTRL_Rope Point_TR_W_end_l",
        "end_b": "CTRL_Rope Point_TR_W_end_r",
        "curve": "ROPE_WEST_TOP_CURVE",
        "flex_axis": "X",
        "base_a": (-2.5625712871551514, 2.5591719150543213, 2.5480053424835205),
        "base_b": (-2.562666177749634, -2.559069871902466, 2.5480058193206787),
        "base_mid": (-2.6269009113311768, 0.0, 2.5243046283721924),
    },
)


def add_world_transform_variable(driver, name, obj, transform_type):
    variable = driver.variables.new()
    variable.name = name
    variable.type = "TRANSFORMS"
    target = variable.targets[0]
    target.id = obj
    target.transform_type = transform_type
    target.transform_space = "WORLD_SPACE"


def add_location_difference_variable(driver, name, end_a, end_b):
    variable = driver.variables.new()
    variable.name = name
    variable.type = "LOC_DIFF"
    for target, obj in zip(variable.targets, (end_a, end_b)):
        target.id = obj
        target.transform_space = "WORLD_SPACE"


def drive_constraint_value(
        constraint, property_name, expression, end_a, end_b, control=None):
    try:
        constraint.driver_remove(property_name)
    except (TypeError, RuntimeError):
        pass
    fcurve = constraint.driver_add(property_name)
    driver = fcurve.driver
    driver.type = "SCRIPTED"
    driver.expression = expression
    for prefix, obj in (("a", end_a), ("b", end_b)):
        add_world_transform_variable(driver, prefix + "x", obj, "LOC_X")
        add_world_transform_variable(driver, prefix + "y", obj, "LOC_Y")
        add_world_transform_variable(driver, prefix + "z", obj, "LOC_Z")
    if control is not None:
        add_world_transform_variable(driver, "mx", control, "LOC_X")
        add_world_transform_variable(driver, "my", control, "LOC_Y")
        add_world_transform_variable(driver, "mz", control, "LOC_Z")


def calibration_key(config, suffix):
    return "SW_TR_" + config["label"] + "_" + suffix


def calibrate_current_state_as_tight(control):
    """One-time migration: the visible state at install time is taut."""
    if int(control.get(ROPE_CALIBRATION_PROPERTY, 0)) == ROPE_CALIBRATION_VERSION:
        return

    bpy.context.view_layer.update()
    if all(key in control for key in (
            "SW_TR_TAUT_CONTROL_X", "SW_TR_TAUT_CONTROL_Y",
            "SW_TR_TAUT_CONTROL_Z")):
        control_world = Vector((
            float(control["SW_TR_TAUT_CONTROL_X"]),
            float(control["SW_TR_TAUT_CONTROL_Y"]),
            float(control["SW_TR_TAUT_CONTROL_Z"]),
        ))
    else:
        control_world = control.matrix_world.translation.copy()
    for config in SAG_CONFIGS:
        end_a = bpy.data.objects.get(config["end_a"])
        end_b = bpy.data.objects.get(config["end_b"])
        midpoint = bpy.data.objects.get(config["midpoint"])
        if end_a is None or end_b is None or midpoint is None:
            raise RuntimeError(
                f"Cannot calibrate {config['label']}: rope control is missing.")
        # Use the captured intact pose rather than whatever temporary stretched
        # position the artist may be testing when this upgrade is run.
        a = Vector(config["base_a"])
        b = Vector(config["base_b"])
        m = Vector(config["base_mid"])
        # Preserve the rope that is visibly present, including its small
        # existing bow.  Using only the endpoint chord incorrectly declares
        # that bow to be zero-length slack and makes the rope grow later.
        control[calibration_key(config, "FIXED_LENGTH")] = (
            (a - m).length + (m - b).length)
        control[calibration_key(config, "TAUT_CHORD")] = (a - b).length
        # A translated rigid eyehook is equivalent to constraining the master
        # around a fixed sphere whose center is offset by eyehook-master.
        center = b - (a - control_world)
        control[calibration_key(config, "CENTER_X")] = center.x
        control[calibration_key(config, "CENTER_Y")] = center.y
        control[calibration_key(config, "CENTER_Z")] = center.z
        control[calibration_key(config, "A0_X")] = a.x
        control[calibration_key(config, "A0_Y")] = a.y
        control[calibration_key(config, "A0_Z")] = a.z

    control["SW_TR_TAUT_CONTROL_X"] = control_world.x
    control["SW_TR_TAUT_CONTROL_Y"] = control_world.y
    control["SW_TR_TAUT_CONTROL_Z"] = control_world.z
    control[ROPE_CALIBRATION_PROPERTY] = ROPE_CALIBRATION_VERSION


def install_fixed_length_sag(config, control):
    midpoint = bpy.data.objects.get(config["midpoint"])
    end_a = bpy.data.objects.get(config["end_a"])
    end_b = bpy.data.objects.get(config["end_b"])
    curve = bpy.data.objects.get(config["curve"])
    missing = [name for name, obj in (
        (config["midpoint"], midpoint),
        (config["end_a"], end_a),
        (config["end_b"], end_b),
        (config["curve"], curve),
    ) if obj is None]
    if missing:
        raise RuntimeError("Missing sag rig object(s): " + ", ".join(missing))

    fixed_length = float(control[calibration_key(config, "FIXED_LENGTH")])
    modeled_length = max(0.0, fixed_length - ROPE_LENGTH_SAFETY_INSET)
    fixed_length_sq = modeled_length * modeled_length
    base_a = Vector(config["base_a"])
    base_b = Vector(config["base_b"])
    base_mid = Vector(config["base_mid"])
    base_center = (base_a + base_b) * 0.5
    base_down = max(1.0e-9, base_center.z - base_mid.z)

    constraint_name = "FIXED_LENGTH_INWARD_SAG_" + config["label"]
    sag_constraint = midpoint.constraints.get(constraint_name)
    if sag_constraint is None:
        sag_constraint = midpoint.constraints.new("LIMIT_LOCATION")
        sag_constraint.name = constraint_name
    elif sag_constraint.type != "LIMIT_LOCATION":
        raise RuntimeError(f"{constraint_name} exists but is not Limit Location.")

    sag_constraint.owner_space = "WORLD"
    sag_constraint.use_transform_limit = False
    sag_constraint.use_min_x = sag_constraint.use_max_x = True
    sag_constraint.use_min_y = sag_constraint.use_max_y = True
    sag_constraint.use_min_z = sag_constraint.use_max_z = True
    sag_constraint.influence = 1.0
    sag_constraint.mute = False

    # A fixed-length two-span cable has midpoint offset
    # h = 1/2*sqrt(L^2 - chord^2). Preserve the captured lateral bow as an
    # offset, then assign all additional slack to the vertical drop. The old
    # model preserved the bow as a ratio of h, which made a small rest bow grow
    # into a huge sideways bulge and stole almost all downward sag.
    # The available sag is calculated from the full 3D endpoint chord. This is
    # essential when a corner is raised: vertical distance consumes rope length
    # just like horizontal distance and can never be ignored or manufactured.
    chord_sq = "(ax-bx)**2+(ay-by)**2+(az-bz)**2"
    height_sq = f"max(0,.25*({fixed_length_sq:.10g}-({chord_sq})))"
    if config["flex_axis"] == "Y":
        base_lateral = base_mid.y - base_center.y
        base_height_sq = base_lateral * base_lateral + base_down * base_down
        lateral = (
            f"({base_lateral:.10g})*sqrt(min(1,"
            f"({height_sq})/{base_height_sq:.10g}))"
        )
        vertical = (
            f"sqrt(max(0,({height_sq})-"
            f"{base_lateral * base_lateral:.10g}))"
        )
        x_expression = "(ax+bx)*0.5"
        y_expression = f"(ay+by)*0.5+({lateral})"
    else:
        base_lateral = base_mid.x - base_center.x
        base_height_sq = base_lateral * base_lateral + base_down * base_down
        lateral = (
            f"({base_lateral:.10g})*sqrt(min(1,"
            f"({height_sq})/{base_height_sq:.10g}))"
        )
        vertical = (
            f"sqrt(max(0,({height_sq})-"
            f"{base_lateral * base_lateral:.10g}))"
        )
        x_expression = f"(ax+bx)*0.5+({lateral})"
        y_expression = "(ay+by)*0.5"
    rest_master = Vector((
        float(control["SW_TR_TAUT_CONTROL_X"]),
        float(control["SW_TR_TAUT_CONTROL_Y"]),
        float(control["SW_TR_TAUT_CONTROL_Z"]),
    ))
    local_z = control.matrix_world.to_3x3() @ Vector((0.0, 0.0, 1.0))
    local_z.normalize()
    local_x = control.matrix_world.to_3x3() @ Vector((1.0, 0.0, 0.0))
    local_x.normalize()
    # Tension is controlled only by travel along the turnbuckle's local Z.
    # At the untouched default the rope is pulled tight. Positive local-Z
    # movement pushes inward and progressively releases slack; negative travel
    # pulls outward and remains tight. Vertical local-X movement cannot switch
    # tension modes or cancel inward sag.
    # Local Z is horizontal in this rig (world-Z contribution is 0.0018), so
    # the compact X/Y projection is both more stable and short enough for
    # Blender's driver-expression limit.
    local_z_travel = (
        f"(mx{(-rest_master.x):+.8g})*{local_z.x:.8g}+"
        f"(my{(-rest_master.y):+.8g})*{local_z.y:.8g}"
    )
    slack_factor = (
        f"max(0.0,min(1.0,({local_z_travel})"
        f"/{INWARD_FULL_SAG_DISTANCE:.17g}))"
    )
    if config["flex_axis"] == "Y":
        y_expression = f"(ay+by)*0.5+({lateral})*({slack_factor})"
    else:
        x_expression = f"(ax+bx)*0.5+({lateral})*({slack_factor})"
    z_expression = f"(az+bz)*0.5-({vertical})*({slack_factor})"

    for minimum, maximum, expression in (
        ("min_x", "max_x", x_expression),
        ("min_y", "max_y", y_expression),
        ("min_z", "max_z", z_expression),
    ):
        drive_constraint_value(
            sag_constraint, minimum, expression, end_a, end_b, control)
        drive_constraint_value(
            sag_constraint, maximum, expression, end_a, end_b, control)

    # These curves already use Bézier splines. Reassert smooth automatic
    # handles and sufficient viewport resolution so no polygonal elbow appears.
    if curve.type != "CURVE":
        raise RuntimeError(f"{curve.name} is not a Curve object.")
    curve.data.resolution_u = max(curve.data.resolution_u, 32)
    for spline in curve.data.splines:
        if spline.type == "BEZIER":
            for point in spline.bezier_points:
                point.handle_left_type = "AUTO"
                point.handle_right_type = "AUTO"


def cubic_bezier_point(p0, h0, h1, p1, factor):
    inverse = 1.0 - factor
    return (p0 * inverse ** 3
            + h0 * (3.0 * inverse * inverse * factor)
            + h1 * (3.0 * inverse * factor * factor)
            + p1 * factor ** 3)


def drive_flex_empty(empty, end_a, midpoint, end_b, fraction):
    """Interpolate a distributed cable control without handlers."""
    weights = ((1.0 - 2.0 * fraction, 2.0 * fraction, 0.0)
               if fraction < 0.5
               else (0.0, 2.0 - 2.0 * fraction, 2.0 * fraction - 1.0))
    sources = (("a", end_a, weights[0]),
               ("m", midpoint, weights[1]),
               ("b", end_b, weights[2]))
    for axis, suffix in enumerate("XYZ"):
        try:
            empty.driver_remove("location", axis)
        except (TypeError, RuntimeError):
            pass
        fcurve = empty.driver_add("location", axis)
        driver = fcurve.driver
        driver.type = "SCRIPTED"
        terms = []
        for prefix, source, weight in sources:
            if abs(weight) < 1.0e-12:
                continue
            variable = prefix + suffix.lower()
            add_world_transform_variable(driver, variable, source, "LOC_" + suffix)
            terms.append(f"{weight:.9g}*{variable}")
        driver.expression = "+".join(terms)


def install_top_rope_flex_controls():
    """Repair and expand top curves with correctly centered Hook controls."""
    for (side, curve_name, end_a_name, midpoint_name, end_b_name,
         tangent_name, base_nodes) in TOP_ROPE_FLEX_CONFIGS:
        curve = bpy.data.objects.get(curve_name)
        end_a = bpy.data.objects.get(end_a_name)
        midpoint = bpy.data.objects.get(midpoint_name)
        end_b = bpy.data.objects.get(end_b_name)
        tangent = bpy.data.objects.get(tangent_name)
        missing = [name for name, obj in (
            (curve_name, curve), (end_a_name, end_a),
            (midpoint_name, midpoint), (end_b_name, end_b),
            (tangent_name, tangent)) if obj is None]
        if missing:
            raise RuntimeError("Missing top-rope object(s): " + ", ".join(missing))
        if curve.type != "CURVE":
            raise RuntimeError(f"{curve_name} is not a Curve object.")

        # Remove both the original hooks and the faulty v1 flex hooks before
        # reconstructing a clean, deterministic point-to-control mapping.
        for modifier in list(curve.modifiers):
            if modifier.type == "HOOK":
                curve.modifiers.remove(modifier)
        if curve.data.animation_data:
            for fcurve in list(curve.data.animation_data.drivers):
                if "bezier_points" in fcurve.data_path:
                    curve.data.animation_data.drivers.remove(fcurve)

        nodes = tuple(tuple(Vector(value) for value in node) for node in base_nodes)
        local_points = []
        for fraction in (0.0, 1.0 / 6.0, 1.0 / 3.0, 0.5,
                         2.0 / 3.0, 5.0 / 6.0, 1.0):
            if fraction <= 0.5:
                local_points.append(cubic_bezier_point(
                    nodes[0][0], nodes[0][2], nodes[1][1], nodes[1][0],
                    fraction * 2.0))
            else:
                local_points.append(cubic_bezier_point(
                    nodes[1][0], nodes[1][2], nodes[2][1], nodes[2][0],
                    fraction * 2.0 - 1.0))
        local_points.append(nodes[3][0].copy())

        collection = (midpoint.users_collection[0]
                      if midpoint.users_collection else bpy.context.scene.collection)
        anchors = (end_a.matrix_world.translation.copy(),
                   midpoint.matrix_world.translation.copy(),
                   end_b.matrix_world.translation.copy())
        flex_controls = []
        for number, fraction in enumerate(TOP_ROPE_FLEX_FRACTIONS, 1):
            name = f"CTRL_Rope Point_TR_{side}_flex_{number:02d}"
            empty = bpy.data.objects.get(name)
            if empty is None:
                empty = bpy.data.objects.new(name, None)
                collection.objects.link(empty)
            elif empty.type != "EMPTY":
                raise RuntimeError(f"{name} exists but is not an Empty.")
            for axis in range(3):
                try:
                    empty.driver_remove("location", axis)
                except (TypeError, RuntimeError):
                    pass
            empty.location = (anchors[0].lerp(anchors[1], fraction * 2.0)
                              if fraction < 0.5 else
                              anchors[1].lerp(anchors[2], fraction * 2.0 - 1.0))
            empty.empty_display_type = "SPHERE"
            empty.empty_display_size = 0.10
            empty.show_in_front = True
            empty.hide_render = True
            empty["Rope Flex Fraction"] = float(fraction)
            empty["Rope Flex Side"] = side
            drive_flex_empty(empty, end_a, midpoint, end_b, fraction)
            flex_controls.append(empty)

        curve.data.splines.clear()
        spline = curve.data.splines.new("BEZIER")
        spline.bezier_points.add(7)
        spline.resolution_u = 16
        spline.use_cyclic_u = False
        targets = (end_a, flex_controls[0], flex_controls[1], midpoint,
                   flex_controls[2], flex_controls[3], end_b, tangent)
        for index, (point, target) in enumerate(zip(spline.bezier_points, targets)):
            point.co = local_points[index]
            point.radius = 1.0
            point.tilt = 0.0
            point.handle_left_type = "AUTO"
            point.handle_right_type = "AUTO"

        for index, target in enumerate(targets):
            hook = curve.modifiers.new(
                name=f"LSO_TOP_ROPE_FLEX_{side}_{index:02d}", type="HOOK")
            hook.object = target
            hook.strength = 1.0
            hook.falloff_type = "NONE"
            # Hook center is required for Curve points. Omitting it makes the
            # modifier rotate/translate the point around the Curve origin,
            # which caused the giant loops in the previous build.
            hook.center = local_points[index]
            hook.vertex_indices_set((index * 3, index * 3 + 1, index * 3 + 2))
            hook.matrix_inverse = target.matrix_world.inverted() @ curve.matrix_world

        curve.data.resolution_u = 16
        curve["LSO_TOP_ROPE_FLEX_VERSION"] = TOP_ROPE_FLEX_VERSION
        curve["LSO_TOP_ROPE_VISIBLE_CONTROLS"] = 7
        curve.update_tag(refresh={"OBJECT", "DATA"})


def restore_original_top_rope_curves():
    """Remove the flex-point experiment and restore the captured 4-point rig."""
    original_hook_names = {
        "S": (
            "ROPE_SOUTH_TOP_CURVE_POINT_0_HOOK",
            "ROPE_SOUTH_TOP_CURVE_POINT_1_HOOK",
            "ROPE_SOUTH_TOP_CURVE_POINT_2_HOOK",
            "ROPE_SOUTH_TOP_CURVE_POINT_3_HOOK",
        ),
        "W": (
            "ROPE_WEST_TOP_CURVE_POINT_0_HOOK",
            "ROPE_WEST_TOP_CURVE_POINT_1_HOOK",
            "ROPE_WEST_TOP_CURVE_POINT_2_HOOK",
            "ROPE_WEST_TOP_CURVE_POINT_3_HOOK",
        ),
        "N": (
            "ROPE_NORTH_TOP_NEUTRAL_HOOK_01",
            "ROPE_NORTH_TOP_POINT_1_INNER_A_HOOK",
            "ROPE_NORTH_TOP_NEUTRAL_HOOK_03",
            "ROPE_NORTH_TOP_NEUTRAL_HOOK_04",
        ),
        "E": (
            "ROPE_EAST_TOP_CURVE_POINT_0_HOOK",
            "ROPE_EAST_TOP_CURVE_POINT_1_HOOK",
            "ROPE_EAST_TOP_CURVE_POINT_2_HOOK",
            "ROPE_EAST_TOP_CURVE_POINT_3_HOOK",
        ),
    }
    for (side, curve_name, end_a_name, midpoint_name, end_b_name,
         tangent_name, base_nodes) in TOP_ROPE_FLEX_CONFIGS:
        curve = bpy.data.objects.get(curve_name)
        targets = tuple(bpy.data.objects.get(name) for name in (
            end_a_name, midpoint_name, end_b_name, tangent_name))
        if curve is None or curve.type != "CURVE" or any(obj is None for obj in targets):
            raise RuntimeError(f"Cannot restore original top-rope curve: {curve_name}")

        for modifier in list(curve.modifiers):
            if modifier.type == "HOOK":
                curve.modifiers.remove(modifier)
        if curve.data.animation_data:
            for fcurve in list(curve.data.animation_data.drivers):
                if "bezier_points" in fcurve.data_path:
                    curve.data.animation_data.drivers.remove(fcurve)

        curve.data.splines.clear()
        spline = curve.data.splines.new("BEZIER")
        spline.bezier_points.add(3)
        spline.resolution_u = 32
        spline.use_cyclic_u = False
        local_points = []
        for point, node in zip(spline.bezier_points, base_nodes):
            co, handle_left, handle_right = (Vector(value) for value in node)
            point.co = co
            point.handle_left_type = "FREE"
            point.handle_right_type = "FREE"
            point.handle_left = handle_left
            point.handle_right = handle_right
            point.radius = 1.0
            point.tilt = 0.0
            point.handle_left_type = "AUTO"
            point.handle_right_type = "AUTO"
            local_points.append(co)

        for index, (target, hook_name) in enumerate(zip(
                targets, original_hook_names[side])):
            hook = curve.modifiers.new(name=hook_name, type="HOOK")
            hook.object = target
            hook.strength = 1.0
            hook.falloff_type = "NONE"
            hook.center = local_points[index]
            hook.vertex_indices_set((index * 3, index * 3 + 1, index * 3 + 2))
            hook.matrix_inverse = target.matrix_world.inverted() @ curve.matrix_world

        curve.data.resolution_u = 32
        for key in ("LSO_TOP_ROPE_FLEX_VERSION", "LSO_TOP_ROPE_VISIBLE_CONTROLS"):
            if key in curve:
                del curve[key]
        curve.update_tag(refresh={"OBJECT", "DATA"})

    # Delete only the sixteen objects created by the abandoned flex upgrade.
    for side in "SWNE":
        for number in range(1, 5):
            name = f"CTRL_Rope Point_TR_{side}_flex_{number:02d}"
            obj = bpy.data.objects.get(name)
            if obj is not None:
                bpy.data.objects.remove(obj, do_unlink=True)


def remove_cyclic_master_reach_constraints(control):
    """Remove v2/v3 constraints that fed the rope graph back into its source."""
    for config in SAG_CONFIGS:
        constraint = control.constraints.get("FIXED_ROPE_REACH_" + config["label"])
        if constraint is not None:
            control.constraints.remove(constraint)
    bpy.context.view_layer.update()


def remove_old_static_limits(control):
    for name in (
            "FIXED_ROPE_NONCYCLIC_TAUT_LIMITS_SW_TR",
            "FIXED_ROPE_ANALYTIC_REACH_SW_TR"):
        old = control.constraints.get(name)
        if old is not None:
            control.constraints.remove(old)


def drive_raw_location_bound(constraint, property_name, expression, control):
    """Drive a constraint bound from the empty's unevaluated input channels."""
    try:
        constraint.driver_remove(property_name)
    except (TypeError, RuntimeError):
        pass
    fcurve = constraint.driver_add(property_name)
    driver = fcurve.driver
    driver.type = "SCRIPTED"
    for axis, name in enumerate(("x", "y", "z")):
        add_single_property_variable(driver, name, control, f"location[{axis}]")
    driver.expression = f"({expression})"
    return fcurve


def install_analytic_fixed_reach(control):
    """Hard, timer-free rope reach using the intersection of two spheres."""
    name = "FIXED_ROPE_ANALYTIC_REACH_SW_TR"
    reach = control.constraints.get(name)
    if reach is None:
        reach = control.constraints.new("LIMIT_LOCATION")
        reach.name = name
    elif reach.type != "LIMIT_LOCATION":
        raise RuntimeError(f"{name} exists but is not Limit Location.")

    data = {}
    for config in SAG_CONFIGS:
        prefix = "s" if config["label"] == "SOUTH_TOP" else "w"
        data[prefix] = {
            "x": float(control[calibration_key(config, "CENTER_X")]),
            "y": float(control[calibration_key(config, "CENTER_Y")]),
            "z": float(control[calibration_key(config, "CENTER_Z")]),
            "r": float(control[calibration_key(config, "FIXED_LENGTH")]),
        }

    def span(prefix, axis_a, axis_b):
        sphere = data[prefix]
        return (
            f"sqrt(max(0.0,{sphere['r'] ** 2:.17g}"
            f"-({axis_a}-({sphere[axis_a]:.17g}))**2"
            f"-({axis_b}-({sphere[axis_b]:.17g}))**2))"
        )

    expressions = {}
    for axis, other_a, other_b in (
            ("x", "y", "z"), ("y", "x", "z"), ("z", "x", "y")):
        s_span = span("s", other_a, other_b)
        w_span = span("w", other_a, other_b)
        expressions["min_" + axis] = (
            f"max(({data['s'][axis]:.17g})-({s_span}),"
            f"({data['w'][axis]:.17g})-({w_span}))"
        )
        expressions["max_" + axis] = (
            f"min(({data['s'][axis]:.17g})+({s_span}),"
            f"({data['w'][axis]:.17g})+({w_span}))"
        )

    reach.owner_space = "WORLD"
    reach.use_transform_limit = True
    reach.mute = False
    reach.influence = 1.0
    curves = []
    for axis in "xyz":
        setattr(reach, "use_min_" + axis, True)
        setattr(reach, "use_max_" + axis, True)
        curves.append(drive_raw_location_bound(
            reach, "min_" + axis, expressions["min_" + axis], control))
        curves.append(drive_raw_location_bound(
            reach, "max_" + axis, expressions["max_" + axis], control))

    # Force Blender to rebuild the newly introduced driver relations now.
    for fcurve in curves:
        expression = fcurve.driver.expression
        fcurve.driver.expression = f"({expression})"
    control.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()


def projected_point_expressions(control):
    """Return four-pass sphere-projected XYZ expressions for a proxy object."""
    spheres = []
    for config in SAG_CONFIGS:
        center = tuple(float(control[calibration_key(config, "CENTER_" + axis)])
                       for axis in "XYZ")
        # A tiny safety inset absorbs float/evaluation noise without visibly
        # shortening the rope.
        radius = max(
            0.0,
            float(control[calibration_key(config, "TAUT_CHORD")]) + 0.0001,
        )
        spheres.append((center, radius))

    point = ("x", "y", "z")
    for center, radius in (spheres[0], spheres[1], spheres[0], spheres[1]):
        offsets = tuple(
            f"(({point[i]})-({center[i]:.17g}))" for i in range(3))
        distance = (
            f"sqrt(max(1.0e-24,({offsets[0]})**2+"
            f"({offsets[1]})**2+({offsets[2]})**2))"
        )
        factor = f"min(1.0,({radius:.17g})/({distance}))"
        point = tuple(
            f"({center[i]:.17g})+({offsets[i]})*({factor})"
            for i in range(3))
    return point


def install_clamped_pad_proxy(control, pad):
    """Use the existing Pad IK as the hard-length proxy; adds no new object."""
    base_keys = tuple(f"SW_TR_PAD_PROXY_BASE_{axis}" for axis in "XYZ")
    rest_keys = tuple(f"SW_TR_PAD_PROXY_CONTROL_REST_{axis}" for axis in "XYZ")
    for index in range(3):
        if base_keys[index] not in control:
            control[base_keys[index]] = float(pad.location[index])
        if rest_keys[index] not in control:
            control[rest_keys[index]] = float(control.location[index])

    projected = projected_point_expressions(control)
    curves = []
    for axis in range(3):
        try:
            pad.driver_remove("location", axis)
        except (TypeError, RuntimeError):
            pass
        curve = pad.driver_add("location", axis)
        driver = curve.driver
        driver.type = "SCRIPTED"
        for source_axis, variable_name in enumerate(("x", "y", "z")):
            add_single_property_variable(
                driver, variable_name, control, f"location[{source_axis}]")
        base = float(control[base_keys[axis]])
        rest = float(control[rest_keys[axis]])
        driver.expression = (
            f"({base:.17g})+(({projected[axis]})-({rest:.17g}))"
        )
        curves.append(curve)

    for curve in curves:
        expression = curve.driver.expression
        curve.driver.expression = f"({expression})"
    pad.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()


def install_clamped_pad_limits(control, pad):
    """Clamp the existing Pad IK to both fixed rope lengths without feedback."""
    for axis in range(3):
        try:
            pad.driver_remove("location", axis)
        except (TypeError, RuntimeError):
            pass

    base_keys = tuple(f"SW_TR_PAD_LIMIT_BASE_{axis}" for axis in "XYZ")
    rest_keys = tuple(f"SW_TR_PAD_LIMIT_CONTROL_REST_{axis}" for axis in "XYZ")
    for index in range(3):
        if base_keys[index] not in control:
            control[base_keys[index]] = float(PAD_IK_BASE_LOCATION[index])
        if rest_keys[index] not in control:
            control[rest_keys[index]] = float(control[
                ("SW_TR_TAUT_CONTROL_X", "SW_TR_TAUT_CONTROL_Y",
                 "SW_TR_TAUT_CONTROL_Z")[index]])

    base = tuple(float(control[key]) for key in base_keys)
    rest = tuple(float(control[key]) for key in rest_keys)
    spheres = []
    for config in SAG_CONFIGS:
        center = tuple(float(control[calibration_key(config, "CENTER_" + axis)])
                       for axis in "XYZ")
        radius = max(
            0.0,
            float(control[calibration_key(config, "TAUT_CHORD")]) + 0.0001,
        )
        pad_center = tuple(base[i] + center[i] - rest[i] for i in range(3))
        spheres.append((center, pad_center, radius))

    name = "FIXED_ROPE_PAD_PROXY_REACH_SW_TR"
    reach = pad.constraints.get(name)
    if reach is None:
        reach = pad.constraints.new("LIMIT_LOCATION")
        reach.name = name
    elif reach.type != "LIMIT_LOCATION":
        raise RuntimeError(f"{name} exists but is not Limit Location.")
    reach.owner_space = "WORLD"
    reach.use_transform_limit = True
    reach.influence = 1.0
    reach.mute = False

    variables = ("x", "y", "z")
    curves = []
    for axis in range(3):
        other = tuple(i for i in range(3) if i != axis)
        lower_parts = []
        upper_parts = []
        for center, pad_center, radius in spheres:
            radicand = (
                f"max(0,{radius * radius:.9g}"
                f"-({variables[other[0]]}-{center[other[0]]:.9g})**2"
                f"-({variables[other[1]]}-{center[other[1]]:.9g})**2)"
            )
            span = f"sqrt({radicand})"
            lower_parts.append(f"{pad_center[axis]:.9g}-{span}")
            upper_parts.append(f"{pad_center[axis]:.9g}+{span}")
        minimum = f"max({lower_parts[0]},{lower_parts[1]})"
        maximum = f"min({upper_parts[0]},{upper_parts[1]})"
        if max(len(minimum), len(maximum)) > 250:
            raise RuntimeError("Generated fixed-rope driver exceeds Blender's limit.")
        setattr(reach, "use_min_" + variables[axis], True)
        setattr(reach, "use_max_" + variables[axis], True)
        curves.append(drive_raw_location_bound(
            reach, "min_" + variables[axis], minimum, control))
        curves.append(drive_raw_location_bound(
            reach, "max_" + variables[axis], maximum, control))

    for curve in curves:
        expression = curve.driver.expression
        curve.driver.expression = f"({expression})"
    pad.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()


def install_fixed_length_pad_projection(control, pad):
    """Project requested motion into both rope spheres with persistent drivers."""
    old_limit = pad.constraints.get("FIXED_ROPE_PAD_PROXY_REACH_SW_TR")
    if old_limit is not None:
        pad.constraints.remove(old_limit)

    base_keys = tuple(f"SW_TR_PAD_LIMIT_BASE_{axis}" for axis in "XYZ")
    rest_keys = tuple(f"SW_TR_PAD_LIMIT_CONTROL_REST_{axis}" for axis in "XYZ")
    for index in range(3):
        if base_keys[index] not in control:
            control[base_keys[index]] = float(PAD_IK_BASE_LOCATION[index])
        if rest_keys[index] not in control:
            control[rest_keys[index]] = float(control[
                ("SW_TR_TAUT_CONTROL_X", "SW_TR_TAUT_CONTROL_Y",
                 "SW_TR_TAUT_CONTROL_Z")[index]])
    base = tuple(float(control[key]) for key in base_keys)
    rest = tuple(float(control[key]) for key in rest_keys)

    sphere_data = []
    for config in SAG_CONFIGS:
        center = tuple(float(control[calibration_key(config, "CENTER_" + axis)])
                       for axis in "XYZ")
        radius = float(control[calibration_key(config, "TAUT_CHORD")])
        sphere_data.append((center, radius))
    south_center, south_radius = sphere_data[0]
    west_center, west_radius = sphere_data[1]

    scene = bpy.context.scene
    south_keys = tuple(f"_LSO_SW_TR_SOUTH_PROJECTED_{axis}" for axis in "XYZ")
    raw_names = ("x", "y", "z")
    # A real rope has one fixed three-dimensional length. Height therefore
    # consumes some of the same available length as horizontal reach: raising
    # the corner must pull it inward (and redistribute the other controls),
    # never extend the rope mesh. Project XYZ through both fixed-length spheres.
    projection_names = raw_names
    south_offsets = tuple(
        f"({projection_names[i]}-{south_center[i]:.9g})" for i in range(3))
    south_distance = (
        f"sqrt(max(1e-12,{south_offsets[0]}**2+"
        f"{south_offsets[1]}**2+{south_offsets[2]}**2))"
    )
    south_factor = f"min(1,{south_radius:.9g}/({south_distance}))"
    for axis, key in enumerate(south_keys):
        data_path = f'["{key}"]'
        try:
            scene.driver_remove(data_path)
        except (TypeError, RuntimeError):
            pass
        if key not in scene:
            scene[key] = float(control.location[axis])
        curve = scene.driver_add(data_path)
        driver = curve.driver
        driver.type = "SCRIPTED"
        for source_axis, variable_name in enumerate(raw_names):
            add_single_property_variable(
                driver, variable_name, control, f"location[{source_axis}]")
        driver.expression = (
            f"{south_center[axis]:.9g}+{south_offsets[axis]}*({south_factor})"
        )
        if len(driver.expression) > 250:
            raise RuntimeError("South fixed-rope driver exceeds Blender's limit.")

    stage_names = ("a", "b", "c")
    west_offsets = tuple(
        f"({stage_names[i]}-{west_center[i]:.9g})" for i in range(3))
    west_distance = (
        f"sqrt(max(1e-12,{west_offsets[0]}**2+"
        f"{west_offsets[1]}**2+{west_offsets[2]}**2))"
    )
    west_factor = f"min(1,{west_radius:.9g}/({west_distance}))"
    curves = []
    for axis in range(3):
        try:
            pad.driver_remove("location", axis)
        except (TypeError, RuntimeError):
            pass
        curve = pad.driver_add("location", axis)
        driver = curve.driver
        driver.type = "SCRIPTED"
        for name, key in zip(stage_names, south_keys):
            add_single_property_variable(driver, name, scene, f'["{key}"]')
        projected = (
            f"{west_center[axis]:.9g}+{west_offsets[axis]}*({west_factor})"
        )
        driver.expression = (
            f"{base[axis]:.9g}+(({projected})-{rest[axis]:.9g})"
        )
        if len(driver.expression) > 250:
            raise RuntimeError("West fixed-rope driver exceeds Blender's limit.")
        curves.append(curve)

    for curve in curves:
        expression = curve.driver.expression
        curve.driver.expression = f"({expression})"
    scene.update_tag()
    pad.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()


def make_root_follow_clamped_pad(root, pad):
    """Route the whole rigid unit through the existing clamped Pad IK."""
    existing = root.constraints.get(ROOT_CONSTRAINT_NAME)
    if existing is not None and existing.type != "CHILD_OF":
        root.constraints.remove(existing)
        existing = None
        bpy.context.view_layer.update()
    if existing is None:
        existing = root.constraints.new("CHILD_OF")
        existing.name = ROOT_CONSTRAINT_NAME
        existing.target = pad
        existing.subtarget = ""
        existing.target_space = "WORLD"
        existing.owner_space = "WORLD"
        existing.inverse_matrix = pad.matrix_world.inverted()

    existing.target = pad
    existing.subtarget = ""
    existing.target_space = "WORLD"
    existing.owner_space = "WORLD"
    existing.use_location_x = existing.use_location_y = existing.use_location_z = True
    existing.use_rotation_x = existing.use_rotation_y = existing.use_rotation_z = True
    existing.use_scale_x = existing.use_scale_y = existing.use_scale_z = True
    existing.influence = 1.0
    existing.mute = False


def remove_self_transform_drivers(control):
    """Clean up the rejected v6 experiment before recalibrating."""
    for axis in range(3):
        try:
            control.driver_remove("delta_location", axis)
        except (TypeError, RuntimeError):
            pass
    control.delta_location = (0.0, 0.0, 0.0)


_reach_guard = False


def project_master_inside_rope_lengths(control):
    """Alternating sphere projection; exact, tiny, and dependency-cycle free."""
    if control.parent is not None:
        raise RuntimeError(f"{control.name} must remain unparented for fixed reach.")

    point = Vector(control.location)
    original = point.copy()
    spheres = []
    for config in SAG_CONFIGS:
        center = Vector((
            float(control[calibration_key(config, "CENTER_X")]),
            float(control[calibration_key(config, "CENTER_Y")]),
            float(control[calibration_key(config, "CENTER_Z")]),
        ))
        radius = float(control[calibration_key(config, "FIXED_LENGTH")])
        spheres.append((center, radius))

    # Alternating projection converges quickly for these two perpendicular ring
    # spans. Eight passes leaves sub-micrometer error in normal control ranges.
    for _ in range(8):
        for center, radius in spheres:
            vector = point - center
            distance = vector.length
            if distance > radius and distance > 1.0e-12:
                point = center + vector * (radius / distance)

    if (point - original).length > 1.0e-9:
        control.location = point


def restore_recoil_bases(state):
    if not state:
        return
    for channel in state.get("channels", {}).values():
        pad = bpy.data.objects.get(channel["object"])
        if pad is not None:
            pad.delta_location = channel["base_delta"]
    for midpoint in state.get("midpoints", {}).values():
        constraint = midpoint.get("constraint")
        if constraint is not None:
            constraint.mute = True
        midpoint["base_world"] = None


def recoil_timer_tick():
    state = bpy.app.driver_namespace.get(RECOIL_STATE_KEY)
    if not state:
        return None
    control = bpy.data.objects.get(EMPTY_NAME)
    if control is None:
        restore_recoil_bases(state)
        return None

    now = time.perf_counter()
    dt = min(1.0 / 30.0, max(1.0 / 240.0, now - state["last_tick"]))
    state["last_tick"] = now
    damping = max(2.0, min(20.0, float(control.get(RECOIL_DAMPING_PROPERTY, 9.0))))
    spring = 105.0
    active = False

    for channel in state["channels"].values():
        remaining = []
        for event_time, impulse in channel["events"]:
            if event_time <= now:
                channel["velocity"] += impulse
            else:
                remaining.append((event_time, impulse))
        channel["events"] = remaining

        acceleration = -spring * channel["position"] - damping * channel["velocity"]
        channel["velocity"] += acceleration * dt
        channel["position"] += channel["velocity"] * dt
        maximum = channel["maximum"]
        channel["position"] = max(-maximum, min(maximum, channel["position"]))

        pad = bpy.data.objects.get(channel["object"])
        if pad is not None:
            pad.delta_location = (
                channel["base_delta"] + channel["direction"] * channel["position"])

        if (channel["events"] or abs(channel["position"]) > 1.0e-5
                or abs(channel["velocity"]) > 1.0e-4):
            active = True

    if active:
        for midpoint in state["midpoints"].values():
            obj = bpy.data.objects.get(midpoint["object"])
            constraint = midpoint["constraint"]
            if obj is None or constraint is None:
                continue
            if midpoint["base_world"] is None:
                midpoint["base_world"] = obj.matrix_world.translation.copy()
            amount = sum(
                state["channels"][key]["position"] * weight
                for key, weight in midpoint["sources"])
            position = midpoint["base_world"] + midpoint["direction"] * amount
            constraint.min_x = constraint.max_x = position.x
            constraint.min_y = constraint.max_y = position.y
            constraint.min_z = constraint.max_z = position.z
            constraint.mute = False

    if active:
        return 1.0 / 60.0

    restore_recoil_bases(state)
    state["timer_running"] = False
    return None


def inject_x_recoil(control):
    state = bpy.app.driver_namespace.get(RECOIL_STATE_KEY)
    if not state:
        return
    current_x = float(control.location.x)
    delta_x = current_x - state["last_control_x"]
    state["last_control_x"] = current_x
    if abs(delta_x) < 1.0e-6:
        return

    now = time.perf_counter()
    strength = max(0.0, min(2.0, float(control.get(RECOIL_STRENGTH_PROPERTY, 1.0))))
    for key, config in RECOIL_TARGETS.items():
        channel = state["channels"][key]
        impulse = max(-0.80, min(0.80, delta_x * 28.0 * config["gain"] * strength))
        channel["events"].append((now + config["delay"], impulse))

    if not state["timer_running"]:
        state["last_tick"] = now
        state["timer_running"] = True
        if not bpy.app.timers.is_registered(recoil_timer_tick):
            bpy.app.timers.register(recoil_timer_tick, first_interval=1.0 / 60.0)


def install_top_rope_recoil(control):
    old_timer = bpy.app.driver_namespace.get(RECOIL_TIMER_KEY)
    if old_timer is not None and bpy.app.timers.is_registered(old_timer):
        bpy.app.timers.unregister(old_timer)
    restore_recoil_bases(bpy.app.driver_namespace.get(RECOIL_STATE_KEY))

    if RECOIL_STRENGTH_PROPERTY not in control:
        control[RECOIL_STRENGTH_PROPERTY] = 1.0
    control.id_properties_ui(RECOIL_STRENGTH_PROPERTY).update(
        min=0.0, max=2.0, soft_min=0.0, soft_max=1.5)
    if RECOIL_DAMPING_PROPERTY not in control:
        control[RECOIL_DAMPING_PROPERTY] = 9.0
    control.id_properties_ui(RECOIL_DAMPING_PROPERTY).update(
        min=2.0, max=20.0, soft_min=5.0, soft_max=14.0)

    channels = {}
    for key, config in RECOIL_TARGETS.items():
        pad = bpy.data.objects.get(config["object"])
        if pad is None:
            raise RuntimeError(f"Recoil pad control not found: {config['object']}")
        animation = pad.animation_data
        if animation and any(fc.data_path == "delta_location" for fc in animation.drivers):
            raise RuntimeError(
                f"Cannot add recoil: {pad.name} already has delta-location drivers.")
        channels[key] = {
            "object": pad.name,
            "base_delta": pad.delta_location.copy(),
            "direction": Vector(config["direction"]).normalized(),
            "gain": config["gain"],
            "maximum": config["maximum"],
            "position": 0.0,
            "velocity": 0.0,
            "events": [],
        }

    midpoints = {}
    for key, config in RECOIL_MIDPOINTS.items():
        obj = bpy.data.objects.get(config["object"])
        if obj is None:
            raise RuntimeError(f"Recoil midpoint not found: {config['object']}")
        constraint = obj.constraints.get(RECOIL_MIDPOINT_CONSTRAINT)
        if constraint is None:
            constraint = obj.constraints.new("LIMIT_LOCATION")
            constraint.name = RECOIL_MIDPOINT_CONSTRAINT
        elif constraint.type != "LIMIT_LOCATION":
            raise RuntimeError(
                f"{obj.name}/{RECOIL_MIDPOINT_CONSTRAINT} is not Limit Location.")
        constraint.owner_space = "WORLD"
        constraint.use_transform_limit = False
        constraint.use_min_x = constraint.use_max_x = True
        constraint.use_min_y = constraint.use_max_y = True
        constraint.use_min_z = constraint.use_max_z = True
        constraint.influence = 1.0
        constraint.mute = True
        midpoints[key] = {
            "object": obj.name,
            "constraint": constraint,
            "direction": Vector(config["direction"]).normalized(),
            "sources": config["sources"],
            "base_world": None,
        }

    state = {
        "last_control_x": float(control.location.x),
        "last_tick": time.perf_counter(),
        "timer_running": False,
        "channels": channels,
        "midpoints": midpoints,
    }
    bpy.app.driver_namespace[RECOIL_STATE_KEY] = state
    bpy.app.driver_namespace[RECOIL_TIMER_KEY] = recoil_timer_tick


def disable_runtime_recoil_callbacks():
    """Remove the callback design that can re-enter Blender's dependency graph."""
    old_state = bpy.app.driver_namespace.get(RECOIL_STATE_KEY)
    restore_recoil_bases(old_state)
    old_timer = bpy.app.driver_namespace.get(RECOIL_TIMER_KEY)
    if old_timer is not None and bpy.app.timers.is_registered(old_timer):
        bpy.app.timers.unregister(old_timer)
    old_handler = bpy.app.driver_namespace.get(FIXED_REACH_HANDLER_KEY)
    if old_handler is not None:
        while old_handler in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(old_handler)
    bpy.app.driver_namespace.pop(RECOIL_STATE_KEY, None)
    bpy.app.driver_namespace.pop(RECOIL_TIMER_KEY, None)
    bpy.app.driver_namespace.pop(FIXED_REACH_HANDLER_KEY, None)


def add_single_property_variable(driver, name, obj, data_path):
    variable = driver.variables.new()
    variable.name = name
    variable.type = "SINGLE_PROP"
    target = variable.targets[0]
    target.id_type = obj.id_type
    target.id = obj
    target.data_path = data_path


def configure_stable_driver(driver, control):
    driver.type = "SCRIPTED"
    # Read the master's raw channels as properties instead of evaluated world
    # transforms.  The latter asks Blender to build a transform dependency
    # through the already closed rope/armature graph, which can leave these
    # drivers unevaluated (or create a dependency cycle).  This master has no
    # parent, so its location channels are its world translation.
    for axis, name in enumerate(("mx", "my", "mz")):
        add_single_property_variable(
            driver, name, control, f"location[{axis}]")
    add_single_property_variable(
        driver, "follow", control, f'["{BROKEN_FOLLOW_PROPERTY}"]')
    add_single_property_variable(
        driver, "maximum", control, f'["{BROKEN_MAX_PULL_PROPERTY}"]')
    add_single_property_variable(
        driver, "wiggle", control, f'["{BROKEN_WIGGLE_PROPERTY}"]')


def drive_stable_property(owner, data_path, array_index, expression, control):
    try:
        owner.driver_remove(data_path, array_index)
    except (TypeError, RuntimeError):
        pass
    fcurve = owner.driver_add(data_path, array_index)
    driver = fcurve.driver
    configure_stable_driver(driver, control)
    driver.expression = expression
    return fcurve


def install_stable_broken_weapon_tether(control):
    """One-way driver rig: no timers, handlers, or graph feedback."""
    created_driver_curves = []
    # Weapon use is the reason for making the relationship robust, not a
    # separate rig state.  The rope/hardware response is therefore always on.
    if LEGACY_BROKEN_MODE_PROPERTY in control:
        del control[LEGACY_BROKEN_MODE_PROPERTY]
    defaults = (
        (BROKEN_FOLLOW_PROPERTY, 0.65, 0.0, 1.0),
        (BROKEN_MAX_PULL_PROPERTY, 1.50, 0.01, 2.50),
        (BROKEN_WIGGLE_PROPERTY, 0.020, 0.0, 0.08),
        ("Positive X North Mid X", 0.0, -2.50, 2.50),
        ("Positive X North Mid Y", 0.0, -2.50, 2.50),
        ("Positive X North Mid Z", 0.0, -2.50, 2.50),
        ("Positive X East Mid X", 0.0, -2.50, 2.50),
        ("Positive X East Mid Y", 0.0, -2.50, 2.50),
        ("Positive X East Mid Z", 0.0, -2.50, 2.50),
    )
    for name, default, minimum, maximum in defaults:
        if name not in control:
            control[name] = default
        control.id_properties_ui(name).update(
            min=minimum, max=maximum, soft_min=minimum, soft_max=maximum)
    control[CORNER_RESPONSE_VERSION_PROPERTY] = CORNER_RESPONSE_VERSION

    rest = (
        float(control["SW_TR_TAUT_CONTROL_X"]),
        float(control["SW_TR_TAUT_CONTROL_Y"]),
        float(control["SW_TR_TAUT_CONTROL_Z"]),
    )
    dx = f"(mx-({rest[0]:.17g}))"
    dy = f"(my-({rest[1]:.17g}))"
    dz = f"(mz-({rest[2]:.17g}))"
    south_tension = f"max(0.0,-({dx}))"
    west_tension = f"max(0.0,({dy}))"
    south_slack = f"max(0.0,({dx}))"
    west_slack = f"max(0.0,-({dy}))"
    local_z = control.matrix_world.to_3x3() @ Vector((0.0, 0.0, 1.0))
    local_z.normalize()
    local_x = control.matrix_world.to_3x3() @ Vector((1.0, 0.0, 0.0))
    local_x.normalize()
    # Project motion onto the master's actual local Z axis. This is the
    # inward/outward rope-tension channel: positive is inward/slack, while
    # local X remains the independent vertical movement channel.
    # Keep this expression compact: Blender truncates driver expressions at
    # 255 characters. Local Z is horizontal here, so X/Y are the stable and
    # exact tension inputs while local-X vertical travel remains independent.
    inward_slack = (
        f"max(0,(mx{(-rest[0]):+.8g})*{local_z.x:.8g}+"
        f"(my{(-rest[1]):+.8g})*{local_z.y:.8g})"
    )

    # Only an outward pull creates tension at the remaining posts. Inward
    # movement creates slack and must not drag all three posts diagonally.
    # Remaining hardware has only a small, capped compliance; the poles stay
    # anchored while the rope absorbs nearly all of the motion.
    pad_responses = {
        "CTRL_Turnbuckle_Pad_TR_S_E_IK": (
            f"-follow*0.10*({south_tension})",
            "0.0",
            f"follow*0.70*({dz})",
        ),
        "CTRL_Turnbuckle_Pad_TR_N_W_IK": (
            "0.0",
            f"follow*0.10*({west_tension})",
            f"follow*0.70*({dz})",
        ),
        "CTRL_Turnbuckle_Pad_TR_N_E_IK": (
            f"-follow*0.03*({south_tension})",
            f"follow*0.03*({west_tension})",
            f"follow*0.40*({dz})",
        ),
    }

    for pad_name, axis_responses in pad_responses.items():
        pad = bpy.data.objects.get(pad_name)
        if pad is None:
            raise RuntimeError(f"Broken-rope corner control not found: {pad_name}")
        # All three captured controls have a zero delta baseline. Recording it
        # keeps the setup idempotent if the user intentionally changes one.
        for axis in range(3):
            key = f"_SW_BROKEN_BASE_DELTA_{axis}"
            if key not in pad:
                pad[key] = float(pad.delta_location[axis])
            base = float(pad[key])
            response = (
                f"max(-maximum,min(maximum,({axis_responses[axis]})))"
            )
            expression = f"({base:.17g})+({response})"
            created_driver_curves.append(drive_stable_property(
                pad, "delta_location", axis, expression, control))

    # Keep the far north/east spans visually taut when the detached corner is
    # lifted.  The former 35% response raised their midpoints by more than
    # 11 cm during a 50 cm lift, visibly bowing both ropes upward.  Downward
    # travel retains that response so the established floor/sag behavior is
    # unchanged; upward travel is gentler and capped at four centimetres.
    short_dz = f"mz{(-rest[2]):+.6g}"
    compact_slack = (
        f"max(0,(mx{(-rest[0]):+.6g})*{local_z.x:.6g}+"
        f"(my{(-rest[1]):+.6g})*{local_z.y:.6g})"
    )
    # This master's local X is aligned to world Z within 0.002 degrees. Using
    # the compact world-Z form keeps every Blender driver safely below its
    # 255-character expression limit while remaining visually exact.
    positive_x_weight = f"min(1,max(0,{short_dz})*4)"
    downward_mid_follow = f"follow*.35*min(0,{short_dz})"
    upward_mid_follow = f"min(.04,follow*.1*max(0,{short_dz}))"
    stable_mid_height = (
        f"{downward_mid_follow}+{upward_mid_follow}"
        f"-min(maximum,follow*.5*({compact_slack}))"
    )
    midpoint_configs = {
        "CTRL_Rope Point_TR_N_mid": (
            (0.0, -2.6025493144989014, 2.5241496562957764),
            stable_mid_height,
        ),
        "CTRL_Rope Point_TR_E_mid": (
            (2.6086108684539795, 0.0, 2.526263475418091),
            stable_mid_height,
        ),
    }
    for midpoint_name, (base_position, vertical_response) in midpoint_configs.items():
        midpoint = bpy.data.objects.get(midpoint_name)
        if midpoint is None:
            raise RuntimeError(f"Broken-rope midpoint not found: {midpoint_name}")
        constraint = midpoint.constraints.get(RECOIL_MIDPOINT_CONSTRAINT)
        if constraint is None:
            constraint = midpoint.constraints.new("LIMIT_LOCATION")
            constraint.name = RECOIL_MIDPOINT_CONSTRAINT
        elif constraint.type != "LIMIT_LOCATION":
            raise RuntimeError(
                f"{midpoint.name}/{RECOIL_MIDPOINT_CONSTRAINT} has wrong type.")
        base_world = Vector(base_position)
        constraint.owner_space = "WORLD"
        constraint.use_transform_limit = False
        constraint.use_min_x = constraint.use_max_x = True
        constraint.use_min_y = constraint.use_max_y = True
        constraint.use_min_z = constraint.use_max_z = True
        constraint.mute = False

        for axis, (minimum_name, maximum_name) in enumerate((
                ("min_x", "max_x"), ("min_y", "max_y"), ("min_z", "max_z"))):
            tuning_property = POSITIVE_X_MIDPOINT_TUNING[midpoint_name][axis]
            automatic = vertical_response if axis == 2 else "0"
            response = f"({automatic})+({positive_x_weight})*tune"
            expression = f"({base_world[axis]:.17g})+({response})"
            for property_name in (minimum_name, maximum_name):
                fcurve = drive_stable_property(
                    constraint, property_name, -1, expression, control)
                variable = fcurve.driver.variables.new()
                variable.name = "tune"
                variable.type = "SINGLE_PROP"
                target = variable.targets[0]
                target.id = control
                target.data_path = f'["{tuning_property}"]'
                created_driver_curves.append(fcurve)

        try:
            constraint.driver_remove("influence")
        except (TypeError, RuntimeError):
            pass
        constraint.influence = 1.0

    # Blender may defer rebuilding newly introduced driver relations until the
    # file is reopened.  Re-parsing each new expression dirties those relations
    # now, so the corner response works immediately after running this script.
    for fcurve in created_driver_curves:
        expression = fcurve.driver.expression
        fcurve.driver.expression = f"({expression})"
    control.update_tag(refresh={"OBJECT"})
    for pad_name in BROKEN_CORNER_FOLLOW:
        bpy.data.objects[pad_name].update_tag(refresh={"OBJECT"})
    for midpoint_name in midpoint_configs:
        bpy.data.objects[midpoint_name].update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()


def install_upward_tangent_alignment(control):
    """Blend far-end tangent helpers onto each rope's live straight angle."""
    rest = Vector((
        float(control["SW_TR_TAUT_CONTROL_X"]),
        float(control["SW_TR_TAUT_CONTROL_Y"]),
        float(control["SW_TR_TAUT_CONTROL_Z"]),
    ))
    local_x = control.matrix_world.to_3x3() @ Vector((1.0, 0.0, 0.0))
    local_x.normalize()
    lift = (
        f"(mx{(-rest.x):+.8g})*{local_x.x:.8g}+"
        f"(my{(-rest.y):+.8g})*{local_x.y:.8g}+"
        f"(mz{(-rest.z):+.8g})*{local_x.z:.8g}"
    )
    # Preserve the captured tangent at rest. Over the first ten centimetres of
    # upward travel, smoothly blend to a perfectly straight continuation.
    influence_expression = f"min(1,max(0,({lift})/0.10))"

    for config in UPWARD_TANGENT_CONFIGS:
        midpoint = bpy.data.objects.get(config["midpoint"])
        endpoint = bpy.data.objects.get(config["endpoint"])
        tangent = bpy.data.objects.get(config["tangent"])
        missing = [name for name, obj in (
            (config["midpoint"], midpoint),
            (config["endpoint"], endpoint),
            (config["tangent"], tangent),
        ) if obj is None]
        if missing:
            raise RuntimeError(
                "Missing upward-tangent control(s): " + ", ".join(missing))

        constraint = tangent.constraints.get(UPWARD_TANGENT_CONSTRAINT)
        if constraint is None:
            constraint = tangent.constraints.new("LIMIT_LOCATION")
            constraint.name = UPWARD_TANGENT_CONSTRAINT
        elif constraint.type != "LIMIT_LOCATION":
            raise RuntimeError(
                f"{tangent.name}/{UPWARD_TANGENT_CONSTRAINT} has wrong type.")
        constraint.owner_space = "WORLD"
        constraint.use_transform_limit = False
        constraint.use_min_x = constraint.use_max_x = True
        constraint.use_min_y = constraint.use_max_y = True
        constraint.use_min_z = constraint.use_max_z = True
        constraint.mute = False

        length = float(config["length"])
        distance = "max(.0001,sqrt((bx-ax)**2+(by-ay)**2+(bz-az)**2))"
        for suffix, minimum_name, maximum_name in (
                ("x", "min_x", "max_x"),
                ("y", "min_y", "max_y"),
                ("z", "min_z", "max_z")):
            expression = (
                f"b{suffix}+(b{suffix}-a{suffix})*{length:.10g}/({distance})"
            )
            drive_constraint_value(
                constraint, minimum_name, expression, midpoint, endpoint)
            drive_constraint_value(
                constraint, maximum_name, expression, midpoint, endpoint)

        try:
            constraint.driver_remove("influence")
        except (TypeError, RuntimeError):
            pass
        fcurve = constraint.driver_add("influence")
        driver = fcurve.driver
        configure_stable_driver(driver, control)
        driver.expression = influence_expression
        tangent.update_tag(refresh={"OBJECT"})

    bpy.context.view_layer.update()


def install_upward_midpoint_tilt(control):
    """Let north/east midpoint handles share an upward endpoint angle."""
    rest = Vector((
        float(control["SW_TR_TAUT_CONTROL_X"]),
        float(control["SW_TR_TAUT_CONTROL_Y"]),
        float(control["SW_TR_TAUT_CONTROL_Z"]),
    ))
    local_x = control.matrix_world.to_3x3() @ Vector((1.0, 0.0, 0.0))
    local_x.normalize()
    lift = (
        f"(mx{(-rest.x):+.8g})*{local_x.x:.8g}+"
        f"(my{(-rest.y):+.8g})*{local_x.y:.8g}+"
        f"(mz{(-rest.z):+.8g})*{local_x.z:.8g}"
    )
    # No baseline rotation change. Reach 65% tracking over the first 25 cm;
    # this distributes the bend while avoiding a rigid, mechanical aim.
    influence_expression = f"min(0.65,max(0,({lift})*2.6))"

    for midpoint_name, endpoint_name in UPWARD_MIDPOINT_TILT_CONFIGS:
        midpoint = bpy.data.objects.get(midpoint_name)
        endpoint = bpy.data.objects.get(endpoint_name)
        if midpoint is None or endpoint is None:
            raise RuntimeError(
                f"Missing midpoint tilt control: {midpoint_name} / {endpoint_name}")
        constraint = midpoint.constraints.get(UPWARD_MIDPOINT_TILT_CONSTRAINT)
        if constraint is None:
            constraint = midpoint.constraints.new("DAMPED_TRACK")
            constraint.name = UPWARD_MIDPOINT_TILT_CONSTRAINT
        elif constraint.type != "DAMPED_TRACK":
            raise RuntimeError(
                f"{midpoint.name}/{UPWARD_MIDPOINT_TILT_CONSTRAINT} has wrong type.")
        constraint.target = endpoint
        constraint.subtarget = ""
        constraint.track_axis = "TRACK_Y"
        constraint.mute = False
        try:
            constraint.driver_remove("influence")
        except (TypeError, RuntimeError):
            pass
        fcurve = constraint.driver_add("influence")
        driver = fcurve.driver
        configure_stable_driver(driver, control)
        driver.expression = influence_expression
        midpoint.update_tag(refresh={"OBJECT"})

    bpy.context.view_layer.update()


def fixed_reach_depsgraph_handler(_scene, _depsgraph):
    global _reach_guard
    if _reach_guard:
        return
    control = bpy.data.objects.get(EMPTY_NAME)
    if control is None or int(control.get(ROPE_CALIBRATION_PROPERTY, 0)) != ROPE_CALIBRATION_VERSION:
        return
    _reach_guard = True
    try:
        project_master_inside_rope_lengths(control)
        inject_x_recoil(control)
    finally:
        _reach_guard = False


def install_fixed_reach_handler(control):
    old = bpy.app.driver_namespace.get(FIXED_REACH_HANDLER_KEY)
    if old is not None:
        while old in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(old)
    bpy.app.driver_namespace[FIXED_REACH_HANDLER_KEY] = fixed_reach_depsgraph_handler
    if fixed_reach_depsgraph_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(fixed_reach_depsgraph_handler)
    project_master_inside_rope_lengths(control)
COLLECTION_NAME = "CTRL - Turnbuckle Units"
EMPTY_DISPLAY_SIZE = 0.75


armature = bpy.data.objects.get(ARMATURE_NAME)
if armature is None or armature.type != "ARMATURE":
    raise RuntimeError(f"Armature not found: {ARMATURE_NAME}")

root_bone = armature.pose.bones.get(ROOT_BONE_NAME)
if root_bone is None:
    raise RuntimeError(f"Pose bone not found: {ARMATURE_NAME} / {ROOT_BONE_NAME}")

old_controlled_bone = armature.pose.bones.get(OLD_CONTROLLED_BONE_NAME)
if old_controlled_bone is None:
    raise RuntimeError(
        f"Pose bone not found: {ARMATURE_NAME} / {OLD_CONTROLLED_BONE_NAME}")

foam_bone = armature.pose.bones.get(FOAM_BONE_NAME)
if foam_bone is None:
    raise RuntimeError(f"Pose bone not found: {ARMATURE_NAME} / {FOAM_BONE_NAME}")

body_bone = armature.pose.bones.get(BODY_BONE_NAME)
if body_bone is None:
    raise RuntimeError(f"Pose bone not found: {ARMATURE_NAME} / {BODY_BONE_NAME}")

pad_ik = bpy.data.objects.get(PAD_IK_NAME)
if pad_ik is None:
    raise RuntimeError(f"Object not found: {PAD_IK_NAME}")

rope_eyehook = bpy.data.objects.get(ROPE_EYEHOOK_NAME)
if rope_eyehook is None:
    raise RuntimeError(f"Object not found: {ROPE_EYEHOOK_NAME}")

rope_endpoints = []
for endpoint_name, constraint_name in ROPE_ENDPOINT_LINKS:
    endpoint = bpy.data.objects.get(endpoint_name)
    if endpoint is None:
        raise RuntimeError(f"Object not found: {endpoint_name}")
    rope_endpoints.append((endpoint, constraint_name))

west_tangent = bpy.data.objects.get(WEST_TANGENT_NAME)
if west_tangent is None:
    raise RuntimeError(f"Object not found: {WEST_TANGENT_NAME}")

collection = bpy.data.collections.get(COLLECTION_NAME)
if collection is None:
    collection = bpy.data.collections.new(COLLECTION_NAME)
    bpy.context.scene.collection.children.link(collection)

control = bpy.data.objects.get(EMPTY_NAME)
if control is None:
    control = bpy.data.objects.new(EMPTY_NAME, None)
    collection.objects.link(control)
    control.matrix_world = armature.matrix_world @ root_bone.matrix
elif control.type != "EMPTY":
    raise RuntimeError(f"{EMPTY_NAME} exists but is not an empty.")

control.empty_display_type = "CUBE"
control.empty_display_size = EMPTY_DISPLAY_SIZE
control.show_name = True
control.show_in_front = True

# Remove the preceding experimental link that controlled only the jaw-post.
# It is the source of translation causing internal rotation and deformation.
old_constraint = old_controlled_bone.constraints.get(OLD_CONSTRAINT_NAME)
if old_constraint is not None:
    old_controlled_bone.constraints.remove(old_constraint)
    bpy.context.view_layer.update()

# If this is the first run of the rigid-root version, place the existing empty
# on the evaluated unit root before attaching it. This produces zero jump.
constraint = root_bone.constraints.get(ROOT_CONSTRAINT_NAME)
if constraint is None:
    control.matrix_world = armature.matrix_world @ root_bone.matrix
    constraint = root_bone.constraints.new("COPY_TRANSFORMS")
    constraint.name = ROOT_CONSTRAINT_NAME
elif constraint.type not in {"COPY_TRANSFORMS", "CHILD_OF"}:
    raise RuntimeError(f"{ROOT_CONSTRAINT_NAME} exists but is not Copy Transforms.")

if constraint.type == "COPY_TRANSFORMS":
    constraint.target = control
    constraint.subtarget = ""
    constraint.target_space = "WORLD"
    constraint.owner_space = "WORLD"
    if hasattr(constraint, "mix_mode"):
        constraint.mix_mode = "REPLACE"
    constraint.influence = 1.0
    constraint.mute = False

# Move the external Pad IK by the identical master delta. Otherwise the pad
# target stays behind and the armature correctly tries to stretch/rotate toward
# it, which looks like unwanted deformation during a simple translation.
pad_follow = pad_ik.constraints.get(PAD_FOLLOW_CONSTRAINT_NAME)
if pad_follow is None:
    pad_follow = pad_ik.constraints.new("CHILD_OF")
    pad_follow.name = PAD_FOLLOW_CONSTRAINT_NAME
    pad_follow.target = control
    pad_follow.subtarget = ""
    pad_follow.target_space = "WORLD"
    pad_follow.owner_space = "WORLD"
    pad_follow.inverse_matrix = control.matrix_world.inverted()
elif pad_follow.type != "CHILD_OF":
    raise RuntimeError(f"{PAD_FOLLOW_CONSTRAINT_NAME} exists but is not Child Of.")

pad_follow.target = control
pad_follow.subtarget = ""
pad_follow.target_space = "WORLD"
pad_follow.owner_space = "WORLD"
pad_follow.use_location_x = True
pad_follow.use_location_y = True
pad_follow.use_location_z = True
pad_follow.use_rotation_x = True
pad_follow.use_rotation_y = True
pad_follow.use_rotation_z = True
pad_follow.use_scale_x = True
pad_follow.use_scale_y = True
pad_follow.use_scale_z = True
pad_follow.influence = 1.0
pad_follow.mute = False

# The foam bone is separate from the jaw-post/body hierarchy in the captured
# rig. Attach it to BODY, not directly to the object-mode empty. This makes the
# foam inherit all body motion while retaining its existing mounting offset.
foam_constraint = foam_bone.constraints.get(FOAM_CONSTRAINT_NAME)
if foam_constraint is None:
    foam_constraint = foam_bone.constraints.new("CHILD_OF")
    foam_constraint.name = FOAM_CONSTRAINT_NAME
    foam_constraint.target = armature
    foam_constraint.subtarget = BODY_BONE_NAME
    foam_constraint.target_space = "WORLD"
    foam_constraint.owner_space = "WORLD"
    foam_constraint.inverse_matrix = (
        armature.matrix_world @ body_bone.matrix
    ).inverted()
elif foam_constraint.type != "CHILD_OF":
    raise RuntimeError(f"{FOAM_CONSTRAINT_NAME} exists but is not Child Of.")
elif foam_constraint.target != armature or foam_constraint.subtarget != BODY_BONE_NAME:
    # Upgrade the preceding version, which targeted the control empty directly.
    # Recalculate the inverse so changing the target does not snap the foam.
    foam_constraint.target = armature
    foam_constraint.subtarget = BODY_BONE_NAME
    foam_constraint.target_space = "WORLD"
    foam_constraint.owner_space = "WORLD"
    foam_constraint.inverse_matrix = (
        armature.matrix_world @ body_bone.matrix
    ).inverted()

foam_constraint.target = armature
foam_constraint.subtarget = BODY_BONE_NAME
foam_constraint.target_space = "WORLD"
foam_constraint.owner_space = "WORLD"
foam_constraint.use_location_x = True
foam_constraint.use_location_y = True
foam_constraint.use_location_z = True
foam_constraint.use_rotation_x = True
foam_constraint.use_rotation_y = True
foam_constraint.use_rotation_z = True
foam_constraint.use_scale_x = True
foam_constraint.use_scale_y = True
foam_constraint.use_scale_z = True
foam_constraint.influence = 1.0
foam_constraint.mute = False

# Attach both physical rope endpoints directly to the rope-side eyehook. The
# west endpoint previously copied the south endpoint, adding a dependency hop
# that could update late and leave a visible kink. Its old Copy Location is
# removed before installing this direct source.
for rope_endpoint, constraint_name in rope_endpoints:
    for existing in list(rope_endpoint.constraints):
        if (existing.type == "COPY_LOCATION"
                and existing.name != constraint_name):
            rope_endpoint.constraints.remove(existing)

    rope_constraint = rope_endpoint.constraints.get(constraint_name)
    if rope_constraint is None:
        rope_constraint = rope_endpoint.constraints.new("COPY_LOCATION")
        rope_constraint.name = constraint_name
    elif rope_constraint.type != "COPY_LOCATION":
        raise RuntimeError(f"{constraint_name} exists but is not Copy Location.")

    rope_constraint.target = rope_eyehook
    rope_constraint.subtarget = ""
    rope_constraint.target_space = "WORLD"
    rope_constraint.owner_space = "WORLD"
    rope_constraint.use_x = True
    rope_constraint.use_y = True
    rope_constraint.use_z = True
    rope_constraint.invert_x = False
    rope_constraint.invert_y = False
    rope_constraint.invert_z = False
    rope_constraint.use_offset = False
    rope_constraint.influence = 1.0
    rope_constraint.mute = False

# Preserve the west rope's tangent distance and direction by moving its control
# with the Pad IK. Since the Pad IK follows the same master delta, translating
# the unit cannot collapse the tangent into a hard corner.
west_tangent_constraint = west_tangent.constraints.get(WEST_TANGENT_CONSTRAINT_NAME)
if west_tangent_constraint is None:
    west_tangent_constraint = west_tangent.constraints.new("CHILD_OF")
    west_tangent_constraint.name = WEST_TANGENT_CONSTRAINT_NAME
    west_tangent_constraint.target = pad_ik
    west_tangent_constraint.subtarget = ""
    west_tangent_constraint.target_space = "WORLD"
    west_tangent_constraint.owner_space = "WORLD"
    west_tangent_constraint.inverse_matrix = pad_ik.matrix_world.inverted()
elif west_tangent_constraint.type != "CHILD_OF":
    raise RuntimeError(
        f"{WEST_TANGENT_CONSTRAINT_NAME} exists but is not Child Of.")

west_tangent_constraint.target = pad_ik
west_tangent_constraint.subtarget = ""
west_tangent_constraint.target_space = "WORLD"
west_tangent_constraint.owner_space = "WORLD"
west_tangent_constraint.use_location_x = True
west_tangent_constraint.use_location_y = True
west_tangent_constraint.use_location_z = True
west_tangent_constraint.use_rotation_x = True
west_tangent_constraint.use_rotation_y = True
west_tangent_constraint.use_rotation_z = True
west_tangent_constraint.use_scale_x = True
west_tangent_constraint.use_scale_y = True
west_tangent_constraint.use_scale_z = True
west_tangent_constraint.influence = 1.0
west_tangent_constraint.mute = False

remove_cyclic_master_reach_constraints(control)
remove_old_static_limits(control)
remove_self_transform_drivers(control)
upgrading_fixed_length_model = (
    int(control.get(ROPE_CALIBRATION_PROPERTY, 0)) != ROPE_CALIBRATION_VERSION)
calibrate_current_state_as_tight(control)
if upgrading_fixed_length_model:
    # Install from the known intact pose, not from a temporarily stretched
    # test position. Later idempotent runs preserve the artist's current pose.
    control.location = (
        float(control["SW_TR_TAUT_CONTROL_X"]),
        float(control["SW_TR_TAUT_CONTROL_Y"]),
        float(control["SW_TR_TAUT_CONTROL_Z"]),
    )
    bpy.context.view_layer.update()
disable_runtime_recoil_callbacks()
install_fixed_length_pad_projection(control, pad_ik)
pad_follow.use_location_x = False
pad_follow.use_location_y = False
pad_follow.use_location_z = False
make_root_follow_clamped_pad(root_bone, pad_ik)
install_stable_broken_weapon_tether(control)
install_upward_tangent_alignment(control)
install_upward_midpoint_tilt(control)

for sag_config in SAG_CONFIGS:
    install_fixed_length_sag(sag_config, control)

# The extra flex-point experiment created unstable top-rope loops in the live
# rig. Permanently restore the captured four-point top-rope curves instead.
restore_original_top_rope_curves()

bpy.context.view_layer.update()

for obj in bpy.context.selected_objects:
    obj.select_set(False)
control.select_set(True)
bpy.context.view_layer.objects.active = control

print(
    f"ONE_TURNBUCKLE_CONTROL_READY::{EMPTY_NAME} -> "
    f"{ARMATURE_NAME}/{ROOT_BONE_NAME}; pad={PAD_IK_NAME}; "
    f"rope_endpoints={','.join(name for name, _ in ROPE_ENDPOINT_LINKS)}"
)

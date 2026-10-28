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
import copy
import json
import math
import time
from mathutils import Euler, Vector


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
CORNER_RESPONSE_VERSION = 14
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
        "CTRL_Rope Point_TR_N_mid | Tweak Loc X",
        "CTRL_Rope Point_TR_N_mid | Tweak Loc Y",
        "CTRL_Rope Point_TR_N_mid | Tweak Loc Z",
    ),
    "CTRL_Rope Point_TR_E_mid": (
        "CTRL_Rope Point_TR_E_mid | Tweak Loc X",
        "CTRL_Rope Point_TR_E_mid | Tweak Loc Y",
        "CTRL_Rope Point_TR_E_mid | Tweak Loc Z",
    ),
}

# User-facing top-rope controls only. Technical WRAP, Snap Target, and helper
# empties are deliberately excluded from manual tuning.
POSITIVE_X_TR_TUNING_TARGETS = (
    ("CTRL_Rope Point_TR_N_end_l", "North End L"),
    ("CTRL_Rope Point_TR_N_end_l_ctrl", "North End L Ctrl"),
    ("CTRL_Rope Point_TR_N_end_r", "North End R"),
    ("CTRL_Rope Point_TR_N_mid", "North Mid"),
    ("CTRL_Rope Point_TR_E_end_l", "East End L"),
    ("CTRL_Rope Point_TR_E_end_l_ctrl", "East End L Ctrl"),
    ("CTRL_Rope Point_TR_E_end_r", "East End R"),
    ("CTRL_Rope Point_TR_E_mid", "East Mid"),
    ("CTRL_Rope Point_TR_S_end_l", "South End L"),
    ("CTRL_Rope Point_TR_S_end_l_ctrl", "South End L Ctrl"),
    ("CTRL_Rope Point_TR_S_end_r", "South End R"),
    ("CTRL_Rope Point_TR_S_mid", "South Mid"),
    ("CTRL_Rope Point_TR_W_end_l", "West End L"),
    ("CTRL_Rope Point_TR_W_end_l_ctrl", "West End L Ctrl"),
    ("CTRL_Rope Point_TR_W_end_r", "West End R"),
    ("CTRL_Rope Point_TR_W_mid", "West Mid"),
    ("CTRL_Rope_TR_N_Mid_IK", "North Mid IK"),
    ("CTRL_Rope_TR_E_Mid_IK", "East Mid IK"),
    ("CTRL_Rope_TR_S_Mid_IK", "South Mid IK"),
    ("CTRL_Rope_TR_W_Mid_IK", "West Mid IK"),
    ("CTRL_Turnbuckle_Pad_TR_N_E_IK", "Pad North East"),
    ("CTRL_Turnbuckle_Pad_TR_N_W_IK", "Pad North West"),
    ("CTRL_Turnbuckle_Pad_TR_S_E_IK", "Pad South East"),
    ("CTRL_Turnbuckle_Pad_TR_S_W_IK", "Pad South West"),
)
POSITIVE_X_TR_TUNING_CONSTRAINT = "SW_TR_POSITIVE_X_USER_TUNE"
POSITIVE_X_TR_LOCATION_TWEAK_CONSTRAINT = "SW_TR_DIRECT_LOCATION_TWEAK"
POSITIVE_X_TR_ROTATION_CONSTRAINT = "SW_TR_DIRECT_ROTATION_TWEAK"
POSITIVE_X_TR_TUNING_REPORT = "SW_TR_Positive_X_Tuning_Values.json"
MASTER_OFFSET_LOCATION_PROPERTIES = tuple(
    f"{EMPTY_NAME} | Off Loc {axis}" for axis in "XYZ")
MASTER_OFFSET_ROTATION_PROPERTIES = tuple(
    f"{EMPTY_NAME} | Off Rot {axis}" for axis in "XYZ")
MASTER_DEFAULT_ROTATION_PROPERTIES = (
    "SW_TR_DEFAULT_ROTATION_X",
    "SW_TR_DEFAULT_ROTATION_Y",
    "SW_TR_DEFAULT_ROTATION_Z",
)
MASTER_OFFSET_PROXY_NAME = "TECH_SW_TR_MASTER_OFFSET_SOURCE"
MASTER_OFFSET_FOLLOW_CONSTRAINT = "SW_TR_MASTER_OFFSET_FOLLOW"
RESPONSIVE_PROPS_HANDLER_KEY = "SW_TR_RESPONSIVE_PROPS_HANDLER"
RESPONSIVE_PROPS_TIMER_KEY = "SW_TR_RESPONSIVE_PROPS_TIMER"
RESPONSIVE_PROPS_STATE_KEY = "SW_TR_RESPONSIVE_PROPS_STATE"
RESPONSIVE_PROPS_GUARD_KEY = "SW_TR_RESPONSIVE_PROPS_GUARD"


def master_motion_source(control):
    """Return the actual master; v011 removes the failed proxy cycle."""
    return control
# The artist-approved full-tension reference was captured with the master at
# positive local X 150 inches. Blender stores that distance as 3.81 metres.
# All exported tuning values are the endpoint of this entire travel, not a
# target that should be reached after the former 0.25 m / 9.84 in ramp.
POSITIVE_X_FULL_TRAVEL_INCHES = 150.0
POSITIVE_X_FULL_TRAVEL_METERS = POSITIVE_X_FULL_TRAVEL_INCHES * 0.0254

# Artist-approved v006 values exported at master X150 on 2026-10-17. These are
# used only when a property does not already exist, so rerunning v008 preserves
# later hand tuning. Values are Blender metres in each target's existing
# world-space additive position channels.
POSITIVE_X_X150_DEFAULTS = {
    "North End L": (-0.18, 0.02, 0.04),
    "North End L Ctrl": (-2.50, -2.50, -0.20),
    "North End R": (0.10, -0.04, 0.0),
    "North Mid": (0.0, 0.0, 0.35),
    "East End L": (-0.03, -0.03, 0.0),
    "East End L Ctrl": (0.08, 0.20, 0.0),
    "East End R": (-0.10, -0.04, 0.0),
    "East Mid": (0.0, 0.0, 0.35),
    "South End L": (0.0, 0.0, 0.0),
    "South End L Ctrl": (0.97, -0.16, -2.50),
    "South End R": (0.0, 0.0, 0.05),
    "South Mid": (0.0, 0.0, -0.19),
    "West End L": (0.0, 0.0, 0.0),
    "West End L Ctrl": (1.56, 0.54, 0.0),
    "West End R": (0.05, -0.12, 0.0),
    "West Mid": (0.0, 0.0, 0.15),
    "North Mid IK": (0.0, 0.0, 0.0),
    "East Mid IK": (0.0, 0.0, 0.35),
    "South Mid IK": (0.0, 0.0, 0.0),
    "West Mid IK": (0.0, 0.0, 0.0),
    "Pad North East": (0.0, 0.0, 0.0),
    "Pad North West": (0.55, 0.0, 0.0),
    "Pad South East": (0.55, 0.0, 0.0),
    "Pad South West": (0.0, 0.0, 0.0),
}


def positive_x_tuning_properties(object_name):
    """Per-Empty zero-based location tweaks on top of the baked X150 math."""
    return tuple(f"{object_name} | Tweak Loc {axis}" for axis in "XYZ")


def positive_x_rotation_properties(object_name):
    return tuple(f"{object_name} | Tweak Rot {axis}" for axis in "XYZ")


def legacy_positive_x_properties(label):
    return tuple(f"Positive X {label} {axis}" for axis in "XYZ")


def write_positive_x_tuning_report(control):
    """Create a copyable Blender Text report of every exposed tuning value."""
    controls = {}
    for object_name, label in POSITIVE_X_TR_TUNING_TARGETS:
        controls[object_name] = {
            "label": label,
            "baked_x150_location": {
                axis: float(value)
                for axis, value in zip("XYZ", POSITIVE_X_X150_DEFAULTS[label])
            },
            "tweak_location": {
                axis: float(control[property_name])
                for axis, property_name in zip(
                    "XYZ", positive_x_tuning_properties(object_name))
            },
            "tweak_rotation_radians": {
                axis: float(control[property_name])
                for axis, property_name in zip(
                    "XYZ", positive_x_rotation_properties(object_name))
            },
        }
    payload = {
        "version": 11,
        "master": EMPTY_NAME,
        "positive_x_full_travel_inches": POSITIVE_X_FULL_TRAVEL_INCHES,
        "positive_x_full_travel_meters": POSITIVE_X_FULL_TRAVEL_METERS,
        "master_offset_location": {
            axis: float(control[property_name])
            for axis, property_name in zip(
                "XYZ", MASTER_OFFSET_LOCATION_PROPERTIES)
        },
        "master_offset_rotation_radians": {
            axis: float(control[property_name])
            for axis, property_name in zip(
                "XYZ", MASTER_OFFSET_ROTATION_PROPERTIES)
        },
        "controls": controls,
    }
    text_block = bpy.data.texts.get(POSITIVE_X_TR_TUNING_REPORT)
    if text_block is None:
        text_block = bpy.data.texts.new(POSITIVE_X_TR_TUNING_REPORT)
    else:
        text_block.clear()
    text_block.write(json.dumps(payload, indent=2, sort_keys=True))


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
        source = master_motion_source(control)
        add_world_transform_variable(driver, "mx", source, "LOC_X")
        add_world_transform_variable(driver, "my", source, "LOC_Y")
        add_world_transform_variable(driver, "mz", source, "LOC_Z")
    return fcurve


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
        f"max(0,min(1,({local_z_travel})/.3048))"
    )
    if config["flex_axis"] == "Y":
        y_expression = f"(ay+by)*.5+({lateral})*({slack_factor})"
    else:
        x_expression = f"(ax+bx)*.5+({lateral})*({slack_factor})"
    z_expression = f"(az+bz)*.5-({vertical})*({slack_factor})"

    side_label = config["label"].split("_", 1)[0].title()
    tuning_label = f"{side_label} Mid"
    baked_values = POSITIVE_X_X150_DEFAULTS[tuning_label]
    positive_x_weight = (
        f"min(1,max(0,mz{(-rest_master.z):+.6g})"
        f"/3.81)"
    )
    for axis, (minimum, maximum, automatic_expression) in enumerate((
        ("min_x", "max_x", x_expression),
        ("min_y", "max_y", y_expression),
        ("min_z", "max_z", z_expression),
    )):
        expression = (
            f"({automatic_expression})+({positive_x_weight})"
            f"*({baked_values[axis]:.8g})"
        )
        for property_name in (minimum, maximum):
            drive_constraint_value(
                sag_constraint, property_name, expression,
                end_a, end_b, control)

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
        if (side in {"S", "W"}
                and curve.get("LSO_NATIVE_SELECTED_SPAN_SUBDIVISION_VERSION") == 16
                and len(curve.data.splines) == 1
                and curve.data.splines[0].type == "BEZIER"
                and len(curve.data.splines[0].bezier_points) == 10):
            # A completed v015 curve is already the exact deterministic result
            # of the requested two native subdivisions. Preserve it verbatim.
            continue

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
        # v015 stores the true pre-subdivision four-point source on the Curve.
        # Always restore from that immutable record on reruns so this older
        # cleanup stage can never reinterpret subdivided points as the source.
        restore_nodes = base_nodes
        stored_source = curve.get("LSO_NATIVE_SELECTED_SPAN_SOURCE_NODES")
        if stored_source:
            try:
                decoded = json.loads(stored_source)
                if len(decoded) == 4 and all(len(node) == 3 for node in decoded):
                    restore_nodes = decoded
            except (TypeError, ValueError, json.JSONDecodeError):
                pass
        for point, node in zip(spline.bezier_points, restore_nodes):
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
    source = master_motion_source(control)
    for axis, name in enumerate(("mx", "my", "mz")):
        add_single_property_variable(
            driver, name, source, f"location[{axis}]")
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
    defaults = [
        (BROKEN_FOLLOW_PROPERTY, 0.65, 0.0, 1.0),
        (BROKEN_MAX_PULL_PROPERTY, 1.50, 0.01, 2.50),
        (BROKEN_WIGGLE_PROPERTY, 0.020, 0.0, 0.08),
    ]
    for object_name, _label in POSITIVE_X_TR_TUNING_TARGETS:
        defaults.extend(
            (property_name, 0.0, -2.50, 2.50)
            for property_name in positive_x_tuning_properties(object_name))
        defaults.extend(
            (property_name, 0.0, -math.pi, math.pi)
            for property_name in positive_x_rotation_properties(object_name))
    defaults.extend(
        (property_name, 0.0, -10.0, 10.0)
        for property_name in MASTER_OFFSET_LOCATION_PROPERTIES)
    defaults.extend(
        (property_name, 0.0, -math.pi, math.pi)
        for property_name in MASTER_OFFSET_ROTATION_PROPERTIES)
    for name, default, minimum, maximum in defaults:
        if name not in control:
            control[name] = default
        ui_values = dict(
            min=minimum, max=maximum,
            soft_min=minimum, soft_max=maximum)
        if " Rot " in name:
            ui_values["subtype"] = "ANGLE"
        elif name in MASTER_OFFSET_LOCATION_PROPERTIES:
            ui_values["subtype"] = "DISTANCE"
        try:
            control.id_properties_ui(name).update(**ui_values)
        except TypeError:
            ui_values.pop("subtype", None)
            control.id_properties_ui(name).update(**ui_values)
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
    positive_x_weight = (
        f"min(1,max(0,{short_dz})"
        f"/3.81)"
    )
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
        midpoint_label = "North Mid" if "_N_" in midpoint_name else "East Mid"
        baked_tuning = POSITIVE_X_X150_DEFAULTS[midpoint_label]
        constraint.owner_space = "WORLD"
        constraint.use_transform_limit = False
        constraint.use_min_x = constraint.use_max_x = True
        constraint.use_min_y = constraint.use_max_y = True
        constraint.use_min_z = constraint.use_max_z = True
        constraint.mute = False

        for axis, (minimum_name, maximum_name) in enumerate((
                ("min_x", "max_x"), ("min_y", "max_y"), ("min_z", "max_z"))):
            automatic = vertical_response if axis == 2 else "0"
            response = (
                f"({automatic})+({positive_x_weight})"
                f"*({baked_tuning[axis]:.8g})"
            )
            expression = f"({base_world[axis]:.17g})+({response})"
            for property_name in (minimum_name, maximum_name):
                fcurve = drive_stable_property(
                    constraint, property_name, -1, expression, control)
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


def install_all_tr_positive_x_tuning(control):
    """Apply baked X150 math, then immediate zero-based artist tweaks."""
    rest_z = float(control["SW_TR_TAUT_CONTROL_Z"])
    midpoint_targets = set(POSITIVE_X_MIDPOINT_TUNING) | {
        "CTRL_Rope Point_TR_S_mid",
        "CTRL_Rope Point_TR_W_mid",
    }

    for object_name, label in POSITIVE_X_TR_TUNING_TARGETS:
        # North/East midpoint tuning is already integrated into their absolute
        # fixed-length constraints, so adding a second offset would double it.
        if object_name in midpoint_targets:
            continue
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "EMPTY":
            raise RuntimeError(f"Top-rope tuning Empty not found: {object_name}")

        constraint = obj.constraints.get(POSITIVE_X_TR_TUNING_CONSTRAINT)
        if constraint is None:
            constraint = obj.constraints.new("TRANSFORM")
            constraint.name = POSITIVE_X_TR_TUNING_CONSTRAINT
        elif constraint.type != "TRANSFORM":
            raise RuntimeError(
                f"{object_name}/{POSITIVE_X_TR_TUNING_CONSTRAINT} has wrong type.")

        constraint.target = master_motion_source(control)
        constraint.subtarget = ""
        constraint.target_space = "WORLD"
        constraint.owner_space = "WORLD"
        constraint.map_from = "LOCATION"
        constraint.map_to = "LOCATION"
        constraint.map_to_x_from = "Z"
        constraint.map_to_y_from = "Z"
        constraint.map_to_z_from = "Z"
        constraint.from_min_z = rest_z
        constraint.from_max_z = rest_z + POSITIVE_X_FULL_TRAVEL_METERS
        constraint.to_min_x = 0.0
        constraint.to_min_y = 0.0
        constraint.to_min_z = 0.0
        constraint.mix_mode = "ADD"
        constraint.use_motion_extrapolate = False
        constraint.influence = 1.0
        constraint.mute = False

        baked_values = POSITIVE_X_X150_DEFAULTS[label]
        for axis, constraint_property in enumerate(
                ("to_max_x", "to_max_y", "to_max_z")):
            try:
                constraint.driver_remove(constraint_property)
            except (TypeError, RuntimeError):
                pass
            setattr(constraint, constraint_property, baked_values[axis])

        obj.update_tag(refresh={"OBJECT"})

    # The named Tweak properties are direct artist corrections, not X150
    # endpoints. Keep them independent of master travel so editing any value
    # has an immediate visible effect even while the master is at its default.
    # The already-existing technical master proxy supplies a constant scale of
    # one; mapping that constant to identical min/max outputs gives us a clean
    # additive constraint without adding another Empty.
    for object_name, _label in POSITIVE_X_TR_TUNING_TARGETS:
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "EMPTY":
            raise RuntimeError(f"Top-rope tuning Empty not found: {object_name}")

        old_location_tweak = obj.constraints.get(
            POSITIVE_X_TR_LOCATION_TWEAK_CONSTRAINT)
        if old_location_tweak is not None:
            obj.constraints.remove(old_location_tweak)
        location_tweak = obj.constraints.new("TRANSFORM")
        location_tweak.name = POSITIVE_X_TR_LOCATION_TWEAK_CONSTRAINT
        location_tweak.target = master_motion_source(control)
        location_tweak.subtarget = ""
        location_tweak.target_space = "LOCAL"
        location_tweak.owner_space = "LOCAL"
        location_tweak.map_from = "SCALE"
        location_tweak.map_to = "LOCATION"
        location_tweak.map_to_x_from = "X"
        location_tweak.map_to_y_from = "Y"
        location_tweak.map_to_z_from = "Z"
        location_tweak.from_min_x = 0.0
        location_tweak.from_min_y = 0.0
        location_tweak.from_min_z = 0.0
        location_tweak.from_max_x = 2.0
        location_tweak.from_max_y = 2.0
        location_tweak.from_max_z = 2.0
        location_tweak.mix_mode = "ADD"
        location_tweak.use_motion_extrapolate = False
        location_tweak.influence = 1.0
        location_tweak.mute = False
        for property_name, minimum_property, maximum_property in zip(
                positive_x_tuning_properties(object_name),
                ("to_min_x", "to_min_y", "to_min_z"),
                ("to_max_x", "to_max_y", "to_max_z")):
            for constraint_property in (minimum_property, maximum_property):
                fcurve = location_tweak.driver_add(constraint_property)
                driver = fcurve.driver
                driver.type = "SCRIPTED"
                driver.expression = "tweak"
                add_single_property_variable(
                    driver, "tweak", control, f'["{property_name}"]')

        old_rotation_tweak = obj.constraints.get(
            POSITIVE_X_TR_ROTATION_CONSTRAINT)
        if old_rotation_tweak is not None:
            obj.constraints.remove(old_rotation_tweak)
        constraint = obj.constraints.new("TRANSFORM")
        constraint.name = POSITIVE_X_TR_ROTATION_CONSTRAINT
        constraint.target = master_motion_source(control)
        constraint.subtarget = ""
        constraint.target_space = "LOCAL"
        constraint.owner_space = "LOCAL"
        constraint.map_from = "SCALE"
        constraint.map_to = "ROTATION"
        constraint.map_to_x_from = "X"
        constraint.map_to_y_from = "Y"
        constraint.map_to_z_from = "Z"
        constraint.from_min_x = 0.0
        constraint.from_min_y = 0.0
        constraint.from_min_z = 0.0
        constraint.from_max_x = 2.0
        constraint.from_max_y = 2.0
        constraint.from_max_z = 2.0
        constraint.mix_mode_rot = "ADD"
        constraint.to_euler_order = "XYZ"
        constraint.use_motion_extrapolate = False
        constraint.influence = 1.0
        constraint.mute = False

        for property_name, minimum_property, maximum_property in zip(
                positive_x_rotation_properties(object_name),
                ("to_min_x_rot", "to_min_y_rot", "to_min_z_rot"),
                ("to_max_x_rot", "to_max_y_rot", "to_max_z_rot")):
            for constraint_property in (minimum_property, maximum_property):
                fcurve = constraint.driver_add(constraint_property)
                driver = fcurve.driver
                driver.type = "SCRIPTED"
                driver.expression = "tweak"
                add_single_property_variable(
                    driver, "tweak", control, f'["{property_name}"]')
        obj.update_tag(refresh={"OBJECT"})

    bpy.context.view_layer.update()


def install_master_offset_drivers(control):
    """Drive the master directly from its own zero-based offset properties."""
    if not all(name in control for name in MASTER_DEFAULT_ROTATION_PROPERTIES):
        for name, value in zip(
                MASTER_DEFAULT_ROTATION_PROPERTIES, control.rotation_euler):
            control[name] = float(value)

    rest_location = Vector((
        float(control["SW_TR_TAUT_CONTROL_X"]),
        float(control["SW_TR_TAUT_CONTROL_Y"]),
        float(control["SW_TR_TAUT_CONTROL_Z"]),
    ))
    rest_rotation = tuple(
        float(control[name]) for name in MASTER_DEFAULT_ROTATION_PROPERTIES)
    basis = Euler(rest_rotation, "XYZ").to_matrix()

    # Remove the v011 work-in-progress proxy. Its property-driver dependency
    # pointed back to a master constrained to the proxy, forming a cycle.
    follow = control.constraints.get(MASTER_OFFSET_FOLLOW_CONSTRAINT)
    if follow is not None:
        control.constraints.remove(follow)
    proxy = bpy.data.objects.get(MASTER_OFFSET_PROXY_NAME)
    if proxy is not None:
        bpy.data.objects.remove(proxy, do_unlink=True)

    for axis in range(3):
        try:
            control.driver_remove("location", axis)
        except (TypeError, RuntimeError):
            pass
        fcurve = control.driver_add("location", axis)
        driver = fcurve.driver
        driver.type = "SCRIPTED"
        driver.use_self = True
        terms = []
        for offset_axis, property_name in enumerate(
                MASTER_OFFSET_LOCATION_PROPERTIES):
            coefficient = basis[axis][offset_axis]
            terms.append(
                f"({coefficient:.6g})*self[{property_name!r}]")
        expression = f"{rest_location[axis]:.12g}+" + "+".join(terms)
        if len(expression) > 255:
            raise RuntimeError(
                f"Master offset driver expression is too long ({len(expression)}).")
        driver.expression = expression

    for axis, property_name in enumerate(MASTER_OFFSET_ROTATION_PROPERTIES):
        try:
            control.driver_remove("rotation_euler", axis)
        except (TypeError, RuntimeError):
            pass
        fcurve = control.driver_add("rotation_euler", axis)
        driver = fcurve.driver
        driver.type = "SCRIPTED"
        driver.use_self = True
        driver.expression = (
            f"{rest_rotation[axis]:.12g}+self[{property_name!r}]")

    control.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()


def remove_obsolete_tuning_properties_and_constraints(control):
    """Remove v008/v009 UI remnants after v010 drivers are installed."""
    obsolete_constraints = {
        "SW_TR_MANUAL_LOCATION_OFFSET",
        "SW_TR_MANUAL_ROTATION_OFFSET",
    }
    for object_name, label in POSITIVE_X_TR_TUNING_TARGETS:
        obj = bpy.data.objects.get(object_name)
        if obj is not None:
            for constraint in list(obj.constraints):
                if constraint.name in obsolete_constraints:
                    obj.constraints.remove(constraint)
        obsolete_names = list(legacy_positive_x_properties(label))
        for category in (
                "X150 Location", "X150 Rotation",
                "Offset Location", "Offset Rotation"):
            obsolete_names.extend(
                f"{object_name} | {category} {axis}" for axis in "XYZ")
        for property_name in obsolete_names:
            if (property_name in control
                    and property_name not in MASTER_OFFSET_LOCATION_PROPERTIES
                    and property_name not in MASTER_OFFSET_ROTATION_PROPERTIES):
                del control[property_name]


def responsive_property_signature(control):
    """Compact value snapshot; evaluated only after an existing DG update."""
    names = list(MASTER_OFFSET_LOCATION_PROPERTIES)
    names.extend(MASTER_OFFSET_ROTATION_PROPERTIES)
    for object_name, _label in POSITIVE_X_TR_TUNING_TARGETS:
        names.extend(positive_x_tuning_properties(object_name))
        names.extend(positive_x_rotation_properties(object_name))
    return tuple(float(control.get(name, 0.0)) for name in names)


def responsive_props_depsgraph_handler(_scene, _depsgraph):
    """Bridge Blender ID-property edits to the saved driver graph."""
    namespace = bpy.app.driver_namespace
    if namespace.get(RESPONSIVE_PROPS_GUARD_KEY, False):
        return
    control = bpy.data.objects.get(EMPTY_NAME)
    if control is None:
        return
    signature = responsive_property_signature(control)
    if signature == namespace.get(RESPONSIVE_PROPS_STATE_KEY):
        return
    namespace[RESPONSIVE_PROPS_STATE_KEY] = signature
    namespace[RESPONSIVE_PROPS_GUARD_KEY] = True
    try:
        control.update_tag(refresh={"OBJECT"})
        for object_name, _label in POSITIVE_X_TR_TUNING_TARGETS:
            obj = bpy.data.objects.get(object_name)
            if obj is not None:
                obj.update_tag(refresh={"OBJECT"})
    finally:
        namespace[RESPONSIVE_PROPS_GUARD_KEY] = False


def responsive_props_timer_tick():
    """Poll only the small property tuple; do no rope or geometry work."""
    responsive_props_depsgraph_handler(None, None)
    return 0.03


def install_responsive_props_handler(control):
    """Install one low-cost property refresh bridge idempotently."""
    namespace = bpy.app.driver_namespace
    old = namespace.get(RESPONSIVE_PROPS_HANDLER_KEY)
    if old is not None:
        while old in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(old)
    namespace[RESPONSIVE_PROPS_HANDLER_KEY] = responsive_props_depsgraph_handler
    old_timer = namespace.get(RESPONSIVE_PROPS_TIMER_KEY)
    if old_timer is not None and bpy.app.timers.is_registered(old_timer):
        bpy.app.timers.unregister(old_timer)
    namespace[RESPONSIVE_PROPS_TIMER_KEY] = responsive_props_timer_tick
    namespace[RESPONSIVE_PROPS_STATE_KEY] = responsive_property_signature(control)
    namespace[RESPONSIVE_PROPS_GUARD_KEY] = False
    if responsive_props_depsgraph_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(
            responsive_props_depsgraph_handler)
    if not bpy.app.timers.is_registered(responsive_props_timer_tick):
        bpy.app.timers.register(
            responsive_props_timer_tick, first_interval=0.03, persistent=True)


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

# v014 recovery: if v012/v013 already converted the live master to a bone,
# restore the proven v011 Empty-driven hierarchy before rebuilding drivers.
HYBRID_CONTROL_BONE_NAME = EMPTY_NAME
HYBRID_TECH_ADAPTER_NAME = "TECH_SW_TR_CONTROL_REQUEST_SOURCE"
hybrid_existing_user_matrix = None
hybrid_existing_control = bpy.data.objects.get(EMPTY_NAME)
if (hybrid_existing_control is not None
        and int(hybrid_existing_control.get(
            "LSO_SW_TR_HYBRID_VERSION", 0)) == 14):
    hybrid_existing_user_matrix = hybrid_existing_control.matrix_world.copy()

for namespace_key in (
        "LSO_SW_TR_CONTROL_BONE_TIMER", "LSO_SW_TR_CONTROL_BONE_HANDLER"):
    callback = bpy.app.driver_namespace.get(namespace_key)
    if callback is not None:
        if bpy.app.timers.is_registered(callback):
            bpy.app.timers.unregister(callback)
        while callback in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(callback)
        bpy.app.driver_namespace.pop(namespace_key, None)

existing_export_bone = armature.pose.bones.get(HYBRID_CONTROL_BONE_NAME)
existing_adapter = bpy.data.objects.get(HYBRID_TECH_ADAPTER_NAME)
if existing_export_bone is not None:
    unit_names = [root_bone.name]
    unit_names.extend(bone.name for bone in root_bone.children_recursive)
    unit_world = {
        name: armature.matrix_world @ armature.pose.bones[name].matrix.copy()
        for name in unit_names
    }
    if existing_adapter is not None and bpy.data.objects.get(EMPTY_NAME) is None:
        existing_adapter.name = EMPTY_NAME
        existing_adapter.hide_set(False)
        existing_adapter.hide_select = False
        existing_adapter.hide_render = False
        for property_name in existing_export_bone.keys():
            if property_name != "_RNA_UI":
                existing_adapter[property_name] = copy.deepcopy(
                    existing_export_bone[property_name])
                try:
                    ui_values = existing_export_bone.id_properties_ui(
                        property_name).as_dict()
                    existing_adapter.id_properties_ui(property_name).update(
                        **ui_values)
                except (KeyError, TypeError, ValueError):
                    pass

    if root_bone.bone.parent == existing_export_bone.bone:
        if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        for selected in bpy.context.selected_objects:
            selected.select_set(False)
        armature.hide_set(False)
        armature.select_set(True)
        bpy.context.view_layer.objects.active = armature
        bpy.ops.object.mode_set(mode="EDIT")
        edit_root = armature.data.edit_bones[ROOT_BONE_NAME]
        root_rest_matrix = edit_root.matrix.copy()
        edit_root.parent = None
        edit_root.use_connect = False
        edit_root.matrix = root_rest_matrix
        bpy.ops.object.mode_set(mode="POSE")
        for bone_name, world_matrix in unit_world.items():
            armature.pose.bones[bone_name].matrix = (
                armature.matrix_world.inverted() @ world_matrix)
        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.context.view_layer.update()

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
install_master_offset_drivers(control)
install_upward_tangent_alignment(control)
install_upward_midpoint_tilt(control)

for sag_config in SAG_CONFIGS:
    install_fixed_length_sag(sag_config, control)

# Add the artist correction layers last so no absolute rope solver constraint
# can overwrite a Tweak value on south/west midpoints.
install_all_tr_positive_x_tuning(control)

remove_obsolete_tuning_properties_and_constraints(control)
install_responsive_props_handler(control)

# The extra flex-point experiment created unstable top-rope loops in the live
# rig. Permanently restore the captured four-point top-rope curves instead.
restore_original_top_rope_curves()
write_positive_x_tuning_report(control)

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


# ---------------------------------------------------------------------------
# v014 hybrid: fast Empty master + one-way export-bone mirror.
# ---------------------------------------------------------------------------

HYBRID_VERSION = 14
HYBRID_FOLLOW_CONSTRAINT = "HYBRID_FAST_EMPTY_TO_EXPORT_BONE"
HYBRID_PROP_TIMER_KEY = "LSO_SW_TR_HYBRID_PROP_TIMER"
HYBRID_PROP_STATE_KEY = "LSO_SW_TR_HYBRID_PROP_STATE"
HYBRID_MASTER_PROP_STATE_KEY = "LSO_SW_TR_HYBRID_MASTER_PROP_STATE"
HYBRID_DISPLAY_OBJECT = "WGT_SW_TR_UPRIGHT_TURNBUCKLE_CONTROL"


def hybrid_animation_owners():
    seen = set()
    for collection_name in (
            "objects", "scenes", "armatures", "curves", "meshes",
            "materials", "node_groups", "shape_keys"):
        collection_data = getattr(bpy.data, collection_name, ())
        for owner in collection_data:
            pointer = owner.as_pointer()
            if pointer not in seen:
                seen.add(pointer)
                yield owner


def hybrid_retarget_live_drivers_to_empty(export_bone, empty):
    """Keep the armature mirror out of every live rope driver dependency."""
    property_paths = {}
    for property_name in export_bone.keys():
        if property_name == "_RNA_UI":
            continue
        escaped_bone = HYBRID_CONTROL_BONE_NAME.replace("\\", "\\\\").replace(
            '"', '\\"')
        escaped_property = property_name.replace("\\", "\\\\").replace(
            '"', '\\"')
        property_paths[
            f'pose.bones["{escaped_bone}"]["{escaped_property}"]'
        ] = f'["{escaped_property}"]'
    retargeted = 0
    for owner in hybrid_animation_owners():
        animation = getattr(owner, "animation_data", None)
        if animation is None:
            continue
        for fcurve in animation.drivers:
            for variable in fcurve.driver.variables:
                if variable.type != "SINGLE_PROP":
                    continue
                for target in variable.targets:
                    replacement = property_paths.get(target.data_path)
                    if target.id == armature and replacement is not None:
                        target.id_type = "OBJECT"
                        target.id = empty
                        target.data_path = replacement
                        retargeted += 1
    return retargeted


def hybrid_copy_properties(source, destination):
    copied = 0
    for property_name in source.keys():
        if property_name == "_RNA_UI":
            continue
        destination[property_name] = copy.deepcopy(source[property_name])
        copied += 1
        try:
            destination.id_properties_ui(property_name).update(
                **source.id_properties_ui(property_name).as_dict())
        except (KeyError, TypeError, ValueError):
            pass
    return copied


def hybrid_ensure_export_bone():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for selected in bpy.context.selected_objects:
        selected.select_set(False)
    armature.hide_set(False)
    armature.select_set(True)
    bpy.context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        edit_root = armature.data.edit_bones[ROOT_BONE_NAME]
        export_edit = armature.data.edit_bones.get(HYBRID_CONTROL_BONE_NAME)
        if export_edit is None:
            export_edit = armature.data.edit_bones.new(HYBRID_CONTROL_BONE_NAME)
            export_edit.matrix = edit_root.matrix.copy()
            export_edit.length = max(edit_root.length * 1.35, 1.0)
        export_edit.parent = None
        export_edit.use_connect = False
        export_edit.use_deform = True
    finally:
        bpy.ops.object.mode_set(mode="POSE")
    export_bone = armature.pose.bones[HYBRID_CONTROL_BONE_NAME]
    export_bone.rotation_mode = "QUATERNION"
    bpy.ops.object.mode_set(mode="OBJECT")
    return export_bone


def hybrid_ensure_display_shape(export_bone):
    shape = bpy.data.objects.get(HYBRID_DISPLAY_OBJECT)
    if shape is None:
        mesh = bpy.data.meshes.new(HYBRID_DISPLAY_OBJECT + "_MESH")
        vertices = (
            (0.0, 0.0, 0.0),
            (0.18, 0.22, 0.18), (-0.18, 0.22, 0.18),
            (-0.18, 0.22, -0.18), (0.18, 0.22, -0.18),
            (0.0, 1.0, 0.0),
        )
        edges = (
            (0, 1), (0, 2), (0, 3), (0, 4),
            (1, 2), (2, 3), (3, 4), (4, 1),
            (1, 5), (2, 5), (3, 5), (4, 5),
        )
        mesh.from_pydata(vertices, edges, ())
        mesh.update()
        shape = bpy.data.objects.new(HYBRID_DISPLAY_OBJECT, mesh)
        shape_collection = bpy.data.collections.get("WGT - Rig Control Shapes")
        if shape_collection is None:
            shape_collection = bpy.data.collections.new(
                "WGT - Rig Control Shapes")
            bpy.context.scene.collection.children.link(shape_collection)
        shape_collection.objects.link(shape)
    shape.hide_render = True
    shape.hide_select = True
    shape.display_type = "WIRE"
    shape.hide_set(True)
    export_bone.custom_shape = shape
    export_bone.use_custom_shape_bone_size = False
    export_bone.custom_shape_scale_xyz = (4.5, 4.5, 4.5)
    export_bone.custom_shape_rotation_euler = (math.radians(90.0), 0.0, 0.0)


def hybrid_parent_foam_to_body():
    """Make foam inherit BODY directly while retaining its mounting offset."""
    foam = armature.pose.bones[FOAM_BONE_NAME]
    body = armature.pose.bones[BODY_BONE_NAME]
    foam_world = armature.matrix_world @ foam.matrix.copy()
    existing = foam.constraints.get(FOAM_CONSTRAINT_NAME)
    if existing is not None:
        foam.constraints.remove(existing)
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for selected in bpy.context.selected_objects:
        selected.select_set(False)
    armature.select_set(True)
    bpy.context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode="EDIT")
    edit_foam = armature.data.edit_bones[FOAM_BONE_NAME]
    edit_body = armature.data.edit_bones[BODY_BONE_NAME]
    foam_rest = edit_foam.matrix.copy()
    edit_foam.parent = edit_body
    edit_foam.use_connect = False
    edit_foam.matrix = foam_rest
    bpy.ops.object.mode_set(mode="POSE")
    foam.matrix = armature.matrix_world.inverted() @ foam_world
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.update()


def hybrid_property_signature(empty):
    signature = []
    for property_name in sorted(empty.keys()):
        if property_name == "_RNA_UI":
            continue
        value = empty[property_name]
        try:
            value = tuple(value.to_list())
        except AttributeError:
            pass
        signature.append((property_name, repr(value)))
    return tuple(signature)


def hybrid_master_property_signature(empty):
    names = (*MASTER_OFFSET_LOCATION_PROPERTIES,
             *MASTER_OFFSET_ROTATION_PROPERTIES)
    return tuple(round(float(empty.get(name, 0.0)), 10) for name in names)


def hybrid_apply_master_properties(empty):
    """Apply zero-based props only on edit; leave ordinary transforms free."""
    rest_location = Vector((
        float(empty["SW_TR_TAUT_CONTROL_X"]),
        float(empty["SW_TR_TAUT_CONTROL_Y"]),
        float(empty["SW_TR_TAUT_CONTROL_Z"]),
    ))
    rest_rotation = Euler(tuple(float(empty[name])
        for name in MASTER_DEFAULT_ROTATION_PROPERTIES), "XYZ")
    location_offset = Vector(tuple(float(empty[name])
        for name in MASTER_OFFSET_LOCATION_PROPERTIES))
    rotation_offset = Vector(tuple(float(empty[name])
        for name in MASTER_OFFSET_ROTATION_PROPERTIES))
    empty.location = rest_location + rest_rotation.to_matrix() @ location_offset
    empty.rotation_mode = "XYZ"
    empty.rotation_euler = Euler(tuple(
        float(rest_rotation[index]) + rotation_offset[index]
        for index in range(3)), "XYZ")
    empty.update_tag(refresh={"OBJECT"})


def hybrid_property_timer():
    empty = bpy.data.objects.get(EMPTY_NAME)
    rig = bpy.data.objects.get(ARMATURE_NAME)
    bone = rig.pose.bones.get(HYBRID_CONTROL_BONE_NAME) if rig else None
    if empty is None or bone is None:
        return 0.1
    master_signature = hybrid_master_property_signature(empty)
    previous_master = bpy.app.driver_namespace.get(
        HYBRID_MASTER_PROP_STATE_KEY)
    if previous_master is not None and master_signature != previous_master:
        hybrid_apply_master_properties(empty)
    bpy.app.driver_namespace[HYBRID_MASTER_PROP_STATE_KEY] = master_signature
    signature = hybrid_property_signature(empty)
    if signature != bpy.app.driver_namespace.get(HYBRID_PROP_STATE_KEY):
        hybrid_copy_properties(empty, bone)
        bone["LSO_SW_TR_HYBRID_VERSION"] = HYBRID_VERSION
        bpy.app.driver_namespace[HYBRID_PROP_STATE_KEY] = signature
    return 0.05


export_bone = hybrid_ensure_export_bone()
hybrid_ensure_display_shape(export_bone)
hybrid_parent_foam_to_body()

# The Empty is the only live transform/property source. The export bone is a
# leaf mirror, so moving the control does not force the rope armature to solve
# through an additional control-bone parent chain.
follow = export_bone.constraints.get(HYBRID_FOLLOW_CONSTRAINT)
if follow is None:
    follow = export_bone.constraints.new("COPY_TRANSFORMS")
    follow.name = HYBRID_FOLLOW_CONSTRAINT
elif follow.type != "COPY_TRANSFORMS":
    export_bone.constraints.remove(follow)
    follow = export_bone.constraints.new("COPY_TRANSFORMS")
    follow.name = HYBRID_FOLLOW_CONSTRAINT
follow.target = control
follow.subtarget = ""
follow.target_space = "WORLD"
follow.owner_space = "WORLD"
follow.mix_mode = "REPLACE"
follow.influence = 1.0
follow.mute = False

control.hide_set(False)
control.hide_select = False
control.hide_render = False
control.show_name = True
control.show_in_front = True
control.empty_display_type = "CUBE"
control.empty_display_size = EMPTY_DISPLAY_SIZE
control["LSO_SW_TR_HYBRID_VERSION"] = HYBRID_VERSION
copied_properties = hybrid_copy_properties(control, export_bone)
retargeted_inputs = hybrid_retarget_live_drivers_to_empty(export_bone, control)

# v011 used transform drivers so the zero-based offset fields could restore a
# known default. In hybrid mode those fields are applied by the lightweight
# property watcher instead, leaving Location/Rotation directly draggable and
# removing the armature evaluation cost from ordinary Empty movement.
preserved_control_matrix = (
    hybrid_existing_user_matrix.copy()
    if hybrid_existing_user_matrix is not None else control.matrix_world.copy())
for channel in ("location", "rotation_euler"):
    for axis in range(3):
        try:
            control.driver_remove(channel, axis)
        except (TypeError, RuntimeError):
            pass
control.matrix_world = preserved_control_matrix
control.update_tag(refresh={"OBJECT"})

old_hybrid_timer = bpy.app.driver_namespace.get(HYBRID_PROP_TIMER_KEY)
if old_hybrid_timer is not None and bpy.app.timers.is_registered(old_hybrid_timer):
    bpy.app.timers.unregister(old_hybrid_timer)
bpy.app.driver_namespace[HYBRID_PROP_TIMER_KEY] = hybrid_property_timer
bpy.app.driver_namespace.pop(HYBRID_PROP_STATE_KEY, None)
bpy.app.driver_namespace[HYBRID_MASTER_PROP_STATE_KEY] = (
    hybrid_master_property_signature(control))
hybrid_property_timer()
if not bpy.app.timers.is_registered(hybrid_property_timer):
    bpy.app.timers.register(hybrid_property_timer, first_interval=0.05,
                            persistent=True)

bpy.context.view_layer.update()
for selected in bpy.context.selected_objects:
    selected.select_set(False)
control.select_set(True)
bpy.context.view_layer.objects.active = control

print(
    "HYBRID_EMPTY_BONE_READY::"
    f"empty={control.name}; export_bone={ARMATURE_NAME}/{export_bone.name}; "
    f"mirrored_properties={copied_properties}; "
    f"retargeted_driver_inputs={retargeted_inputs}; "
    f"foam_parent={BODY_BONE_NAME}"
)


# ---------------------------------------------------------------------------
# v016: native subdivision with Hook indices remapped to original controls.
# ---------------------------------------------------------------------------

CURVE_SUBDIVISION_VERSION = 16
TOP_CURVES_TO_SUBDIVIDE = (
    "ROPE_SOUTH_TOP_CURVE",
    "ROPE_WEST_TOP_CURVE",
)

ORIGINAL_HOOK_REMAP = {
    "ROPE_SOUTH_TOP_CURVE": (
        ("ROPE_SOUTH_TOP_CURVE_POINT_0_HOOK", 0),
        ("ROPE_SOUTH_TOP_CURVE_POINT_1_HOOK", 4),
        ("ROPE_SOUTH_TOP_CURVE_POINT_2_HOOK", 8),
        ("ROPE_SOUTH_TOP_CURVE_POINT_3_HOOK", 9),
    ),
    "ROPE_WEST_TOP_CURVE": (
        ("ROPE_WEST_TOP_CURVE_POINT_0_HOOK", 0),
        ("ROPE_WEST_TOP_CURVE_POINT_1_HOOK", 4),
        ("ROPE_WEST_TOP_CURVE_POINT_2_HOOK", 8),
        ("ROPE_WEST_TOP_CURVE_POINT_3_HOOK", 9),
    ),
}


def curve_control_count(curve_object):
    total = 0
    for spline in curve_object.data.splines:
        total += (len(spline.bezier_points) if spline.type == "BEZIER"
                  else len(spline.points))
    return total


def curve_path_length(curve_object):
    return sum(float(spline.calc_length(resolution=128))
               for spline in curve_object.data.splines)


def curve_last_coordinate(spline):
    if spline.type == "BEZIER":
        return spline.bezier_points[-1].co.copy()
    return spline.points[-1].co.copy()


def subdivide_selected_curve_span_twice(curve_name):
    curve_object = bpy.data.objects.get(curve_name)
    if curve_object is None or curve_object.type != "CURVE":
        raise RuntimeError(f"Top curve not found: {curve_name}")
    if not curve_object.data.splines:
        raise RuntimeError(f"Top curve has no spline: {curve_name}")

    if (curve_object.get("LSO_NATIVE_SELECTED_SPAN_SUBDIVISION_VERSION")
            == CURVE_SUBDIVISION_VERSION
            and len(curve_object.data.splines) == 1
            and curve_object.data.splines[0].type == "BEZIER"
            and len(curve_object.data.splines[0].bezier_points) == 10):
        current_length = curve_path_length(curve_object)
        return 10, 10, current_length, current_length

    before_matrix = curve_object.matrix_world.copy()
    before_modifiers = tuple(modifier.name for modifier in curve_object.modifiers)
    before_count = curve_control_count(curve_object)
    before_length = curve_path_length(curve_object)
    last_coordinates = [curve_last_coordinate(spline)
                        for spline in curve_object.data.splines]
    source_nodes = []
    for spline in curve_object.data.splines:
        if spline.type != "BEZIER":
            continue
        source_nodes.extend((
            tuple(point.co),
            tuple(point.handle_left),
            tuple(point.handle_right),
        ) for point in spline.bezier_points)

    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for selected in bpy.context.selected_objects:
        selected.select_set(False)
    curve_object.hide_set(False)
    curve_object.select_set(True)
    bpy.context.view_layer.objects.active = curve_object
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.curve.select_all(action="DESELECT")

    # Select every control point except the final point of each spline. This is
    # the scripted equivalent of the user's box selection. Handles belonging
    # to that last Bezier point remain unselected as well.
    for spline in curve_object.data.splines:
        if spline.type == "BEZIER":
            points = spline.bezier_points
            for point in points[:-1]:
                point.select_control_point = True
                point.select_left_handle = True
                point.select_right_handle = True
            final = points[-1]
            final.select_control_point = False
            final.select_left_handle = False
            final.select_right_handle = False
        else:
            for point in spline.points[:-1]:
                point.select = True
            spline.points[-1].select = False

    # Exactly the same native operation as Right Click > Subdivide performed
    # twice. No Subdivision Surface or other modifier is created.
    for _pass in range(2):
        result = bpy.ops.curve.subdivide(number_cuts=1)
        if "FINISHED" not in result:
            raise RuntimeError(
                f"Native curve subdivision failed for {curve_name}: {result}")

    bpy.ops.object.mode_set(mode="OBJECT")

    # Two subdivision passes turn the original control indices 0,1,2,3 into
    # 0,4,8,9. Hook modifiers do not reliably update these Curve vertex
    # indices themselves. Explicitly bind every existing Hook back to the same
    # original control (left handle, center, right handle). The six inserted
    # controls remain unhooked interpolation points, exactly as intended.
    for hook_name, point_index in ORIGINAL_HOOK_REMAP[curve_name]:
        hook = curve_object.modifiers.get(hook_name)
        if hook is None or hook.type != "HOOK":
            raise RuntimeError(
                f"Required original Hook missing on {curve_name}: {hook_name}")
        hook.vertex_indices_set((
            point_index * 3,
            point_index * 3 + 1,
            point_index * 3 + 2,
        ))
        hook.center = curve_object.data.splines[0].bezier_points[point_index].co

    curve_object.data.update_tag()
    curve_object.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()

    after_count = curve_control_count(curve_object)
    after_length = curve_path_length(curve_object)
    if after_count <= before_count:
        raise RuntimeError(f"No curve points were added to {curve_name}.")
    if tuple(modifier.name for modifier in curve_object.modifiers) != before_modifiers:
        raise RuntimeError(f"A modifier changed on {curve_name}.")
    if max(abs(a - b) for row_a, row_b in zip(
            curve_object.matrix_world, before_matrix)
            for a, b in zip(row_a, row_b)) > 1.0e-7:
        raise RuntimeError(f"Object transform changed on {curve_name}.")
    for spline, before_last in zip(
            curve_object.data.splines, last_coordinates):
        # Entering/exiting Edit Mode can round an already driven Curve point by
        # a few 32-bit-float ULPs.  This remains far below visible/model-space
        # movement and must not be confused with moving the excluded endpoint.
        if (curve_last_coordinate(spline) - before_last).length > 2.0e-5:
            raise RuntimeError(f"Final control moved on {curve_name}.")
        if spline.type == "BEZIER":
            final = spline.bezier_points[-1]
            if (final.select_control_point or final.select_left_handle
                    or final.select_right_handle):
                raise RuntimeError(
                    f"Final Bezier control became selected on {curve_name}.")
        elif spline.points[-1].select:
            raise RuntimeError(
                f"Final spline control became selected on {curve_name}.")
    # Native Bezier subdivision preserves the path. Allow only sampling noise.
    if abs(after_length - before_length) > max(2.0e-4, before_length * 2.0e-5):
        raise RuntimeError(
            f"Visible path changed while subdividing {curve_name}: "
            f"{before_length:.9f} -> {after_length:.9f}")
    curve_object["LSO_NATIVE_SELECTED_SPAN_SUBDIVISION_VERSION"] = (
        CURVE_SUBDIVISION_VERSION)
    if "LSO_NATIVE_SELECTED_SPAN_SOURCE_NODES" not in curve_object:
        curve_object["LSO_NATIVE_SELECTED_SPAN_SOURCE_NODES"] = json.dumps(
            source_nodes, separators=(",", ":"))
    return before_count, after_count, before_length, after_length


subdivision_results = {
    curve_name: subdivide_selected_curve_span_twice(curve_name)
    for curve_name in TOP_CURVES_TO_SUBDIVIDE
}

# Leave the fast hybrid Empty selected for immediate testing.
if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
    bpy.ops.object.mode_set(mode="OBJECT")
for selected in bpy.context.selected_objects:
    selected.select_set(False)
control.select_set(True)
bpy.context.view_layer.objects.active = control

print(
    "NATIVE_SW_TOP_CURVE_SUBDIVISION_READY::"
    + "; ".join(
        f"{name}={values[0]}->{values[1]} points"
        for name, values in subdivision_results.items())
)

"""Artist-edit the South and West top ropes without installing an add-on.

Run once from Blender's Text Editor. The script:
  * keeps the current South/West curve shape as the starting shape;
  * creates editable DRAW_ guide Curves for direct Curve Pen/Edit Mode work;
  * adds zero-based point/handle controls to the existing SW turnbuckle Empty;
  * connects both inputs directly to the real rope Curves;
  * disables (does not delete) the old Hook modifiers while artist mode is active.

Edit MODE below to RESTORE_ORIGINAL_RIG and run again to mute the artist
drivers and re-enable the original Hook modifiers.
"""

import bpy
import json


MODE = "CREATE_OR_UPDATE"  # or "RESTORE_ORIGINAL_RIG"
MASTER_NAME = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
GUIDE_COLLECTION = "Rope Artist Draw Guides"
DRIVER_TAG = "RopeArtistDrawGuide"
CURVES = {
    "SOUTH": ("ROPE_SOUTH_TOP_CURVE", "DRAW_ROPE_SOUTH_TOP_CURVE", (1.0, 0.12, 0.04, 1.0)),
    "WEST": ("ROPE_WEST_TOP_CURVE", "DRAW_ROPE_WEST_TOP_CURVE", (0.04, 0.35, 1.0, 1.0)),
}


def fail(message):
    raise RuntimeError("SW rope artist controls stopped safely: " + message)


master = bpy.data.objects.get(MASTER_NAME)
if master is None or master.type != "EMPTY":
    fail(f"required Empty is missing: {MASTER_NAME}")


def ensure_collection():
    collection = bpy.data.collections.get(GUIDE_COLLECTION)
    if collection is None:
        collection = bpy.data.collections.new(GUIDE_COLLECTION)
        bpy.context.scene.collection.children.link(collection)
    elif collection.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(collection)
    collection.hide_viewport = False
    collection.hide_render = True
    return collection


def remove_driver(owner, data_path, axis=None):
    try:
        if axis is None:
            owner.driver_remove(data_path)
        else:
            owner.driver_remove(data_path, axis)
    except (TypeError, RuntimeError):
        pass


def single_prop(driver, variable_name, id_block, data_path):
    variable = driver.variables.new()
    variable.name = variable_name
    variable.type = "SINGLE_PROP"
    target = variable.targets[0]
    if isinstance(id_block, bpy.types.Curve):
        target.id_type = "CURVE"
    else:
        target.id_type = "OBJECT"
    target.id = id_block
    target.data_path = data_path


def prop_path(name):
    return f'["{name}"]'


def ensure_number_property(name, default, description, subtype="NONE",
                           soft_min=-10.0, soft_max=10.0, hard_min=None,
                           hard_max=None):
    if name not in master:
        master[name] = float(default)
    settings = {
        "description": description,
        "soft_min": soft_min,
        "soft_max": soft_max,
    }
    if subtype != "NONE":
        settings["subtype"] = subtype
    if hard_min is not None:
        settings["min"] = hard_min
    if hard_max is not None:
        settings["max"] = hard_max
    master.id_properties_ui(name).update(**settings)


def ensure_guide(curve_object, guide_name, color):
    guide = bpy.data.objects.get(guide_name)
    if guide is None:
        guide_data = curve_object.data.copy()
        guide_data.name = "DATA_" + guide_name
        guide = bpy.data.objects.new(guide_name, guide_data)
        ensure_collection().objects.link(guide)
        guide.matrix_world = curve_object.matrix_world.copy()
        guide["Rope Artist Source"] = curve_object.name
        guide["Rope Artist Draw Guide"] = True
        # The guide is a control line, never another rendered rope.
        guide_data.bevel_depth = 0.0
        guide_data.bevel_resolution = 0
        guide_data.extrude = 0.0
        guide_data.offset = 0.0
        for spline in guide_data.splines:
            if spline.type == "BEZIER":
                for point in spline.bezier_points:
                    # Preserve the current coordinates while allowing direct
                    # independent handle drawing/editing.
                    left = point.handle_left.copy()
                    right = point.handle_right.copy()
                    point.handle_left_type = "FREE"
                    point.handle_right_type = "FREE"
                    point.handle_left = left
                    point.handle_right = right
    elif guide.type != "CURVE":
        fail(f"{guide_name} exists but is not a Curve")
    guide.hide_render = True
    guide.hide_viewport = False
    guide.hide_select = False
    guide.hide_set(False)
    guide.show_in_front = True
    guide.display_type = "WIRE"
    guide.color = color
    return guide


def point_key(side, index, label):
    return f"{side} P{index:02d} | {label}"


def add_guide_component(driver, name, guide_data, path, axis):
    single_prop(driver, name, guide_data, f"{path}[{axis}]")


def drive_point_coordinate(real_point, guide_data, spline_index, point_index,
                           side, axis):
    data_path = "co"
    remove_driver(real_point, data_path, axis)
    fcurve = real_point.driver_add(data_path, axis)
    driver = fcurve.driver
    driver.type = "SCRIPTED"
    guide_path = f"splines[{spline_index}].bezier_points[{point_index}].co"
    add_guide_component(driver, "g", guide_data, guide_path, axis)
    prop = point_key(side, point_index, f"Loc {'XYZ'[axis]}")
    single_prop(driver, "o", master, prop_path(prop))
    driver.expression = "g+o"


def rotated_component_expression(axis, vector_prefix):
    # ZYX Euler rotation of the guide handle vector around its point.
    vx, vy, vz = (f"{vector_prefix}{letter}" for letter in "xyz")
    x1 = vx
    y1 = f"(({vy})*cos(rx)-({vz})*sin(rx))"
    z1 = f"(({vy})*sin(rx)+({vz})*cos(rx))"
    x2 = f"(({x1})*cos(ry)+({z1})*sin(ry))"
    y2 = y1
    z2 = f"(-({x1})*sin(ry)+({z1})*cos(ry))"
    if axis == 0:
        return f"(({x2})*cos(rz)-({y2})*sin(rz))"
    if axis == 1:
        return f"(({x2})*sin(rz)+({y2})*cos(rz))"
    return z2


def drive_handle_coordinate(real_point, guide_data, spline_index, point_index,
                            side, handle_side, axis):
    attribute = "handle_left" if handle_side == "L" else "handle_right"
    remove_driver(real_point, attribute, axis)
    fcurve = real_point.driver_add(attribute, axis)
    driver = fcurve.driver
    driver.type = "SCRIPTED"
    base = f"splines[{spline_index}].bezier_points[{point_index}]"
    for component, letter in enumerate("xyz"):
        add_guide_component(driver, f"c{letter}", guide_data, base + ".co", component)
        add_guide_component(driver, f"h{letter}", guide_data, base + f".{attribute}", component)
    for component, letter in enumerate("xyz"):
        prop = point_key(side, point_index, f"Rot {'XYZ'[component]}")
        single_prop(driver, f"r{letter}", master, prop_path(prop))
    loc_prop = point_key(side, point_index, f"Loc {'XYZ'[axis]}")
    offset_prop = point_key(
        side, point_index, f"Handle {handle_side} {'XYZ'[axis]}")
    single_prop(driver, "lo", master, prop_path(loc_prop))
    single_prop(driver, "ho", master, prop_path(offset_prop))
    rotated = rotated_component_expression(axis, "v")
    # Replace vector aliases with the guide handle-minus-point values.
    rotated = (rotated.replace("vx", "(hx-cx)")
                        .replace("vy", "(hy-cy)")
                        .replace("vz", "(hz-cz)"))
    center = "xyz"[axis]
    driver.expression = f"c{center}+lo+({rotated})+ho"


def drive_scalar(real_point, guide_data, spline_index, point_index,
                 side, attribute, label, expression):
    remove_driver(real_point, attribute)
    fcurve = real_point.driver_add(attribute)
    driver = fcurve.driver
    driver.type = "SCRIPTED"
    path = f"splines[{spline_index}].bezier_points[{point_index}].{attribute}"
    single_prop(driver, "g", guide_data, path)
    single_prop(driver, "o", master, prop_path(point_key(side, point_index, label)))
    driver.expression = expression


def mute_artist_drivers(curve_object, muted):
    animation = curve_object.data.animation_data
    if animation is None:
        return
    for fcurve in animation.drivers:
        is_artist = False
        for variable in fcurve.driver.variables:
            for target in variable.targets:
                target_id = getattr(target, "id", None)
                if (target_id == master or (
                        target_id is not None
                        and target_id.name.startswith("DATA_DRAW_ROPE_"))):
                    is_artist = True
                    break
            if is_artist:
                break
        if is_artist:
            fcurve.mute = muted


def set_original_hooks(curve_object, enabled):
    if "Rope Artist Original Hooks" not in curve_object:
        curve_object["Rope Artist Original Hooks"] = json.dumps([
            modifier.name for modifier in curve_object.modifiers
            if modifier.type == "HOOK"
        ])
    hook_names = set(json.loads(curve_object["Rope Artist Original Hooks"]))
    for modifier in curve_object.modifiers:
        if modifier.type == "HOOK" and modifier.name in hook_names:
            modifier.show_viewport = enabled
            modifier.show_render = enabled


def configure_curve(side, curve_name, guide_name, color):
    curve_object = bpy.data.objects.get(curve_name)
    if curve_object is None or curve_object.type != "CURVE":
        fail(f"required Curve is missing: {curve_name}")
    guide = ensure_guide(curve_object, guide_name, color)
    if len(guide.data.splines) != len(curve_object.data.splines):
        fail(f"guide spline count changed; restore or recreate {guide_name}")

    point_total = 0
    for spline_index, (real_spline, guide_spline) in enumerate(zip(
            curve_object.data.splines, guide.data.splines)):
        if real_spline.type != "BEZIER" or guide_spline.type != "BEZIER":
            fail(f"{curve_name} and its guide must contain Bezier splines")
        if len(real_spline.bezier_points) != len(guide_spline.bezier_points):
            fail(
                f"{guide_name} now has a different point count. "
                "Undo added/deleted points, or start from the saved backup.")
        for point_index, real_point in enumerate(real_spline.bezier_points):
            for axis, letter in enumerate("XYZ"):
                ensure_number_property(
                    point_key(side, point_index, f"Loc {letter}"), 0.0,
                    f"Additive local {letter} position for {side} point {point_index}",
                    "DISTANCE", -2.0, 2.0)
                ensure_number_property(
                    point_key(side, point_index, f"Rot {letter}"), 0.0,
                    f"Rotate both handles around {side} point {point_index}",
                    "ANGLE", -3.14159265, 3.14159265)
                ensure_number_property(
                    point_key(side, point_index, f"Handle L {letter}"), 0.0,
                    f"Fine offset for the left handle {letter}",
                    "DISTANCE", -2.0, 2.0)
                ensure_number_property(
                    point_key(side, point_index, f"Handle R {letter}"), 0.0,
                    f"Fine offset for the right handle {letter}",
                    "DISTANCE", -2.0, 2.0)
            ensure_number_property(
                point_key(side, point_index, "Tilt"), 0.0,
                f"Additive rope roll/tilt for {side} point {point_index}",
                "ANGLE", -3.14159265, 3.14159265)
            ensure_number_property(
                point_key(side, point_index, "Radius Scale"), 1.0,
                f"Cross-section radius multiplier for {side} point {point_index}",
                "NONE", 0.1, 2.0, 0.001, 10.0)

            left = real_point.handle_left.copy()
            right = real_point.handle_right.copy()
            real_point.handle_left_type = "FREE"
            real_point.handle_right_type = "FREE"
            real_point.handle_left = left
            real_point.handle_right = right
            for axis in range(3):
                drive_point_coordinate(
                    real_point, guide.data, spline_index, point_index, side, axis)
                drive_handle_coordinate(
                    real_point, guide.data, spline_index, point_index,
                    side, "L", axis)
                drive_handle_coordinate(
                    real_point, guide.data, spline_index, point_index,
                    side, "R", axis)
            drive_scalar(
                real_point, guide.data, spline_index, point_index,
                side, "tilt", "Tilt", "g+o")
            drive_scalar(
                real_point, guide.data, spline_index, point_index,
                side, "radius", "Radius Scale", "max(0.001,g*o)")
            point_total += 1

    set_original_hooks(curve_object, False)
    mute_artist_drivers(curve_object, False)
    curve_object["Rope Artist Draw Guide"] = guide.name
    curve_object["Rope Artist Empty"] = master.name
    curve_object["Rope Artist Mode"] = True
    curve_object.data.update_tag()
    curve_object.update_tag(refresh={"OBJECT", "DATA"})
    return guide, point_total


if MODE == "RESTORE_ORIGINAL_RIG":
    restored = []
    for side, (curve_name, guide_name, color) in CURVES.items():
        curve_object = bpy.data.objects.get(curve_name)
        if curve_object is None:
            continue
        mute_artist_drivers(curve_object, True)
        set_original_hooks(curve_object, True)
        curve_object["Rope Artist Mode"] = False
        restored.append(curve_name)
    bpy.context.view_layer.update()
    print("ROPE_ARTIST_ORIGINAL_RIG_RESTORED::" + ";".join(restored))
else:
    results = []
    guides = []
    for side, config in CURVES.items():
        guide, count = configure_curve(side, *config)
        guides.append(guide)
        results.append(f"{side}:{count} points")

    master["Rope Artist Instructions"] = (
        "Edit DRAW_ROPE_SOUTH_TOP_CURVE / DRAW_ROPE_WEST_TOP_CURVE with "
        "Curve Pen or Edit Mode. Use the SOUTH/WEST P## properties here for "
        "position, handle rotation, handle offsets, tilt and radius.")
    master["Rope Artist Mode"] = True
    master.update_tag(refresh={"OBJECT"})
    bpy.context.view_layer.update()

    # Leave the master selected so its new properties are immediately visible.
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for selected in tuple(bpy.context.selected_objects):
        selected.select_set(False)
    master.hide_set(False)
    master.select_set(True)
    bpy.context.view_layer.objects.active = master

    print(
        "ROPE_ARTIST_EMPTY_PROPS_AND_DRAW_GUIDES_READY::"
        + ";".join(results)
        + "; guides=" + ",".join(guide.name for guide in guides))

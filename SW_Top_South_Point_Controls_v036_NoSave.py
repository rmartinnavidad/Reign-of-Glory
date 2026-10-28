"""Add live, movable controls to all 10 South and West top rope Bezier points.

Run on the clean four-point scene or an existing v018-v035 ten-point scene.
On a clean scene, this runs v018 first, not v035. This changes memory only.
The existing TR rope props still move their anchor controls; these new controls
follow those anchors and can also be moved/rotated directly. Their artist offset
properties live on the turnbuckle master.
"""

import bpy
import json
import runpy
from pathlib import Path
from bisect import bisect_left
from math import atan2, cos, sin, pi
from mathutils import Vector, Matrix, Euler

MASTER_NAME = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
CURVE_NAME = "ROPE_SOUTH_TOP_CURVE"
COLLECTION_NAME = "CTRL - South Top Curve Points v036"
MARKER = "LSO South Point Controls v036"
TIMER_KEY = "LSO_SOUTH_POINT_CONTROLS_V036_TIMER"
WEST_CURVE_NAME = "ROPE_WEST_TOP_CURVE"
WEST_COLLECTION_NAME = "CTRL - West Top Curve Points v036"
WEST_MARKER = "LSO West Point Controls v036"
WEST_DRAPE_MARKER = "LSO West Floor Drape v036"
V18 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\Add_Turnbuckle_Unit_Hybrid_v018_CanvasSafe_Cable.py")
ANCHORS = (
    "CTRL_Rope Point_TR_S_end_r",
    "CTRL_Rope Point_TR_S_mid",
    "CTRL_Rope Point_TR_S_end_l",
    "CTRL_Rope Point_TR_S_end_l_ctrl",
)
MESH_CONTACT_MODIFIER = "LSO Canvas Contact - Visible Ropes"
MESH_CONTACT_GROUP = "LSO Canvas Contact - Visible Ropes v036"
ARTIST_CAPTURE = Path(r"C:\Users\rmart\Reign of Glory\blender\arena\Object Captures\wrestling ring_INTENTED_ROPE_SOUTH_TOP_CURVE_X-50_z175.py")
ARTIST_MARKER = "LSO South Artist Drape v036"
NE_FLOOR_CONSTRAINT = "LSO Canvas Contact - Top Midpoint"


def install_west_point_controls(master):
    """Replace West's blended Hooks with ten independent, editable point Hooks."""
    curve = bpy.data.objects.get(WEST_CURVE_NAME)
    require(curve is not None and curve.type == "CURVE", "missing West top curve")
    require(len(curve.data.splines) == 1 and
            len(curve.data.splines[0].bezier_points) == 10,
            "expected one ten-point West Bezier spline")
    if curve.get(WEST_MARKER):
        require(all(bpy.data.objects.get(f"CTRL_Rope_TR_W_Curve_P{i:02d}")
                    for i in range(10)), "West point controls are incomplete")
        return
    anchors = [bpy.data.objects.get(name) for name in (
        "CTRL_Rope Point_TR_W_end_r", "CTRL_Rope Point_TR_W_mid",
        "CTRL_Rope Point_TR_W_end_l", "CTRL_Rope Point_TR_W_end_l_ctrl")]
    require(all(a is not None for a in anchors), "missing West anchor control")
    old_hooks = [m for m in curve.modifiers if m.type == "HOOK"]
    require(len(old_hooks) == 16, "expected the 16 original/blended West Hooks")
    require(all(bpy.data.objects.get(f"CTRL_Rope_TR_W_Curve_P{i:02d}") is None
                for i in range(10)), "a West point control already exists")
    canvas = bpy.data.objects.get("MESH_Ring Canvas")
    require(canvas is not None and canvas.type == "MESH", "missing ring canvas")
    canvas_top = max((canvas.matrix_world @ Vector(c)).z for c in canvas.bound_box)
    saved_location, saved_rotation = master.location.copy(), master.rotation_euler.copy()
    try:
        master.location = Vector(tuple(float(master[f"SW_TR_TAUT_CONTROL_{a}"])
                                       for a in "XYZ"))
        master.rotation_euler = tuple(float(master[f"SW_TR_DEFAULT_ROTATION_{a}"])
                                      for a in "XYZ")
        for _ in range(3):
            bpy.context.view_layer.update()
        collection = bpy.data.collections.new(WEST_COLLECTION_NAME)
        bpy.context.scene.collection.children.link(collection)
        points = curve.data.splines[0].bezier_points
        positions = [curve.matrix_world @ p.co for p in points]
        anchor_positions = [a.matrix_world.translation.copy() for a in anchors]
        # Original four point indices after two subdivisions: 0, 4, 8, 9.
        for modifier in old_hooks:
            curve.modifiers.remove(modifier)
        for i, point in enumerate(points):
            if i <= 4:
                left, right, weight = 0, 1, i / 4.0
            elif i <= 8:
                left, right, weight = 1, 2, (i-4) / 4.0
            else:
                left, right, weight = 3, 3, 0.0
            a, b = anchors[left], anchors[right]
            correction = positions[i] - anchor_positions[left].lerp(
                anchor_positions[right], weight)
            base = bpy.data.objects.new(f"BASE_Rope_TR_W_Curve_P{i:02d}", None)
            collection.objects.link(base)
            base.location = positions[i]
            base.empty_display_type = "PLAIN_AXES"
            base.empty_display_size = .035
            base.hide_set(True)
            base.hide_render = True
            handle_min_delta = min(
                (curve.matrix_world @ h).z - positions[i].z
                for h in (point.co, point.handle_left, point.handle_right))
            for axis, name in enumerate("XYZ"):
                fc = base.driver_add("location", axis)
                driver = fc.driver
                driver.type = "SCRIPTED"
                add_transform_variable(driver, "a", a, name)
                if left != right:
                    add_transform_variable(driver, "b", b, name)
                    expression = (f"a*{1-weight:.9g}+b*{weight:.9g}"
                                  f"+({correction[axis]:.12g})")
                else:
                    expression = f"a+({correction[axis]:.12g})"
                if axis == 2 and i not in (0, 8, 9):
                    # Clamp the actual Hook target, including its low handle.
                    # The curve and Spline IK thus remain above the canvas;
                    # the mesh-level contact modifier is only a backstop.
                    expression = (f"max(({expression}),"
                                  f"{canvas_top + .070 - handle_min_delta:.12g})")
                driver.expression = expression
            control = bpy.data.objects.new(f"CTRL_Rope_TR_W_Curve_P{i:02d}", None)
            collection.objects.link(control)
            control.parent = base
            control.empty_display_type = "SPHERE"
            control.empty_display_size = .085
            control.show_in_front = True
            control.show_name = True
            control.hide_render = True
            control["West Bezier Point"] = i
            control["Move this Empty or use the matching master Offset properties"] = True
            offset = bpy.data.objects.new(f"OFFSET_Rope_TR_W_Curve_P{i:02d}", None)
            collection.objects.link(offset)
            offset.parent = control
            offset.empty_display_type = "PLAIN_AXES"
            offset.empty_display_size = .025
            offset.hide_render = True
            for name in "XYZ":
                location_prop = f"{control.name} | Offset Loc {name}"
                rotation_prop = f"{control.name} | Offset Rot {name}"
                master[location_prop] = 0.0
                master[rotation_prop] = 0.0
                master.id_properties_ui(location_prop).update(subtype="DISTANCE")
                master.id_properties_ui(rotation_prop).update(subtype="ANGLE")
            bpy.context.view_layer.update()
            hook = curve.modifiers.new(f"LSO West Artist P{i:02d}", "HOOK")
            hook.object = offset
            hook.strength = 1.0
            hook.falloff_type = "NONE"
            hook.center = point.co.copy()
            hook.vertex_indices_set((i*3, i*3+1, i*3+2))
            hook.matrix_inverse = offset.matrix_world.inverted() @ curve.matrix_world
        bpy.context.view_layer.objects.active = curve
        with bpy.context.temp_override(object=curve, active_object=curve):
            for i in range(10):
                bpy.ops.object.modifier_move_to_index(
                    modifier=f"LSO West Artist P{i:02d}", index=i)
        for obj in collection.objects:
            if obj.animation_data:
                for fc in obj.animation_data.drivers:
                    fc.driver.expression = f"({fc.driver.expression})"
                obj.update_tag(refresh={"OBJECT"})
        curve[WEST_MARKER] = True
        curve.data.update_tag()
    finally:
        master.location = saved_location
        master.rotation_euler = saved_rotation
        bpy.context.view_layer.update()


def install_west_floor_drape(master, curve):
    """Give West its own smooth, floor-following path at the released pose."""
    if curve.get(WEST_DRAPE_MARKER):
        return
    require(all(bpy.data.objects.get(f"BASE_Rope_TR_W_Curve_P{i:02d}")
                for i in range(10)), "West point bases are missing")
    saved_location, saved_rotation = master.location.copy(), master.rotation_euler.copy()
    rest = Vector(tuple(float(master[f"SW_TR_TAUT_CONTROL_{a}"]) for a in "XYZ"))
    rotation = Euler(tuple(float(master[f"SW_TR_DEFAULT_ROTATION_{a}"])
                           for a in "XYZ"), "XYZ")
    reference = rest + rotation.to_matrix() @ Vector((-1.27, 0, 4.445))
    canvas = bpy.data.objects.get("MESH_Ring Canvas")
    require(canvas is not None and canvas.type == "MESH", "missing ring canvas")
    floor = max((canvas.matrix_world @ Vector(c)).z for c in canvas.bound_box) + .105
    def smooth(value):
        value = min(1.0, max(0.0, value))
        return value*value*(3-2*value)
    try:
        master.location = reference
        master.rotation_euler = rotation
        for _ in range(3):
            bpy.context.view_layer.update()
        baseline = [bpy.data.objects[f"BASE_Rope_TR_W_Curve_P{i:02d}"].matrix_world.translation.copy()
                    for i in range(9)]
        chord = baseline[8]-baseline[0]
        planar = Vector((-chord.y, chord.x, 0.0))
        require(planar.length > .1, "West floor-pose chord is invalid")
        planar.normalize()
        center = (baseline[0]+baseline[8])*.5
        if (center+planar*.2).xy.length > (center-planar*.2).xy.length:
            planar.negate()  # Bow the slack toward the ring interior.
        desired = []
        for i, old in enumerate(baseline):
            t = i/8.0
            point = old.copy()
            # Unlike South's drawn profile, West has its own broad sideways
            # sweep and settles onto the canvas before the far eyehook.
            point += planar * (.24*sin(pi*t) + .035*sin(2*pi*t))
            if 0 < i < 8:
                point.z = (floor
                           + max(0.0, baseline[0].z-floor)*(1-smooth(t/.42))
                           + max(0.0, baseline[8].z-floor)*smooth((t-.74)/.26))
            desired.append(point)
        direction = reference-rest
        dot = '+'.join(f'(m{letter}-{rest[k]:.6g})*{direction[k]:.6g}'
                       for k, letter in enumerate('xyz'))
        weight = f'max(0,min(1,({dot})/{direction.length_squared:.6g}))'
        for i in range(9):
            base = bpy.data.objects[f"BASE_Rope_TR_W_Curve_P{i:02d}"]
            control = bpy.data.objects[f"CTRL_Rope_TR_W_Curve_P{i:02d}"]
            delta = desired[i]-baseline[i]
            for axis in range(3):
                driver = next(f for f in base.animation_data.drivers
                              if f.data_path == 'location' and f.array_index == axis).driver
                expression = driver.expression
                for letter, axis_name in zip('xyz', 'XYZ'):
                    add_transform_variable(driver, 'm'+letter, master, axis_name)
                driver.expression = f'({expression})+({delta[axis]:.12g})*({weight})'
            spline_point = curve.data.splines[0].bezier_points[i]
            old_tangent = curve.matrix_world.to_3x3() @ (
                spline_point.handle_right-spline_point.handle_left)
            new_tangent = desired[min(8, i+1)]-desired[max(0, i-1)]
            if old_tangent.length > 1e-5 and new_tangent.length > 1e-5:
                angles = old_tangent.rotation_difference(new_tangent).to_euler('XYZ')
                for axis in range(3):
                    fc = control.driver_add('rotation_euler', axis)
                    driver = fc.driver
                    driver.type = 'SCRIPTED'
                    for letter, axis_name in zip('xyz', 'XYZ'):
                        add_transform_variable(driver, 'm'+letter, master, axis_name)
                    driver.expression = f'({angles[axis]:.12g})*({weight})'
        curve[WEST_DRAPE_MARKER] = True
        curve.data.update_tag()
    finally:
        master.location = saved_location
        master.rotation_euler = saved_rotation
        bpy.context.view_layer.update()


def install_north_east_curve_contact():
    """Limit the actual Spline-IK Hook targets, not only the visible mesh."""
    canvas = bpy.data.objects.get("MESH_Ring Canvas")
    require(canvas is not None and canvas.type == "MESH", "missing ring canvas")
    top = max((canvas.matrix_world @ Vector(c)).z for c in canvas.bound_box)
    for side, letter, point_index, expected_points in (
            ("NORTH", "N", 1, 4),
            ("EAST", "E", 1, 4)):
        curve = bpy.data.objects.get(f"ROPE_{side}_TOP_CURVE")
        middle = bpy.data.objects.get(f"CTRL_Rope Point_TR_{letter}_mid")
        require(curve is not None and curve.type == "CURVE", f"missing {side} top curve")
        require(middle is not None and middle.type == "EMPTY", f"missing {side} midpoint")
        require(len(curve.data.splines) == 1 and
                len(curve.data.splines[0].bezier_points) == expected_points,
                f"unexpected {side} top curve topology")
        point = curve.data.splines[0].bezier_points[point_index]
        center_z = (curve.matrix_world @ point.co).z
        lowest_handle_delta = min((curve.matrix_world @ h).z-center_z
                                  for h in (point.co, point.handle_left, point.handle_right))
        # If all Hook-driven Bezier vertices remain above this value, the
        # segment between them also stays above the mat (convex-hull property).
        minimum = top + .070 - lowest_handle_delta
        constraint = middle.constraints.get(NE_FLOOR_CONSTRAINT)
        if constraint is None:
            constraint = middle.constraints.new("LIMIT_LOCATION")
            constraint.name = NE_FLOOR_CONSTRAINT
        constraint.owner_space = "WORLD"
        constraint.use_min_z = True
        constraint.min_z = minimum
        constraint.use_max_z = False
        constraint.influence = 1.0
        constraint.mute = False
        middle.update_tag(refresh={"OBJECT"})
        curve["LSO Canvas Contact Minimum Midpoint Z"] = minimum



def artist_world_samples(count):
    require(ARTIST_CAPTURE.is_file(), "missing artist-drawn South reference")
    source = ARTIST_CAPTURE.read_text(encoding="utf-8")
    require('CAPTURE = json.loads(r"""' in source, "artist capture format changed")
    payload = source.split('CAPTURE = json.loads(r"""', 1)[1].split('""")', 1)[0]
    capture = json.loads(payload)
    name = "INTENTED_ROPE_SOUTH_TOP_CURVE_X-50_z175"
    artist = next((obj for obj in capture["objects"] if obj["name"] == name), None)
    require(artist is not None, "artist South curve missing from capture")
    points = artist["data"]["splines"][0]["bezier_points"]
    require(len(points) == 11, "artist South curve is not the 11-point reference")
    world = Matrix(artist["matrix_world"])
    dense = []
    for left, right in zip(points, points[1:]):
        p0, h0 = Vector(left["co"]), Vector(left["handle_right"])
        h1, p1 = Vector(right["handle_left"]), Vector(right["co"])
        for j in range(25):
            t = j / 24.0
            value = ((1-t)**3*p0 + 3*(1-t)**2*t*h0
                     + 3*(1-t)*t*t*h1 + t**3*p1)
            if dense and j == 0:
                continue
            dense.append(world @ value)
    lengths = [0.0]
    for a, b in zip(dense, dense[1:]):
        lengths.append(lengths[-1] + (b-a).length)
    require(lengths[-1] > 0.1, "artist South curve has zero length")
    result = []
    for i in range(count):
        distance = lengths[-1] * i / (count-1)
        k = max(1, min(len(lengths)-1, bisect_left(lengths, distance)))
        fraction = (distance-lengths[k-1]) / max(1e-12, lengths[k]-lengths[k-1])
        result.append(dense[k-1].lerp(dense[k], fraction))
    return result


def install_artist_south_drape(master, curve):
    """Add a zero-at-rest drape to the ten Hook targets and their tangents."""
    if curve.get(ARTIST_MARKER):
        return
    require(all(bpy.data.objects.get(f"BASE_Rope_TR_S_Curve_P{i:02d}") for i in range(10)),
            "South point bases are missing")
    saved_location = master.location.copy()
    saved_rotation = master.rotation_euler.copy()
    rest = Vector(tuple(float(master[f"SW_TR_TAUT_CONTROL_{a}"]) for a in "XYZ"))
    rotation = Euler(tuple(float(master[f"SW_TR_DEFAULT_ROTATION_{a}"])
                           for a in "XYZ"), "XYZ")
    reference = rest + rotation.to_matrix() @ Vector((-1.27, 0, 4.445))
    try:
        master.location = reference
        master.rotation_euler = rotation
        for _ in range(3):
            bpy.context.view_layer.update()
        baseline = [bpy.data.objects[f"BASE_Rope_TR_S_Curve_P{i:02d}"].matrix_world.translation.copy()
                    for i in range(9)]
        artist = artist_world_samples(9)
        start_a, end_a = artist[0], artist[-1]
        start_b, end_b = baseline[0], baseline[-1]
        a_chord, b_chord = end_a-start_a, end_b-start_b
        a_planar, b_planar = a_chord.xy.length, b_chord.xy.length
        require(a_planar > .1 and b_planar > .1, "South reference endpoint span is invalid")
        yaw = atan2(b_chord.y, b_chord.x) - atan2(a_chord.y, a_chord.x)
        planar_scale = b_planar / a_planar
        vertical_scale = b_chord.z / a_chord.z if abs(a_chord.z) > .1 else 1.0
        def fit(p):
            d = p-start_a
            return Vector((start_b.x + planar_scale*(cos(yaw)*d.x-sin(yaw)*d.y),
                           start_b.y + planar_scale*(sin(yaw)*d.x+cos(yaw)*d.y),
                           start_b.z + vertical_scale*d.z))
        desired = [fit(p) for p in artist]
        canvas = bpy.data.objects.get("MESH_Ring Canvas")
        require(canvas is not None and canvas.type == "MESH", "missing ring canvas")
        canvas_top = max((canvas.matrix_world @ Vector(c)).z for c in canvas.bound_box)
        # The artist drew the centerline on the mat surface. Give the actual
        # rope and Spline-IK bones thickness above it, not just the visible
        # mesh a post-deformation clipping plane.
        for point in desired[1:-1]:
            point.z = max(point.z, canvas_top + .070)
        # Read the master's evaluated motion, so either direct dragging or
        # the existing zero-based Offset properties gives the same result.
        direction = reference-rest
        dot = '+'.join(f'(m{letter}-{rest[k]:.6g})*{direction[k]:.6g}'
                       for k, letter in enumerate('xyz'))
        weight = f'max(0,min(1,({dot})/{direction.length_squared:.6g}))'
        for i in range(9):
            base = bpy.data.objects[f"BASE_Rope_TR_S_Curve_P{i:02d}"]
            control = bpy.data.objects[f"CTRL_Rope_TR_S_Curve_P{i:02d}"]
            delta = desired[i]-baseline[i]
            for axis in range(3):
                driver = next(f for f in base.animation_data.drivers
                              if f.data_path == 'location' and f.array_index == axis).driver
                expression = driver.expression
                for letter, axis_name in zip('xyz', 'XYZ'):
                    add_transform_variable(driver, 'm'+letter, master, axis_name)
                driver.expression = f'({expression})+({delta[axis]:.12g})*({weight})'
            # Hook rotation turns each Bezier tangent toward the authored path,
            # avoiding the sharp, straight handle directions of the old rig.
            spline_point = curve.data.splines[0].bezier_points[i]
            old_tangent = curve.matrix_world.to_3x3() @ (
                spline_point.handle_right-spline_point.handle_left)
            new_tangent = (desired[min(8, i+1)]-desired[max(0, i-1)])
            if old_tangent.length > 1e-5 and new_tangent.length > 1e-5:
                angles = old_tangent.rotation_difference(new_tangent).to_euler('XYZ')
                for axis in range(3):
                    fc = control.driver_add('rotation_euler', axis)
                    driver = fc.driver
                    driver.type = 'SCRIPTED'
                    for letter, axis_name in zip('xyz', 'XYZ'):
                        add_transform_variable(driver, 'm'+letter, master, axis_name)
                    driver.expression = f'({angles[axis]:.12g})*({weight})'
        curve[ARTIST_MARKER] = True
        curve.data.update_tag()
    finally:
        master.location = saved_location
        master.rotation_euler = saved_rotation
        bpy.context.view_layer.update()


def install_visible_canvas_contact():
    """Project only below-canvas mesh vertices up after armature deformation."""
    mesh = bpy.data.objects.get("MESH_Ring Ropes")
    canvas = bpy.data.objects.get("MESH_Ring Canvas")
    require(mesh is not None and mesh.type == "MESH", "missing rope mesh")
    require(canvas is not None and canvas.type == "MESH", "missing ring canvas")
    corners = [mesh.matrix_world.inverted() @ (canvas.matrix_world @ Vector(c))
               for c in canvas.bound_box]
    min_x, max_x = min(p.x for p in corners), max(p.x for p in corners)
    min_y, max_y = min(p.y for p in corners), max(p.y for p in corners)
    floor = max(p.z for p in corners) + .004
    group = bpy.data.node_groups.get(MESH_CONTACT_GROUP)
    if group is None:
        group = bpy.data.node_groups.new(MESH_CONTACT_GROUP, "GeometryNodeTree")
        group.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
        group.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    else:
        group.nodes.clear()
    nodes, links = group.nodes, group.links
    inp = nodes.new("NodeGroupInput")
    out = nodes.new("NodeGroupOutput")
    pos = nodes.new("GeometryNodeInputPosition")
    xyz = nodes.new("ShaderNodeSeparateXYZ")
    links.new(pos.outputs["Position"], xyz.inputs["Vector"])

    def math(operation, a, b=None):
        n = nodes.new("ShaderNodeMath")
        n.operation = operation
        links.new(a, n.inputs[0])
        if b is not None:
            if isinstance(b, (int, float)):
                n.inputs[1].default_value = b
            else:
                links.new(b, n.inputs[1])
        return n.outputs[0]

    inside_x0 = math("GREATER_THAN", xyz.outputs["X"], min_x)
    inside_x1 = math("LESS_THAN", xyz.outputs["X"], max_x)
    inside_y0 = math("GREATER_THAN", xyz.outputs["Y"], min_y)
    inside_y1 = math("LESS_THAN", xyz.outputs["Y"], max_y)
    inside = math("MULTIPLY", math("MULTIPLY", inside_x0, inside_x1),
                  math("MULTIPLY", inside_y0, inside_y1))
    clamped = math("MAXIMUM", xyz.outputs["Z"], floor)
    delta = math("SUBTRACT", clamped, xyz.outputs["Z"])
    final_z = math("ADD", xyz.outputs["Z"], math("MULTIPLY", delta, inside))
    joined = nodes.new("ShaderNodeCombineXYZ")
    links.new(xyz.outputs["X"], joined.inputs["X"])
    links.new(xyz.outputs["Y"], joined.inputs["Y"])
    links.new(final_z, joined.inputs["Z"])
    project = nodes.new("GeometryNodeSetPosition")
    links.new(inp.outputs["Geometry"], project.inputs["Geometry"])
    links.new(joined.outputs["Vector"], project.inputs["Position"])
    links.new(project.outputs["Geometry"], out.inputs["Geometry"])
    modifier = mesh.modifiers.get(MESH_CONTACT_MODIFIER)
    if modifier is None:
        modifier = mesh.modifiers.new(MESH_CONTACT_MODIFIER, "NODES")
    modifier.node_group = group
    # This must run after the rope armature, on the actual visible geometry.
    bpy.context.view_layer.objects.active = mesh
    with bpy.context.temp_override(object=mesh, active_object=mesh):
        bpy.ops.object.modifier_move_to_index(modifier=modifier.name,
                                               index=len(mesh.modifiers) - 1)
    mesh["LSO Canvas Contact World Z"] = max(
        (canvas.matrix_world @ Vector(c)).z for c in canvas.bound_box) + .004
    mesh.update_tag(refresh={"OBJECT"})


def require(condition, message):
    if not condition:
        raise RuntimeError("South top controls: " + message + "; no changes made")


def add_transform_variable(driver, name, obj, axis):
    var = driver.variables.new()
    var.name = name
    var.type = "TRANSFORMS"
    target = var.targets[0]
    target.id = obj
    target.transform_type = "LOC_" + axis
    target.transform_space = "WORLD_SPACE"


def apply_property_offsets():
    master = bpy.data.objects.get(MASTER_NAME)
    if master is None:
        return
    for side in "SW":
        for i in range(10):
            offset = bpy.data.objects.get(f"OFFSET_Rope_TR_{side}_Curve_P{i:02d}")
            if offset is None:
                continue
            name = f"CTRL_Rope_TR_{side}_Curve_P{i:02d}"
            position = tuple(float(master.get(f"{name} | Offset Loc {axis}", 0.0))
                             for axis in "XYZ")
            rotation = tuple(float(master.get(f"{name} | Offset Rot {axis}", 0.0))
                             for axis in "XYZ")
            if (offset.location - Vector(position)).length > 1e-8:
                offset.location = position
                offset.update_tag(refresh={"OBJECT"})
            if any(abs(offset.rotation_euler[k] - rotation[k]) > 1e-8
                   for k in range(3)):
                offset.rotation_euler = rotation
                offset.update_tag(refresh={"OBJECT"})


def install_timer():
    old = bpy.app.driver_namespace.get(TIMER_KEY)
    if old is not None and bpy.app.timers.is_registered(old):
        bpy.app.timers.unregister(old)

    def tick():
        try:
            apply_property_offsets()
        except Exception as error:
            print("South point offset update error:", error)
        return .05

    bpy.app.driver_namespace[TIMER_KEY] = tick
    bpy.app.timers.register(tick, first_interval=.05, persistent=True)


def build():
    master = bpy.data.objects.get(MASTER_NAME)
    curve = bpy.data.objects.get(CURVE_NAME)
    require(curve is not None and curve.type == "CURVE", "missing South curve")
    if not curve.get(ARTIST_MARKER):
        artist_world_samples(9)  # Validate the reference before changing any rig data.
    if master is None and len(curve.data.splines) == 1 and len(curve.data.splines[0].bezier_points) == 4:
        require(V18.is_file(), "required v018 base script is missing")
        runpy.run_path(str(V18), run_name="__main__")
        master = bpy.data.objects.get(MASTER_NAME)
        curve = bpy.data.objects.get(CURVE_NAME)
    require(master is not None and master.type == "EMPTY", "missing master Empty")
    require(len(curve.data.splines) == 1 and
            len(curve.data.splines[0].bezier_points) == 10,
            "expected one ten-point South Bezier spline")
    require(curve.modifiers.get("LSO Canvas Safety • Preserve Hooks") is not None,
            "v018 or later South curve setup is not present")
    if curve.get(MARKER):
        install_artist_south_drape(master, curve)
        install_west_point_controls(master)
        install_west_floor_drape(master, bpy.data.objects[WEST_CURVE_NAME])
        install_north_east_curve_contact()
        install_visible_canvas_contact()
        install_timer()
        print("SOUTH TOP v036: controls already installed; property watcher reconnected.")
        return
    anchors = [bpy.data.objects.get(name) for name in ANCHORS]
    require(all(anchors), "one or more existing South anchor controls are missing")
    original_hooks = [m for m in curve.modifiers if m.type == "HOOK"]
    require(len(original_hooks) == 16, "expected the 16 original/blended South Hooks")
    require(all(bpy.data.objects.get(f"CTRL_Rope_TR_S_Curve_P{i:02d}") is None
                for i in range(10)), "a point control name already exists")
    require(all(key in master for key in (
        "SW_TR_TAUT_CONTROL_X", "SW_TR_TAUT_CONTROL_Y", "SW_TR_TAUT_CONTROL_Z",
        "SW_TR_DEFAULT_ROTATION_X", "SW_TR_DEFAULT_ROTATION_Y", "SW_TR_DEFAULT_ROTATION_Z")),
        "the master rest transform is unavailable")

    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    saved_location = master.location.copy()
    saved_rotation = master.rotation_euler.copy()
    try:
        # Bind at the intact rest pose, then return the artist's exact pose.
        master.location = Vector(tuple(float(master[k]) for k in (
            "SW_TR_TAUT_CONTROL_X", "SW_TR_TAUT_CONTROL_Y", "SW_TR_TAUT_CONTROL_Z")))
        master.rotation_euler = tuple(float(master[k]) for k in (
            "SW_TR_DEFAULT_ROTATION_X", "SW_TR_DEFAULT_ROTATION_Y",
            "SW_TR_DEFAULT_ROTATION_Z"))
        bpy.context.view_layer.update()

        collection = bpy.data.collections.new(COLLECTION_NAME)
        bpy.context.scene.collection.children.link(collection)
        spline = curve.data.splines[0]
        # Original 4 points land at indices 0,4,8,9 after two subdivisions.
        source_mix = []
        for i in range(10):
            if i <= 4:
                source_mix.append((0, 1, i / 4.0))
            elif i <= 8:
                source_mix.append((1, 2, (i - 4) / 4.0))
            else:
                source_mix.append((3, 3, 0.0))

        positions = [curve.matrix_world @ p.co for p in spline.bezier_points]
        anchor_positions = [a.matrix_world.translation.copy() for a in anchors]
        # Disable old Hooks only after rest samples have been captured.
        for modifier in original_hooks:
            curve.modifiers.remove(modifier)

        for i, (point, initial_world) in enumerate(zip(spline.bezier_points, positions)):
            left, right, weight = source_mix[i]
            a, b = anchors[left], anchors[right]
            neutral = anchor_positions[left].lerp(anchor_positions[right], weight)
            correction = initial_world - neutral
            base = bpy.data.objects.new(f"BASE_Rope_TR_S_Curve_P{i:02d}", None)
            collection.objects.link(base)
            base.location = initial_world
            base.empty_display_type = "PLAIN_AXES"
            base.empty_display_size = .035
            base.hide_set(True)
            base.hide_render = True
            for axis_index, axis_name in enumerate("XYZ"):
                fcurve = base.driver_add("location", axis_index)
                driver = fcurve.driver
                driver.type = "SCRIPTED"
                add_transform_variable(driver, "a", a, axis_name)
                if left != right:
                    add_transform_variable(driver, "b", b, axis_name)
                    driver.expression = (f"a*{1.0-weight:.9g}+b*{weight:.9g}"
                                         f"+({correction[axis_index]:.12g})")
                else:
                    driver.expression = f"a+({correction[axis_index]:.12g})"

            control = bpy.data.objects.new(f"CTRL_Rope_TR_S_Curve_P{i:02d}", None)
            collection.objects.link(control)
            control.parent = base
            control.location = (0, 0, 0)
            control.empty_display_type = "SPHERE"
            control.empty_display_size = .085
            control.show_in_front = True
            control.show_name = True
            control.hide_render = True
            control["South Bezier Point"] = i
            control["Move this Empty or use the matching master Offset properties"] = True
            offset = bpy.data.objects.new(f"OFFSET_Rope_TR_S_Curve_P{i:02d}", None)
            collection.objects.link(offset)
            offset.parent = control
            offset.empty_display_type = "PLAIN_AXES"
            offset.empty_display_size = .025
            # Hook targets must remain evaluated; hiding this helper with
            # hide_set() can suspend its property drivers in Blender.
            offset.hide_set(False)
            offset.hide_render = True
            for axis_index, axis_name in enumerate("XYZ"):
                location_prop = f"{control.name} | Offset Loc {axis_name}"
                rotation_prop = f"{control.name} | Offset Rot {axis_name}"
                master[location_prop] = 0.0
                master[rotation_prop] = 0.0
                master.id_properties_ui(location_prop).update(subtype="DISTANCE")
                master.id_properties_ui(rotation_prop).update(subtype="ANGLE")

            bpy.context.view_layer.update()
            hook = curve.modifiers.new(f"LSO South Artist P{i:02d}", "HOOK")
            hook.object = offset
            hook.strength = 1.0
            hook.falloff_type = "NONE"
            hook.center = point.co.copy()
            hook.vertex_indices_set((i * 3, i * 3 + 1, i * 3 + 2))
            hook.matrix_inverse = offset.matrix_world.inverted() @ curve.matrix_world

        # The tube Geometry Nodes modifier must evaluate *after* the Hooks.
        # Appending Hooks below it makes the visible tube ignore every control.
        bpy.context.view_layer.objects.active = curve
        for obj in bpy.context.selected_objects:
            obj.select_set(False)
        curve.select_set(True)
        with bpy.context.temp_override(object=curve, active_object=curve):
            for i in range(10):
                bpy.ops.object.modifier_move_to_index(
                    modifier=f"LSO South Artist P{i:02d}", index=i)

        # Blender may defer new driver relation edges until reopening the file.
        # Reparse them in this session so master and property edits are live now.
        for obj in collection.objects:
            if obj.animation_data:
                for fcurve in obj.animation_data.drivers:
                    expression = fcurve.driver.expression
                    fcurve.driver.expression = f"({expression})"
                obj.update_tag(refresh={"OBJECT"})
        master.update_tag(refresh={"OBJECT"})
        curve[MARKER] = True
        curve.data.update_tag()
        install_artist_south_drape(master, curve)
        install_west_point_controls(master)
        install_west_floor_drape(master, bpy.data.objects[WEST_CURVE_NAME])
        install_north_east_curve_contact()
        install_visible_canvas_contact()
        install_timer()
        bpy.context.view_layer.update()
        print("SW TOP v036: 10 visible point controls per side installed; original rope props retained.")
        print("No .blend file was created or saved.")
    finally:
        master.location = saved_location
        master.rotation_euler = saved_rotation
        bpy.context.view_layer.update()


if __name__ == "__main__":
    build()

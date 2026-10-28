"""v043 continuation of v041: curve-first SW top-rope glide.

Run once in Blender's Text Editor. It performs v041 setup/repair internally,
then adds glide motion. This does NOT replace or edit
MESH_Ring Ropes, its weights, BONES_ROPES, Spline IK, either Curve datablock,
the turnbuckle hierarchy, or the existing Hook modifiers. It only applies
temporary delta transforms to the existing Hook BASE empties. It does not
save a .blend. To remove its live effect, set RESET_ONLY=True and run again.

This is a kinematic, fixed-length two-span cable solver, not a collision
simulation for wrestlers or other moving objects. Poses requiring more rope
than exists are rejected and the last valid shape is retained.
"""

import bpy
import math
import runpy
from pathlib import Path
from mathutils import Vector

RESET_ONLY = False
MASTER = "CTRL_Ring_Hardware_Turnbuckle_Jaw_Post_TR_S_W"
MESH = "MESH_Ring Ropes"
CANVAS = "MESH_Ring Canvas"
KEY = "_LSO_SW_CURVE_GLIDE_V043"
TIMER_KEY = "_LSO_SW_CURVE_GLIDE_V043_TIMER"
SAVED_BASELINE = "LSO_V043_Original_Deltas"
JOIN_CONSTRAINT = "LSO V043 Shared SW Eye Junction"
SEAM_SLIDE_BIAS_M = 0.085
SEAM_LATERAL_SLIDE = 0.86
V038 = Path(r"C:\Users\rmart\Downloads\Rope_Rig_Versions\SW_Top_West_CapturedShape_v038_NoSave.py")
CLEARANCE = 0.095  # rope centre above canvas, metres
OUTSIDE_FLOOR = 0.075

SPEC = {
    "S": ("ROPE_SOUTH_TOP_CURVE", "CTRL_Rope Point_TR_S_end_r",
          "CTRL_Rope Point_TR_S_end_l", "SW_TR_SOUTH_TOP_FIXED_LENGTH"),
    "W": ("ROPE_WEST_TOP_CURVE", "CTRL_Rope Point_TR_W_end_l",
          "CTRL_Rope Point_TR_W_end_r", "SW_TR_WEST_TOP_FIXED_LENGTH"),
}


def smoothstep(value):
    t = max(0.0, min(1.0, value))
    return t*t*(3.0-2.0*t)


def world(obj):
    return obj.matrix_world.translation.copy()


def ensure_v041():
    """Keep v041's setup and repair in this v043, without running v041."""
    if bpy.data.meshes.get("LSO_SW_SLIDING_TOP_ROPE_ORIGINAL_v042") is not None:
        raise RuntimeError("v042 direct-mesh deformation is present. Reopen the clean scene; nothing changed")
    curves = [bpy.data.objects.get(name) for name in
              ("ROPE_WEST_TOP_CURVE", "ROPE_SOUTH_TOP_CURVE")]
    if any(obj is None or obj.type != "CURVE" or len(obj.data.splines) != 1
           for obj in curves):
        raise RuntimeError("Original South/West top curves are missing; nothing changed")
    counts = [len(obj.data.splines[0].bezier_points) for obj in curves]
    if counts == [4, 4]:
        if not V038.is_file():
            raise RuntimeError("The original v038 setup required by v041 is missing; nothing changed")
        runpy.run_path(str(V038), run_name="__main__")
    elif counts != [10, 10]:
        raise RuntimeError(f"Unexpected curve counts {counts}; nothing changed")
    master = bpy.data.objects.get(MASTER)
    if master is None or master.type != "EMPTY":
        raise RuntimeError("v041 master Empty is missing; nothing changed")
    if not master.get("LSO_WEST_XMINUS50_Z175_ARTIST_PROFILE_V038"):
        if not V038.is_file():
            raise RuntimeError("The original v038 setup required by v041 is missing; nothing changed")
        runpy.run_path(str(V038), run_name="__main__")

    # v041's original repair: validate all twenty artist Hook chains before
    # removing only v040 pickup follow and old v041 floor constraints.
    bases, offsets = [], []
    for side, curve in zip("WS", curves):
        for i in range(10):
            base = bpy.data.objects.get(f"BASE_Rope_TR_{side}_Curve_P{i:02d}")
            ctrl = bpy.data.objects.get(f"CTRL_Rope_TR_{side}_Curve_P{i:02d}")
            offset = bpy.data.objects.get(f"OFFSET_Rope_TR_{side}_Curve_P{i:02d}")
            hook = curve.modifiers.get(
                f"LSO {'West' if side == 'W' else 'South'} Artist P{i:02d}")
            if (base is None or ctrl is None or offset is None or
                    ctrl.parent != base or offset.parent != ctrl or
                    hook is None or hook.type != "HOOK" or hook.object != offset):
                raise RuntimeError(f"v041 {side} P{i:02d} Hook chain is incomplete; nothing removed")
            bases.append(base)
            offsets.append(offset)
    for base in bases:
        for constraint in tuple(base.constraints):
            if constraint.name.startswith("LSO SW Pickup Follow v040"):
                base.constraints.remove(constraint)
    for offset in offsets:
        constraint = offset.constraints.get("LSO SW Rope Canvas Contact v041")
        if constraint is not None:
            offset.constraints.remove(constraint)
        for key in ("LSO_V041_CANVAS_TOP_M", "LSO_V041_HOOK_REACH_M"):
            if key in offset:
                del offset[key]


def floor_at(state, x, y):
    x0, x1, y0, y1, top = state["canvas_bounds"]
    # A hard inside/outside switch made individual samples fall a metre in
    # one update as Y crossed the apron. Let the support height descend
    # continuously outside the mat edge; inside remains canvas-safe.
    dx = max(x0-x, 0.0, x-x1)
    dy = max(y0-y, 0.0, y-y1)
    outside = math.hypot(dx, dy)
    mat_floor = top + CLEARANCE
    return mat_floor + (OUTSIDE_FLOOR-mat_floor)*smoothstep(outside/0.45)


def soft_floor(z, floor):
    gap = z-floor
    if gap >= 0.08:
        return z
    if gap <= 0.0:
        return floor
    u = gap/0.08
    return floor + 0.08*(2.0*u*u-u*u*u)


def path(state, side, start, end, length):
    """One smooth span; keep depth and height variation without an S-fold."""
    chord = (end-start).length
    if chord > length + 0.002:
        return None
    along = end-start
    lateral = Vector((-along.y, along.x, 0.0))
    if lateral.length < 1e-6:
        lateral = Vector((1.0, 0.0, 0.0))
    lateral.normalize()
    # Choose the interior-facing side only once. Re-evaluating that choice
    # from the moving centre flipped the lateral sign during Y drags.
    previous = state["lateral_ref"].get(side)
    if previous is None:
        center = (start+end)*0.5
        if (center+lateral*0.1).xy.length > (center-lateral*0.1).xy.length:
            lateral.negate()
    elif lateral.dot(previous) < 0.0:
        lateral.negate()
    state["lateral_ref"][side] = lateral.copy()

    # Put the deepest part of the two spans at different positions. The
    # lateral displacement always has one sign, so the rope may form a wide
    # C/U but cannot reverse through its own centreline along this span.
    bend_bias = 0.48 if side == "W" else -0.48

    def sample(vertical, sideways, count=33):
        result = []
        for i in range(count):
            t = i/(count-1)
            p = start.lerp(end, t)
            if 0 < i < count-1:
                bell = math.sin(math.pi*t)
                # Positive, asymmetric envelopes keep the individual points
                # at different Y/Z positions as the picked-up end moves.
                # The along-chord coordinate stays ordered at every sample.
                sweep = bell*(1.0 + bend_bias*math.cos(math.pi*t))
                drape = bell*(1.0 - 0.32*bend_bias*math.cos(math.pi*t))
                p += lateral*(sideways*sweep)
                p.z = soft_floor(p.z-vertical*drape, floor_at(state,p.x,p.y))
            result.append(p)
        return result

    def measure(points):
        return sum((points[i+1]-points[i]).length for i in range(len(points)-1))

    # A completely taut span is a line. With slack, solve path amplitude by
    # bisection so the arclength, rather than the endpoint chord, is fixed.
    low = sample(0.0, 0.0)
    if measure(low) > length+0.003:
        return None
    high_drop = max(1.0, (start.z+end.z)*0.5 + 0.5)
    drop_path = sample(high_drop, 0.0)
    if measure(drop_path) >= length:
        lo, hi = 0.0, high_drop
        for _ in range(15):
            mid = (lo+hi)*0.5
            candidate = sample(mid, 0.0)
            if measure(candidate) < length:
                lo = mid
            else:
                hi = mid
        return sample((lo+hi)*0.5, 0.0)
    # Once the rope rests on the mat, extra length forms a wide floor loop.
    sideways_hi = 0.1
    while measure(sample(high_drop, sideways_hi)) < length and sideways_hi < 8.0:
        sideways_hi *= 2.0
    if sideways_hi >= 8.0 and measure(sample(high_drop, sideways_hi)) < length:
        return None
    lo, hi = 0.0, sideways_hi
    for _ in range(15):
        mid = (lo+hi)*0.5
        if measure(sample(high_drop, mid)) < length:
            lo = mid
        else:
            hi = mid
    return sample(high_drop, (lo+hi)*0.5)


def sample_at(polyline, factor):
    scaled = max(0.0, min(1.0, factor))*(len(polyline)-1)
    index = min(len(polyline)-2, int(scaled))
    return polyline[index].lerp(polyline[index+1], scaled-index)


def get_scene_state():
    master = bpy.data.objects.get(MASTER)
    mesh = bpy.data.objects.get(MESH)
    canvas = bpy.data.objects.get(CANVAS)
    if not master or master.type != "EMPTY" or not mesh or mesh.type != "MESH":
        raise RuntimeError("v041 master Empty or original rope mesh is missing; nothing changed")
    if not canvas or canvas.type != "MESH":
        raise RuntimeError("Ring canvas is missing; nothing changed")
    required = ("SW_TR_TAUT_CONTROL_X", "SW_TR_TAUT_CONTROL_Y",
                "SW_TR_TAUT_CONTROL_Z")
    if any(key not in master for key in required):
        raise RuntimeError("v041 rest-position properties are missing; nothing changed")
    data = {}
    for side, (curve_name, eye_name, far_name, length_name) in SPEC.items():
        curve = bpy.data.objects.get(curve_name)
        eye = bpy.data.objects.get(eye_name)
        far = bpy.data.objects.get(far_name)
        if not curve or curve.type != "CURVE" or not eye or not far:
            raise RuntimeError(f"v041 {side} curve/endpoint is missing; nothing changed")
        if len(curve.data.splines) != 1 or len(curve.data.splines[0].bezier_points) != 10:
            raise RuntimeError(f"v041 {side} is not a ten-point Bezier curve; nothing changed")
        if length_name not in master:
            raise RuntimeError(f"Fixed rope length {length_name} is missing; nothing changed")
        bases = []
        offsets = []
        for i in range(10):  # point 9 is the eyehook tangent and must also move
            base = bpy.data.objects.get(f"BASE_Rope_TR_{side}_Curve_P{i:02d}")
            ctrl = bpy.data.objects.get(f"CTRL_Rope_TR_{side}_Curve_P{i:02d}")
            offset = bpy.data.objects.get(f"OFFSET_Rope_TR_{side}_Curve_P{i:02d}")
            hook = curve.modifiers.get(f"LSO {'South' if side == 'S' else 'West'} Artist P{i:02d}")
            if (not base or not ctrl or not offset or ctrl.parent != base or
                    offset.parent != ctrl or not hook or hook.type != "HOOK" or
                    hook.object != offset or base.parent is not None):
                raise RuntimeError(f"v041 {side} P{i:02d} Hook chain is incomplete; nothing changed")
            bases.append(base)
            offsets.append(offset)
        data[side] = dict(curve=curve, eye=eye, far=far, bases=bases,
                          offsets=offsets,
                          length=float(master[length_name]))
    corners = [canvas.matrix_world @ Vector(c) for c in canvas.bound_box]
    bounds = (min(c.x for c in corners), max(c.x for c in corners),
              min(c.y for c in corners), max(c.y for c in corners),
              max(c.z for c in corners))
    return master, data, bounds


def reset():
    state = bpy.app.driver_namespace.get(KEY)
    callback = bpy.app.driver_namespace.get(TIMER_KEY)
    if callback:
        try:
            if bpy.app.timers.is_registered(callback):
                bpy.app.timers.unregister(callback)
        except ReferenceError:
            pass
    if state:
        for reference, loc, rot, scale in state.get("original_deltas", ()):
            # Older in-memory v043 runs stored RNA objects here. Blender can
            # remove those objects during scene reload, making even .name
            # raise ReferenceError. The live object is always looked up anew.
            try:
                name = reference if isinstance(reference, str) else reference.name
            except ReferenceError:
                continue
            live = bpy.data.objects.get(name)
            if live is not None:
                live.delta_location = loc
                live.delta_rotation_euler = rot
                live.delta_scale = scale
        bpy.context.view_layer.update()
    # If Blender was saved and reopened while v043 was active, the runtime
    # callback is gone but the deltas remain. Recover from the baseline stored
    # on each affected Empty rather than mistaking them for artist edits.
    for side in "SW":
        for i in range(10):
            base = bpy.data.objects.get(f"BASE_Rope_TR_{side}_Curve_P{i:02d}")
            if base is None or SAVED_BASELINE not in base:
                continue
            values = list(base[SAVED_BASELINE])
            if len(values) == 9:
                base.delta_location = values[0:3]
                base.delta_rotation_euler = values[3:6]
                base.delta_scale = values[6:9]
            del base[SAVED_BASELINE]
    west_hook = bpy.data.objects.get("OFFSET_Rope_TR_W_Curve_P09")
    if west_hook is not None:
        join = west_hook.constraints.get(JOIN_CONSTRAINT)
        if join is not None:
            west_hook.constraints.remove(join)
    bpy.app.driver_namespace.pop(KEY, None)
    bpy.app.driver_namespace.pop(TIMER_KEY, None)
    print("SW curve glide reset; original v041 control deltas restored. No .blend saved.")


def tick():
    state = bpy.app.driver_namespace.get(KEY)
    if not state:
        return None
    if state["busy"]:
        return 0.04
    state["busy"] = True
    try:
        master = state["master"]
        if bpy.data.objects.get(MASTER) != master:
            return None
        pose = tuple(round(v,5) for v in master.matrix_world.translation)
        pose += tuple(round(v,5) for v in master.matrix_world.to_euler())
        pose += tuple(round(v,5) for side in "SW" for obj in
                      (state["sides"][side]["eye"], state["sides"][side]["far"])
                      for v in obj.matrix_world.translation)
        if pose == state.get("last_pose"):
            return 0.04
        rest = state["rest_master"]
        displacement = (world(master)-rest).length
        blend = smoothstep((displacement-0.005)/0.16)
        # One material budget across both original curve objects. The eyehook
        # is a frictionless sliding guide: when one chord grows, rope is
        # transferred from the other span instead of manufactured.
        sw = state["sides"]["W"]
        ss = state["sides"]["S"]
        eye_w, far_w = world(sw["eye"]), world(sw["far"])
        eye_s, far_s = world(ss["eye"]), world(ss["far"])
        chord_w, chord_s = (eye_w-far_w).length, (eye_s-far_s).length
        total = sw["length"] + ss["length"]
        if chord_w+chord_s > total-0.001:
            state["invalid"] = "The two adjacent corners need more rope than exists"
            return 0.04
        # Chord imbalance is the sliding coordinate of the continuous rope.
        slide = 0.5*((chord_w-state["rest_chords"]["W"])-
                     (chord_s-state["rest_chords"]["S"]))
        # The original West Spline IK chain places its Bone.098 head about
        # 85 mm beyond the physical SW eye when the picked-up corner moves.
        # Transfer that much material from South to West. Total cable length
        # stays unchanged; the existing mesh seam then has both bone chains
        # meeting at the eye instead of being skinned across a gap.
        local_motion = master.rotation_euler.to_matrix().inverted() @ (world(master)-rest)
        slide += SEAM_SLIDE_BIAS_M + SEAM_LATERAL_SLIDE*local_motion.y
        min_slide = chord_w-sw["length"]+0.001
        max_slide = ss["length"]-chord_s-0.001
        requested_slide = slide
        slide = max(min_slide, min(max_slide, slide))
        state["slide_debug"] = (requested_slide, slide, min_slide, max_slide)
        target_w = sw["length"]+slide
        target_s = ss["length"]-slide
        # Curve orientation: West runs far->eye; South runs eye->far.
        # West P09 is not disposable metadata: it is the last *visible*
        # Bezier point. v041 leaves it 1.5 m beyond the moving eye, where it
        # crosses South. Give the two separate splines one short, shared
        # passage through the eye instead. West ends exactly where South
        # begins; no duplicate overlapping metre of rope is manufactured.
        preliminary = path(state,"S",eye_s,far_s,target_s)
        if preliminary is None:
            state["invalid"] = "South cannot reach the moving eye at fixed length"
            return 0.04
        outgoing = sample_at(preliminary,0.08)-eye_s
        if outgoing.length < 1e-5:
            outgoing = far_s-eye_s
        if outgoing.length < 1e-5:
            state["invalid"] = "South eye tangent is undefined"
            return 0.04
        outgoing.normalize()
        connector = min(0.04,max(0.0,target_w-chord_w-0.002))
        if connector < 0.005:
            connector = 0.0
        exit_point = eye_s + outgoing*connector
        west = path(state,"W",far_w,eye_w,target_w-connector)
        south = path(state,"S",exit_point,far_s,target_s)
        if west is None or south is None:
            state["invalid"] = "This pose cannot be represented at fixed rope length"
            return 0.04
        targets = {"W": west, "S": south}
        for side in "SW":
            info = state["sides"][side]
            bases = info["bases"]
            spline = info["curve"].data.splines[0]
            curve_matrix = info["curve"].matrix_world.to_3x3()
            polyline = targets[side]
            for i, base in enumerate(bases[:9]):
                # Native v041 location is driver-owned. Add only a delta,
                # preserving every driver and the artist's child offsets.
                original_delta = state["delta_by_name"][base.name][0]
                native = base.location.copy() + original_delta
                # There are ten Bezier controls, not nine. P09 is the actual
                # endpoint (the shared eye for West, the far anchor for
                # South); P08 must remain one interval BEFORE that endpoint.
                # Sampling i/8 put P08 on top of P09 and made a near-zero
                # terminal span with a visibly sharp fold in the rope mesh.
                desired = sample_at(polyline,i/9.0)
                # South P00 and West P09 are the *same physical passage*
                # through the moving eye. Do not ease one endpoint away from
                # the other while blending the rest of the cable shape.
                point_blend = 1.0 if side == "S" and i == 0 else blend
                base.delta_location = original_delta + point_blend*(desired-native)
                source = curve_matrix @ (spline.bezier_points[i].handle_right-
                                         spline.bezier_points[i].handle_left)
                t0 = sample_at(polyline,max(0.0,(i-0.5)/9.0))
                t1 = sample_at(polyline,min(1.0,(i+0.5)/9.0))
                tangent = t1-t0
                if side == "W" and i == 8 and connector:
                    incoming = tangent.normalized()
                    tangent = incoming + outgoing
                    if tangent.length < 1e-5:
                        tangent = outgoing.copy()
                original_rot = state["delta_by_name"][base.name][1]
                original_scale = state["delta_by_name"][base.name][2]
                if source.length > 1e-5 and tangent.length > 1e-5:
                    rotation = source.rotation_difference(tangent).to_euler("XYZ")
                    base.delta_rotation_euler = tuple(
                        original_rot[k] + blend*rotation[k] for k in range(3))
                # Shorten long old handles near canvas contact. Smooth
                # neighbouring tangents remain aligned with the cable path.
                segment = max(0.01,(t1-t0).length)
                old_span = max(0.01,source.length)
                # old_span is the *sum* of the two Bezier handle lengths.
                # The former 1.5 multiplier let those handles reach across
                # neighbouring points and produce a loop despite ordered
                # point centres. Cap the total span below one local segment.
                factor = min(1.0,max(0.02,0.55*segment/old_span))
                if side == "W" and i == 8 and connector:
                    factor = min(factor,0.35)
                if side == "S" and i == 0 and connector:
                    factor = min(factor,0.45)
                s = 1.0-blend*(1.0-factor)
                base.delta_scale = tuple(original_scale[k]*s for k in range(3))
        west_info = state["sides"]["W"]
        base = west_info["bases"][9]
        loc, rot, scale = state["delta_by_name"][base.name]
        native = base.location.copy()+loc
        # The artist edits the CTRL children, not just their BASE parents.
        # Put the West BASE near the actual South Hook target, including the
        # artist's child offset. The Copy Location constraint below then
        # makes the two evaluated Hook targets exactly coincident.
        south_info = state["sides"]["S"]
        south_artist = (world(south_info["offsets"][0])-
                        world(south_info["bases"][0]))
        west_artist = world(west_info["offsets"][9])-world(base)
        base.delta_location = loc+(exit_point+south_artist-west_artist-native)
        source = west_info["curve"].matrix_world.to_3x3() @ (
            west_info["curve"].data.splines[0].bezier_points[9].handle_right-
            west_info["curve"].data.splines[0].bezier_points[9].handle_left)
        if source.length > 1e-5:
            angle = source.rotation_difference(outgoing).to_euler("XYZ")
            base.delta_rotation_euler = tuple(rot[k]+angle[k] for k in range(3))
            # With no spare connector length, the endpoint sits at the eye;
            # collapse only its handle, never send the point back outside.
            handle_scale = min(1.0,max(0.02,0.7*connector/source.length))
            base.delta_scale = tuple(scale[k]*handle_scale for k in range(3))
        state["last_pose"] = pose
        state["invalid"] = ""
        # Let Blender's normal viewport/depsgraph cycle evaluate the edits.
        # Forcing a full update here is costly in this 11k-object scene.
    except ReferenceError:
        return None
    except Exception as exc:
        state["error"] = str(exc)
        print("SW curve glide update stopped:", exc)
        return None
    finally:
        state["busy"] = False
    return 0.04


def install():
    reset()
    ensure_v041()
    master, sides, bounds = get_scene_state()  # complete preflight before writes
    west_hook = sides["W"]["offsets"][9]
    south_hook = sides["S"]["offsets"][0]
    existing_join = west_hook.constraints.get(JOIN_CONSTRAINT)
    if existing_join is not None and (
            existing_join.type != "COPY_LOCATION" or
            existing_join.target != south_hook):
        raise RuntimeError("The SW junction constraint name is already in use; nothing changed")
    if existing_join is None:
        existing_join = west_hook.constraints.new("COPY_LOCATION")
        existing_join.name = JOIN_CONSTRAINT
    existing_join.target = south_hook
    existing_join.target_space = "WORLD"
    existing_join.owner_space = "WORLD"
    existing_join.use_x = existing_join.use_y = existing_join.use_z = True
    existing_join.influence = 1.0
    rest = Vector(tuple(float(master[f"SW_TR_TAUT_CONTROL_{a}"]) for a in "XYZ"))
    original = []
    index = {}
    for side in "SW":
        for base in sides[side]["bases"]:
            values = (base.delta_location.copy(), base.delta_rotation_euler.copy(),
                      base.delta_scale.copy())
            original.append((base.name,*values))
            index[base.name] = values
            base[SAVED_BASELINE] = tuple(values[0])+tuple(values[1])+tuple(values[2])
    state = dict(master=master,sides=sides,canvas_bounds=bounds,rest_master=rest,
                 rest_chords={side:(world(info["eye"])-world(info["far"])).length
                              for side,info in sides.items()},
                 original_deltas=original,delta_by_name=index,last_pose=None,
                 invalid="",error="",busy=False,lateral_ref={})
    bpy.app.driver_namespace[KEY] = state
    bpy.app.driver_namespace[TIMER_KEY] = tick
    tick()
    if state["error"]:
        error = state["error"]
        reset()
        raise RuntimeError(f"SW curve glide could not initialize: {error}")
    bpy.app.timers.register(tick,first_interval=0.04,persistent=False)
    print("SW v043 curve glide installed: two separate curves, one length budget,"
          " original Spline IK and mesh retained. No .blend saved.")


if __name__ == "__main__":
    reset() if RESET_ONLY else install()

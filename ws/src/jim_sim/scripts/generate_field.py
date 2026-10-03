#!/usr/bin/env python3
"""
Generate the Gazebo field world and its ground-truth file.

Everything the oracle detector "sees" comes from the same source of truth
this script writes, so simulation and detection cannot silently disagree.

Coordinates here are in the work_surface frame: u to the right and v away
from you, both in metres, origin at the centre of the reachable area. The
whole scene is emitted inside <model name="field">, whose pose shifts
work_surface coordinates into world coordinates.

The planted field is deliberately much larger than the gantry's reach.
Only what falls inside the 250 x 250 mm reachable square is written to
ground_truth.yaml, so the oracle physically cannot report a weed the
machine could not treat. The outline marker on the soil makes that
boundary visible to an observer.

Run it to regenerate with a different seed or layout:
    python3 generate_field.py --seed 7 --weeds 8
"""

import argparse
import math
import random

# --------------------------------------------------------------------------
# Workspace geometry, derived from the URDF.
#
#   joint_x = 0 .. 0.25   moves the laser along world +X
#   joint_y = 0 .. 0.25   moves the laser along world -Y
#
# The centre of the reachable ground area sits at base_link (0.1837, -0.2516)
# and base_link is 0.30 m above the soil. work_surface is placed there so the
# reachable region becomes a tidy square centred on the origin.
# --------------------------------------------------------------------------
WORK_CENTRE_X = 0.1837
WORK_CENTRE_Y = -0.2516
HALF = 0.125          # half of the 250 mm travel

# --------------------------------------------------------------------------
# Scale. A real field dwarfs this machine, and the demo only lands if that
# is obvious: the gantry treats a 250 mm square inside an 8 x 6 m plot,
# roughly a 1:32 ratio. Modelling every plant at that size would be tens of
# thousands of models, so the field is drawn at two levels of detail:
#
#   within DETAIL_HALF   individual potato saplings, full geometry
#   beyond it            continuous row strips, a few visuals per row
#
# The transition sits well outside the reachable square, so everything the
# panel actually scrutinises is a real plant. The strips only have to read
# as receding crop rows, which at that distance they do.
# --------------------------------------------------------------------------
ROW_SPACING_V = 0.085
CROP_SPACING_U = 0.062

DETAIL_HALF = 0.500    # individually modelled saplings out to here
FIELD_HALF_U = 4.000   # 8 m of row length
FIELD_HALF_V = 3.000   # 6 m across the rows
SEG_LEN = 2.000        # row strips are cut into segments this long

WALL_HALF_U = 4.200    # dry-stone boundary at the field edge
WALL_HALF_V = 3.200
TREE_RING_U = 4.550    # treeline stands behind the wall
TREE_RING_V = 3.550
TREE_SPACING = 1.000


def _mat(ambient, diffuse, specular="0.05 0.06 0.05 1", emissive=None):
    e = f"\n            <emissive>{emissive}</emissive>" if emissive else ""
    return f"""
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>{specular}</specular>{e}
          </material>"""


# --------------------------------------------------------------------------
# Potato sapling
#
# A 3-4 week potato plant is a compound leaf structure, not a rosette: an
# erect central stem carrying opposed pairs of oval leaflets on short
# petioles, topped by a single larger terminal leaflet. Built from
# flattened ellipsoids so it stays cheap - these are visual targets, not
# obstacles, so they are static and carry no collision geometry.
# --------------------------------------------------------------------------
def crop_model(name, u, v, height, pairs, rot, tint):
    stem_r = 0.0020 + 0.0006 * (height / 0.075)
    parts = [f"""
        <visual name="stem">
          <pose>0 0 {height/2:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{stem_r:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>{_mat("0.10 0.22 0.09 1", "0.20 0.40 0.16 1")}
        </visual>"""]

    # Mid-green with a faint blue cast, varied slightly per plant.
    amb = f"{0.07 + tint*0.03:.3f} {0.20 + tint*0.05:.3f} {0.11 + tint*0.04:.3f} 1"
    dif = f"{0.13 + tint*0.06:.3f} {0.38 + tint*0.10:.3f} {0.24 + tint*0.07:.3f} 1"

    for i in range(pairs):
        # Leaflet pairs climb the stem, each pair rotated off the last so
        # the plant reads as three-dimensional rather than flat.
        z = height * (0.30 + 0.62 * (i / max(1, pairs)))
        a = rot + i * 1.35
        lr = 0.014 + 0.004 * (height / 0.075)
        rx, ry, rz = 0.017, 0.0105, 0.0016
        for k, sgn in enumerate((1.0, -1.0)):
            ang = a + (0.0 if sgn > 0 else math.pi)
            lx, ly = lr * math.cos(ang), lr * math.sin(ang)
            # slight upward cup, and a droop that grows down the stem
            pitch = -0.30 + 0.10 * i
            parts.append(f"""
        <visual name="leaflet_{i}_{k}">
          <pose>{lx:.4f} {ly:.4f} {z:.4f} 0 {pitch:.3f} {ang:.4f}</pose>
          <geometry><ellipsoid><radii>{rx:.4f} {ry:.4f} {rz:.4f}</radii></ellipsoid></geometry>{_mat(amb, dif)}
        </visual>""")

    # Terminal leaflet: noticeably larger, sits on the growing tip.
    parts.append(f"""
        <visual name="leaflet_terminal">
          <pose>0 0 {height + 0.004:.4f} 0 -0.12 {rot:.4f}</pose>
          <geometry><ellipsoid><radii>0.0215 0.0140 0.0018</radii></ellipsoid></geometry>{_mat(amb, dif)}
        </visual>""")

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{u:.4f} {v:.4f} 0 0 0 0</pose>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


def weed_model(name, u, v, scale, rot):
    """
    Weed: taller, spikier, yellower green than the crop. Deliberately
    distinguishable by eye so the demo reads clearly on a projector.
    """
    blades = []
    n = 5
    for i in range(n):
        a = rot + i * (2 * math.pi / n) + random.uniform(-0.3, 0.3)
        tilt = random.uniform(0.15, 0.55)
        br = 0.011 * scale
        bx, by = br * math.cos(a), br * math.sin(a)
        blades.append(f"""
        <visual name="blade_{i}">
          <pose>{bx:.4f} {by:.4f} {0.020*scale:.4f} {-tilt*math.sin(a):.4f} {tilt*math.cos(a):.4f} {a:.4f}</pose>
          <geometry>
            <ellipsoid><radii>{0.0035*scale:.4f} {0.0035*scale:.4f} {0.020*scale:.4f}</radii></ellipsoid>
          </geometry>{_mat("0.24 0.30 0.04 1", "0.52 0.62 0.10 1")}
        </visual>""")

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{u:.4f} {v:.4f} 0 0 0 0</pose>
      <link name="link">{''.join(blades)}
      </link>
    </model>"""


def reach_marker_model():
    """
    Thin bright outline on the soil showing exactly what the gantry can
    reach. This is the single most valuable thing in the scene: it lets an
    observer see at a glance that the machine treats what is inside the
    square and ignores everything outside it.
    """
    w, h = 0.003, 0.001
    span = 2 * HALF + w
    bars = [
        ("north", 0.0, HALF, span, w),
        ("south", 0.0, -HALF, span, w),
        ("east", HALF, 0.0, w, 2 * HALF),
        ("west", -HALF, 0.0, w, 2 * HALF),
    ]
    vis = []
    for nm, bx, by, sx, sy in bars:
        vis.append(f"""
        <visual name="edge_{nm}">
          <pose>{bx:.4f} {by:.4f} {h/2:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>{_mat("0.00 0.55 0.60 1", "0.05 0.95 1.00 1", "0.20 0.30 0.30 1", "0.00 0.65 0.75 1")}
        </visual>""")

    return f"""
    <model name="reach_outline">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">{''.join(vis)}
      </link>
    </model>"""


def far_rows_model(rows):
    """
    The field beyond DETAIL_HALF, drawn as continuous crop rows.

    One thin box per segment rather than per plant. At this distance a
    receding row reads as a row; individual saplings out here would cost
    thousands of models and render as a green smear anyway. Height and
    tint jitter keep the rows from looking extruded.
    """
    vis = []
    n = 0
    for v in rows:
        if abs(v) <= DETAIL_HALF:
            # leave a hole for the individually modelled plants
            spans = [(-FIELD_HALF_U, -DETAIL_HALF), (DETAIL_HALF, FIELD_HALF_U)]
        else:
            spans = [(-FIELD_HALF_U, FIELD_HALF_U)]

        for u0, u1 in spans:
            total = u1 - u0
            k = max(1, int(round(total / SEG_LEN)))
            step = total / k
            for s in range(k):
                a = u0 + s * step
                cu = a + step / 2
                h = random.uniform(0.050, 0.072)
                t = random.random()
                amb = f"{0.06 + t*0.03:.3f} {0.18 + t*0.05:.3f} {0.09 + t*0.04:.3f} 1"
                dif = f"{0.12 + t*0.06:.3f} {0.35 + t*0.10:.3f} {0.20 + t*0.07:.3f} 1"
                vis.append(f"""
        <visual name="row_{n}">
          <pose>{cu:.4f} {v + random.uniform(-0.004, 0.004):.4f} {h/2:.4f} 0 0 0</pose>
          <geometry><box><size>{step - 0.02:.4f} 0.0480 {h:.4f}</size></box></geometry>{_mat(amb, dif)}
        </visual>""")
                n += 1

    return f"""
    <model name="crop_rows_far">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">{''.join(vis)}
      </link>
    </model>""", n


def wall_model():
    """Low boundary wall enclosing the scene, earthy stone colour."""
    t, ht = 0.150, 0.350
    segs = [
        ("north", 0.0, WALL_HALF_V, 2 * WALL_HALF_U + t, t),
        ("south", 0.0, -WALL_HALF_V, 2 * WALL_HALF_U + t, t),
        ("east", WALL_HALF_U, 0.0, t, 2 * WALL_HALF_V),
        ("west", -WALL_HALF_U, 0.0, t, 2 * WALL_HALF_V),
    ]
    vis = []
    for nm, bx, by, sx, sy in segs:
        vis.append(f"""
        <visual name="wall_{nm}">
          <pose>{bx:.4f} {by:.4f} {ht/2:.4f} 0 0 0</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {ht:.4f}</size></box></geometry>{_mat("0.24 0.21 0.18 1", "0.46 0.41 0.35 1", "0.06 0.06 0.06 1")}
        </visual>""")

    return f"""
    <model name="boundary_wall">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">{''.join(vis)}
      </link>
    </model>"""


def tree_model(name, x, y, height, lean_r, lean_p, tint, blobs):
    """Tapered trunk plus overlapping foliage spheres, varied per tree."""
    trunk_h = height * 0.45
    trunk_r = 0.020 * (height / 0.80)
    parts = [f"""
        <visual name="trunk">
          <pose>0 0 {trunk_h/2:.4f} {lean_r:.4f} {lean_p:.4f} 0</pose>
          <geometry><cylinder><radius>{trunk_r:.4f}</radius><length>{trunk_h:.4f}</length></cylinder></geometry>{_mat("0.16 0.11 0.07 1", "0.34 0.24 0.16 1", "0.03 0.03 0.03 1")}
        </visual>"""]

    amb = f"{0.05 + tint*0.04:.3f} {0.17 + tint*0.06:.3f} {0.06 + tint*0.03:.3f} 1"
    dif = f"{0.10 + tint*0.10:.3f} {0.33 + tint*0.14:.3f} {0.13 + tint*0.07:.3f} 1"

    for i in range(blobs):
        f = i / max(1, blobs - 1) if blobs > 1 else 0.0
        r = height * (0.235 - 0.055 * f)
        z = trunk_h * 0.92 + height * (0.14 + 0.30 * f)
        ox = math.cos(2.1 * i + tint * 6.0) * height * 0.055
        oy = math.sin(2.1 * i + tint * 6.0) * height * 0.055
        parts.append(f"""
        <visual name="foliage_{i}">
          <pose>{ox:.4f} {oy:.4f} {z:.4f} 0 0 0</pose>
          <geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>{_mat(amb, dif, "0.02 0.03 0.02 1")}
        </visual>""")

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


def in_reach(u, v):
    return abs(u) <= HALF and abs(v) <= HALF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--weeds", type=int, default=7,
                    help="weeds inside the reachable square (these are the "
                         "only ones written to ground truth)")
    ap.add_argument("--outside-weeds", type=int, default=18,
                    help="visual-only weeds beyond the gantry's reach")
    ap.add_argument("--foliage", type=int, default=3,
                    help="foliage spheres per tree; drop to 2 if fps suffers")
    ap.add_argument("--world-out", default="worlds/jim_field.sdf")
    ap.add_argument("--truth-out", default="config/ground_truth.yaml")
    args = ap.parse_args()

    random.seed(args.seed)

    crops, weeds_in, weeds_out, trees = [], [], [], []

    # --- rows span the whole field; only the near ones get real plants --
    rows = []
    k = 0
    while k * ROW_SPACING_V <= FIELD_HALF_V:
        rows.append(k * ROW_SPACING_V)
        if k:
            rows.append(-k * ROW_SPACING_V)
        k += 1
    rows.sort()

    cols = []
    j = 0
    while j * CROP_SPACING_U <= DETAIL_HALF:
        cols.append(j * CROP_SPACING_U)
        if j:
            cols.append(-j * CROP_SPACING_U)
        j += 1
    cols.sort()

    idx = 0
    for v in [r for r in rows if abs(r) <= DETAIL_HALF]:
        for u in cols:
            # small jitter so it does not look like a CAD drawing
            cu = u + random.uniform(-0.005, 0.005)
            cv = v + random.uniform(-0.007, 0.007)
            crops.append((f"crop_{idx}", cu, cv,
                          random.uniform(0.060, 0.090),      # stem height
                          random.choice([3, 3, 4]),           # leaflet pairs
                          random.uniform(0, 6.28),
                          random.random()))
            idx += 1

    # --- weeds inside the reach: these go to the world AND ground truth --
    gaps = [(rows[i] + rows[i + 1]) / 2 for i in range(len(rows) - 1)
            if abs(rows[i]) < HALF]
    for i in range(args.weeds):
        wu = random.uniform(-HALF + 0.015, HALF - 0.015)
        wv = random.choice(gaps) + random.uniform(-0.012, 0.012) if gaps \
            else random.uniform(-HALF + 0.015, HALF - 0.015)
        wv = max(-HALF + 0.015, min(HALF - 0.015, wv))
        weeds_in.append((f"weed_{i}", wu, wv,
                         random.uniform(0.8, 1.3),
                         random.uniform(0, 6.28)))

    # --- weeds beyond the reach: world ONLY, never ground truth ---------
    # A 20 mm buffer keeps them unambiguously outside the outline, so the
    # distinction stays legible on a projector.
    # They stay inside the detailed zone: a weed dropped 3 m out would be a
    # couple of pixels and would prove nothing to the panel.
    i = 0
    guard = 0
    while i < args.outside_weeds and guard < 10000:
        guard += 1
        wu = random.uniform(-DETAIL_HALF + 0.02, DETAIL_HALF - 0.02)
        wv = random.uniform(-DETAIL_HALF + 0.02, DETAIL_HALF - 0.02)
        if abs(wu) < HALF + 0.020 and abs(wv) < HALF + 0.020:
            continue
        weeds_out.append((f"weed_far_{i}", wu, wv,
                          random.uniform(0.8, 1.3),
                          random.uniform(0, 6.28)))
        i += 1

    # --- treeline behind the wall ---------------------------------------
    # Full-size trees, not saplings. Nothing sells the scale of the plot
    # like a 4 m tree on the horizon next to a 250 mm working square.
    t_idx = 0
    for side in range(4):
        ring = TREE_RING_U if side in (1, 3) else TREE_RING_V
        along = TREE_RING_V if side in (1, 3) else TREE_RING_U
        n_side = max(1, int((2 * along) / TREE_SPACING))
        step = (2 * along) / n_side
        for s in range(n_side):
            p = -along + s * step
            if side == 0:
                x, y = p, ring
            elif side == 1:
                x, y = ring, -p
            elif side == 2:
                x, y = -p, -ring
            else:
                x, y = -ring, p
            trees.append((f"tree_{t_idx}", x, y,
                          random.uniform(2.50, 5.00),
                          random.uniform(-0.05, 0.05),
                          random.uniform(-0.05, 0.05),
                          random.random(),
                          args.foliage))
            t_idx += 1

    # --- world ---------------------------------------------------------
    far_rows, n_row_segs = far_rows_model(rows)

    body = "".join(crop_model(*c) for c in crops)
    body += far_rows
    body += "".join(weed_model(*w) for w in weeds_in)
    body += "".join(weed_model(*w) for w in weeds_out)
    body += "".join(tree_model(*t) for t in trees)
    body += wall_model()
    body += reach_marker_model()

    n_models = (len(crops) + len(weeds_in) + len(weeds_out)
                + len(trees) + 3)

    world = f"""<?xml version="1.0" ?>
<!-- GENERATED by generate_field.py (seed={args.seed}) - do not hand edit -->
<sdf version="1.9">
  <world name="jim_field">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material>
            <ambient>0.20 0.15 0.11 1</ambient>
            <diffuse>0.30 0.22 0.16 1</diffuse>
            <specular>0.01 0.01 0.01 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Plants are placed in work_surface coordinates then shifted into
         world coordinates by this offset. -->
    <model name="field">
      <static>true</static>
      <pose>{WORK_CENTRE_X} {WORK_CENTRE_Y} 0 0 0 0</pose>
      {body}
    </model>

  </world>
</sdf>
"""

    # --- ground truth ---------------------------------------------------
    # Only what the gantry can actually reach. Weeds outside the square
    # exist in the world but must never reach the detector.
    crops_truth = [c for c in crops if in_reach(c[1], c[2])]

    truth = [
        "# GENERATED by generate_field.py - do not hand edit",
        f"# seed: {args.seed}",
        "#",
        "# Positions are in the work_surface frame, metres.",
        "# This is the SAME data used to build the world, so the oracle",
        "# detector is reading genuine ground truth, not a second guess.",
        "#",
        "# Only plants inside the reachable square appear here. The world",
        "# also contains weeds beyond the gantry's reach; they are visual",
        "# only and the oracle never reports them.",
        "",
        "work_surface:",
        f"  centre_x: {WORK_CENTRE_X}",
        f"  centre_y: {WORK_CENTRE_Y}",
        f"  half_extent: {HALF}",
        "",
        "weeds:",
    ]
    for name, u, v, s, r in weeds_in:
        truth.append(f"  - {{ id: {name}, u: {u:.5f}, v: {v:.5f}, size: {s:.3f} }}")
    truth += ["", "crops:"]
    for name, u, v, h, p, r, t in crops_truth:
        truth.append(f"  - {{ id: {name}, u: {u:.5f}, v: {v:.5f}, size: {h/0.075:.3f} }}")

    open(args.world_out, "w").write(world)
    open(args.truth_out, "w").write("\n".join(truth) + "\n")

    print(f"world  -> {args.world_out}")
    print(f"truth  -> {args.truth_out}")
    print(f"  field              {2*FIELD_HALF_U:.1f} x {2*FIELD_HALF_V:.1f} m "
          f"vs {2*HALF*1000:.0f} x {2*HALF*1000:.0f} mm reach "
          f"({FIELD_HALF_U/HALF:.0f}:1)")
    print(f"  crops modelled     {len(crops)}  ({len(crops_truth)} in reach)")
    print(f"  far row segments   {n_row_segs}  (1 model)")
    print(f"  weeds in reach     {len(weeds_in)}  (in ground truth)")
    print(f"  weeds out of reach {len(weeds_out)}  (world only)")
    print(f"  trees              {len(trees)}  ({args.foliage} foliage spheres each)")
    print(f"  total models       {n_models}")


if __name__ == "__main__":
    main()

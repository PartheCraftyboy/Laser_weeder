#!/usr/bin/env python3
"""
Generate the Gazebo field world and its ground-truth file.

Everything the oracle detector "sees" comes from the same source of truth
this script writes, so simulation and detection cannot silently disagree.

Coordinates here are in the work_surface frame: a 250 x 250 mm square
centred under the gantry, with u to the right and v away from you, both
in metres, origin at the centre of the reachable area.

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

CROP_ROWS_V = [-0.085, 0.0, 0.085]   # three rows running along u
CROP_SPACING_U = 0.062                # seedlings along each row


def crop_model(name, u, v, scale, rot):
    """
    A 2-4 week seedling: short stem, a whorl of flattened leaves.
    Static, no collision - these are visual targets, not obstacles, and
    collision geometry on 20+ plants would cost solver time for nothing.
    """
    leaves = []
    n = 6
    for i in range(n):
        a = rot + i * (2 * math.pi / n)
        lr = 0.026 * scale
        lx, ly = lr * math.cos(a), lr * math.sin(a)
        leaves.append(f"""
        <visual name="leaf_{i}">
          <pose>{lx:.4f} {ly:.4f} {0.016*scale:.4f} 0 0.42 {a:.4f}</pose>
          <geometry>
            <ellipsoid><radii>{0.024*scale:.4f} {0.013*scale:.4f} {0.0022*scale:.4f}</radii></ellipsoid>
          </geometry>
          <material>
            <ambient>0.10 0.30 0.08 1</ambient>
            <diffuse>0.18 0.52 0.14 1</diffuse>
            <specular>0.05 0.08 0.05 1</specular>
          </material>
        </visual>""")

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{u:.4f} {v:.4f} 0 0 0 0</pose>
      <link name="link">
        <visual name="stem">
          <pose>0 0 {0.008*scale:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{0.0022*scale:.4f}</radius><length>{0.016*scale:.4f}</length></cylinder></geometry>
          <material>
            <ambient>0.12 0.26 0.08 1</ambient>
            <diffuse>0.22 0.44 0.14 1</diffuse>
          </material>
        </visual>{''.join(leaves)}
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
          </geometry>
          <material>
            <ambient>0.24 0.30 0.04 1</ambient>
            <diffuse>0.52 0.62 0.10 1</diffuse>
          </material>
        </visual>""")

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{u:.4f} {v:.4f} 0 0 0 0</pose>
      <link name="link">{''.join(blades)}
      </link>
    </model>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--weeds", type=int, default=7)
    ap.add_argument("--world-out", default="worlds/jim_field.sdf")
    ap.add_argument("--truth-out", default="config/ground_truth.yaml")
    args = ap.parse_args()

    random.seed(args.seed)

    crops, weeds = [], []

    # --- crops in neat rows -------------------------------------------
    idx = 0
    for v in CROP_ROWS_V:
        u = -HALF + 0.030
        while u <= HALF - 0.030:
            # small jitter so it does not look like a CAD drawing
            cu = u + random.uniform(-0.004, 0.004)
            cv = v + random.uniform(-0.006, 0.006)
            crops.append((f"crop_{idx}", cu, cv,
                          random.uniform(0.85, 1.15),
                          random.uniform(0, 6.28)))
            idx += 1
            u += CROP_SPACING_U

    # --- weeds, biased to the gaps between rows ------------------------
    gaps = [(CROP_ROWS_V[i] + CROP_ROWS_V[i + 1]) / 2
            for i in range(len(CROP_ROWS_V) - 1)]
    gaps += [CROP_ROWS_V[0] - 0.028, CROP_ROWS_V[-1] + 0.028]

    for i in range(args.weeds):
        # keep 15 mm clear of the workspace edge so every weed is reachable
        wu = random.uniform(-HALF + 0.015, HALF - 0.015)
        wv = random.choice(gaps) + random.uniform(-0.012, 0.012)
        wv = max(-HALF + 0.015, min(HALF - 0.015, wv))
        weeds.append((f"weed_{i}", wu, wv,
                      random.uniform(0.8, 1.3),
                      random.uniform(0, 6.28)))

    # --- world ---------------------------------------------------------
    body = "".join(crop_model(*c) for c in crops)
    body += "".join(weed_model(*w) for w in weeds)

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
          <geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>4 4</size></plane></geometry>
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

    truth = [
        "# GENERATED by generate_field.py - do not hand edit",
        f"# seed: {args.seed}",
        "#",
        "# Positions are in the work_surface frame, metres.",
        "# This is the SAME data used to build the world, so the oracle",
        "# detector is reading genuine ground truth, not a second guess.",
        "",
        "work_surface:",
        f"  centre_x: {WORK_CENTRE_X}",
        f"  centre_y: {WORK_CENTRE_Y}",
        f"  half_extent: {HALF}",
        "",
        "weeds:",
    ]
    for name, u, v, s, r in weeds:
        truth.append(f"  - {{ id: {name}, u: {u:.5f}, v: {v:.5f}, size: {s:.3f} }}")
    truth += ["", "crops:"]
    for name, u, v, s, r in crops:
        truth.append(f"  - {{ id: {name}, u: {u:.5f}, v: {v:.5f}, size: {s:.3f} }}")

    open(args.world_out, "w").write(world)
    open(args.truth_out, "w").write("\n".join(truth) + "\n")

    print(f"world  -> {args.world_out}   ({len(crops)} crops, {len(weeds)} weeds)")
    print(f"truth  -> {args.truth_out}")


if __name__ == "__main__":
    main()

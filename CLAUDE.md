# Jim — laser weeding gantry, ROS 2 Jazzy capstone

Read this before touching anything. It encodes decisions that took hours
to reach; re-deriving them wastes time we do not have.

## What the machine is

A static 2-DOF Cartesian gantry over a 250 x 250 mm plot. A camera finds
weeds, the gantry positions a laser head over each one. No mobile base.
No arm. Two prismatic joints, that is the whole robot.

## Architecture decisions already made — do not revisit

- **NO Nav2.** There is no mobile base, no odometry, no costmap. Nothing
  for it to do.
- **NO MoveIt.** 2-DOF Cartesian IK is the identity function plus a
  constant offset. `targeting_node.py` solves it in four lines using TF.
  This is a defensible engineering decision, written up for the report.
- **ros2_control + JointTrajectoryController** on the two prismatic joints.
- **Gazebo Harmonic** via `gz_ros2_control`. Hardware later via a custom
  `hardware_interface::SystemInterface` speaking serial to an Arduino.
- **Oracle detector, not YOLO.** Reads the same ground truth that built
  the world, injects configurable noise / misses / false positives.
  Publishes `vision_msgs/Detection3DArray`. A real detector would publish
  the same message on the same topic and nothing downstream changes.

## Hard numbers

```
travel            0.250 m both axes
lead screw        T8, 2 mm pitch x 4 start = 8 mm/rev
steps/mm          25 full step, 400 at 16x microstepping
resolution        0.04 mm/full step
max velocity      0.035 m/s   <- lead screw CRITICAL SPEED, not motor torque
                                 4.76e6 * 6.2 / 300^2 = 328 rpm, 80% derate
motors            NEMA 17, 1.8 deg, run at 0.8-1.0 A NOT 1.5 A
                  (1.5 A puts the case at ~88 C, PLA Tg is 60 C)
drivers           MKS TMC2209, StallGuard sensorless homing
torque needed     ~13 mN.m against ~400 mN.m available (12x margin)
```

## Frame conventions

```
world
└── base_link              300 mm above soil
    ├── x_carriage         prismatic joint_x, 180 deg roll from CAD
    │   └── y_carriage     prismatic joint_y
    │       └── laser_aim_link    Z axis IS the beam direction
    └── work_surface       soil plane, origin at centre of reachable area
                           static TF: (0.1837, -0.2516, -0.30) from base_link
```

- **Metres and radians everywhere** (REP-103). Exactly ONE unit conversion
  in the whole stack, inside the hardware interface. ROS never sees steps.
- `joint_x` carries a 180 deg roll from the CAD export, so machine +Y runs
  along world −Y. This is why `targeting_node.solve()` negates joint_y.
  It is not a bug. Do not "fix" it.
- All weed positions are in `work_surface`, which keeps CAD offsets out of
  the targeting maths entirely.

## Packages

```
jim_description   xacro, meshes, ros2_control tags, RViz config
jim_bringup       controller config, plain sim launch, bare world
jim_sim           field world + generator, oracle detector, targeting node
```

## Landmines already hit — do not repeat

1. **`ParameterValue(Command(...), value_type=str)`** is mandatory for
   `robot_description`. Without it the parameter layer tries to YAML-parse
   the URDF and dies.
2. **`GZ_SIM_RESOURCE_PATH`** must point at the parent of the package share
   dir. Gazebo cannot read the ament index, so `package://` URIs mean
   nothing to it. Symptom: "Failed to load geometry for visual".
3. **`/clock` needs an explicit `ros_gz_bridge`.** Gazebo publishes its
   clock on gz-transport, not DDS. Every `use_sim_time` node waits forever
   otherwise.
4. **`use_sim_time: True` on EVERY node**, including controller spawners.
5. **Controller spawn ordering** must be Gazebo → spawn robot → controllers.
   The controller_manager lives *inside* Gazebo, created by the plugin.
   Use `OnProcessExit` handlers, never sleeps.
6. **`colcon build` copies src → install.** Gazebo only ever reads install.
   Unsaved edits, skipped rebuilds and stale shells all look identical.

## Known open bug

`joint_y` did not move in the first bringup. Error tracked whatever
tolerance was set, which is the signature of a joint whose setpoint ramps
away from a stationary actual position.

Next diagnostic step:
```
ros2 control list_hardware_interfaces
```
If `joint_y/position` shows as a claimed command interface, the joint is
bound and something is physically blocking it — suspect self-collision
between `y_carriage` and `x_carriage`, whose collision boxes overlap.
Fix is to delete collision geometry from both carriages; nothing in this
machine collides with anything.

If it does not show as claimed, the `ros2_control` block failed to bind
the joint, which is a different bug in `jim.ros2_control.xacro`.

## Running it

```bash
# host
cd ~/Jim && export HOST_UID=$(id -u) HOST_GID=$(id -g)
docker compose up -d
xhost +local:docker        # resets every login

# container
docker exec -it jim bash
cd ~/ws && colcon build && source install/setup.bash
ros2 launch jim_sim field.launch.py
```

## What is left, in priority order

1. Fix `joint_y`
2. Supervisor: state machine as an action server (master context section 48)
3. Safety: heartbeat, e-stop, workspace rejection
4. **Validation harness** — spawn N weeds, run headless, log commanded vs
   actual, output a histogram. This is the highest-value deliverable for
   the grade and the easiest to run out of time on. Do not cut it.
5. Polish: one-command launch, README, architecture diagram

## Working style

The person is a capstone student, rusty on practical ROS, strong on the
mechanical and control reasoning. Explain *why*, not just what — they have
caught two real errors by understanding rather than pasting. Do not
dumb things down and do not skip the reasoning.

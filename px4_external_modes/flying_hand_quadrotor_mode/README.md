# Flying Hand Quadrotor External Mode

`flying_hand_quadrotor_mode` is a dynamically registered PX4 ROS 2 External Mode for the
four-rotor x500_arm2x. PX4 keeps estimation, external-mode supervision, and
control allocation. The ROS 2 process owns whole-body control and publishes the
standard `VehicleThrustSetpoint` and `VehicleTorqueSetpoint` inputs through
`WrenchSetpointType`; it does not publish `OffboardControlMode`.

## Controller

The deployed controller is fully contained in this package:

```text
EE pose target
  -> 17-state, 8-input ACADOS SQP-RTI NMPC
  -> matched L1 force/moment and arm-servo correction
  -> exact four-rotor box projection
  -> PX4 normalized thrust/torque and ACE arm position commands
```

Its architecture follows the nominal whole-body optimization plus adaptive
disturbance-rejection split described by [Flying
Hand](https://arxiv.org/html/2504.10334v1), constrained here to the feasible
collective-thrust and body-moment wrench of a conventional quadrotor.
This is the repository's underactuated x500 adaptation, not the paper's native
tilted-rotor fully actuated aircraft. Use `flying_hand_fully_actuated_mode` for
the separately calibrated six-rotor architecture reconstruction.

The NMPC horizon is 2.5 s with 100 shooting intervals at 100 Hz. Its control is
`[collective thrust, Mx, My, Mz, arm_position_cmd[4]]`. Nonlinear path constraints
enforce all four rotor thrust limits about the current whole-body center of mass;
linear constraints enforce all four arm velocity limits. The objective tracks the
full end-effector SE(3) pose while penalizing base tilt, velocity, angular rate,
arm deviation, energy, and control change. The generated C solver is checked in,
so normal deployment does not require CasADi or Python.

The model is deliberately mesh-free. The installed control URDF defines joint
transforms, limits, link inertials, and the end-effector frame. At runtime, fixed
dimension kinematics recompute the whole-body center of mass, inertia about that
center of mass, and center-of-mass Jacobian from the current arm configuration.
Those properties parameterize each generated-solver call and the rotor allocation
matrix. They are frozen over that individual horizon and refreshed at the next
control sample. The model includes first-order arm servos, but does not claim
collision avoidance or exact moving-arm momentum coupling over the horizon; the
feedback and adaptive layers absorb the remaining model residual. The adaptive
layer compensates matched body-Z force, three moments, and arm servo delay;
estimated body-X/Y disturbance is fed to the next NMPC solve but is never sent as
an impossible lateral force.

The requested wrench is projected twice before publication: first into the
physical four-rotor thrust box, then into the PX4 normalized control and actuator
boxes using the calibrated nonlinear throttle-to-thrust relation. The physical
wrench reconstructed from the final actuator command is the value used by the L1
predictor, slew state, and diagnostics.

## Frames

- Vehicle position, velocity, target EE pose, and published EE pose use local NED.
- Vehicle attitude is the PX4 body-FRD to local-NED quaternion.
- Physical wrench diagnostics use `base_link_flu`.
- PX4 setpoints use normalized body FRD conventions.
- The target `PoseStamped.header.frame_id` must be `px4_local_ned` by default.

## ROS 2 Interface

Inputs:

- `/fmu/out/vehicle_odometry`
- `/fmu/out/sensor_combined`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_land_detected`
- `/ace_follower/arm/state`
- `/ace_follower/arm/sync_status`
- `/flying_hand_quadrotor/ee_pose_setpoint`

Outputs:

- `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/in/vehicle_torque_setpoint`
- `/ace_leader/arm/command`
- `/ace_leader/gripper/command`
- `/ace_leader/arm/sync_mode`
- `/flying_hand_quadrotor/ee_pose`
- `/flying_hand_quadrotor/wrench_nominal`
- `/flying_hand_quadrotor/wrench_adaptive`
- `/flying_hand_quadrotor/wrench_applied`
- `/flying_hand_quadrotor/status`

All names are ROS parameters. PX4 topics use the interface library's message
version suffix and optional `px4_topic_namespace_prefix`.

## Safety

The mode cannot arm the vehicle and is available only when the vehicle is an
armed, airborne multicopter with fresh odometry, IMU and arm state, a ready
follower, no PX4 failsafe, no competing publisher on any arm, sync, thrust, or
torque command topic, and an installed controller callback. If no EE target has
arrived, the current EE pose is latched on entry.

The controller runs at 100 Hz. A solve exceeding 8 ms reuses the previous
feasible output; three consecutive overruns latch a fault. Invalid output and
high-rate state older than 20 ms fault immediately. The lower-rate PX4
`VehicleStatus` and `VehicleLandDetected` streams use separate 750 ms and 1500 ms
freshness limits. Replayed or non-monotonic PX4 source timestamps are rejected;
the follower `JointState` must provide a monotonic ROS timestamp plus position and
velocity for all four arm joints. On fault, the last feasible wrench is held briefly while the
executor publishes a non-blocking Position mode request up to three times and
checks `VehicleStatus` for confirmation, then continues retrying at 1 Hz if no
confirmation arrives. The producer never intentionally generates a zero-thrust
setpoint. Mode
exit holds the final arm pose, preserves the gripper opening, and publishes arm
sync mode `stop` for one second before returning to `sync_request` so a later mode
entry remains possible.

PX4 exposes thrust and torque as two standard DDS topics, so ROS 2 cannot make
their delivery atomic without a PX4 interface change. This package publishes
thrust first, torque second, with the same `timestamp_sample`; the PX4 allocator
is driven by torque and reads the latest thrust. Closed-loop deployment must
still verify this pairing under the target DDS load; end-to-end absence of a
mispaired initial or stale thrust sample cannot be guaranteed by this external
package alone.

When the mode is inactive, shadow execution commits only its private controller
state and diagnostics; it does not publish flight or arm commands. Activation
resets the predictor and filters before the first closed-loop solve.

## Profiles

```bash
# Simulation closed-loop profile
ros2 launch flying_hand_quadrotor_mode sim_flying_hand_quadrotor.launch.py

# Real vehicle: computation and diagnostics only
ros2 launch flying_hand_quadrotor_mode real_flying_hand_quadrotor_shadow.launch.py

# Real vehicle closed loop; enable only after shadow validation
ros2 launch flying_hand_quadrotor_mode real_flying_hand_quadrotor.launch.py
```

Take off in PX4 Position mode, verify `/flying_hand_quadrotor/status`, then
select the dynamically registered **Flying Hand Quadrotor** mode. Return to
Position before landing.
The real closed-loop profile should remain unused until its shadow timing,
state-age, actuator-saturation, and EE-error acceptance checks pass.

## Build And Code Generation

```bash
git submodule update --init --recursive third_party/acados_vendor/vendor/acados
colcon build --packages-select acados_vendor px4_msgs px4_ros2_cpp flying_hand_quadrotor_mode
colcon test --packages-select flying_hand_quadrotor_mode
```

Regenerating the solver is an explicit development operation:

```bash
renderer=third_party/acados_vendor/vendor/acados/bin/t_renderer
mkdir -p "$(dirname "$renderer")"
curl -fL \
  https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64 \
  -o "$renderer"
echo "1973ddd1b536dcd4a059a22f7259c3b0e6b6c878cbfbe7e9b962eba195f361d9  $renderer" \
  | sha256sum -c -
chmod +x "$renderer"
python3 -m pip install -r px4_external_modes/flying_hand_quadrotor_mode/tools/requirements-codegen.txt
export ACADOS_SOURCE_DIR=$PWD/third_party/acados_vendor/vendor/acados
python3 px4_external_modes/flying_hand_quadrotor_mode/tools/generate_solver.py
```

The generator rejects an ACADOS checkout that is dirty, incomplete, or not at the
pinned revision before importing its template package. It also requires the
ACADOS v0.5.5 Linux x86_64 `t_renderer` at the pinned SHA256 and refuses a missing
or substituted renderer instead of downloading one implicitly.

The mesh-free control URDF can be regenerated from the audited x500_arm2x source
with:

```bash
python3 px4_external_modes/flying_hand_quadrotor_mode/tools/generate_control_urdf.py \
  ../ACESim/acesim/env/mujoco/asset/x500_arm2x/x500_arm2x.urdf
```

The expected source SHA256 is embedded in the generated output. The checked-in
control URDF remains the deployment source of truth; the full visual/collision
asset is intentionally not duplicated in this repository.

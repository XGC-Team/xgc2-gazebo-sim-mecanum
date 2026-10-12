# Native validation

The Mecanum plugin, in ideal and in high-fidelity mode, is verified with the Scout plugins in one headless
`gzserver` by `scout/test/native/verify.py` of `gazebo-sim-scout`; see `docs/native-validation.md` of that package for
the world, the commands and the checks. The test builds the Mecanum assets from this checkout
(`models/xgc2_mecanum_ugv/model.sdf.xacro`, passed as `--mecanum-source`), without their visuals, and loads the plugin
that this package installs.

2026-10-10: at the fixture's fixed 1 ms physics step, all 24 checks passed in ten consecutive runs (240 checks total, 59.2-72.0 seconds per run),
using XRPC C++ 0.2.0 at `d33afa8` and chassis-hold 0.2.0 at `fcfe8f4`, both built into
`/home/node/workspace/build/gazebo-hold/prefix`. Every run used private ROS and Gazebo masters and its own
ROS home, with no surviving processes or collected orphans. Two of the twenty gzserver shutdowns reported
the known Gazebo transport teardown abort that the existing test tolerates.

What it establishes for this plugin, in both drive models: the plugin joins the HOLD roster of the world by its
entity ID and leaves it when it is deleted; engage writes zero (`zero_written`) and the robot comes to rest
(`stopped`, from the horizontal speed and yaw rate of the model; the high-fidelity model brakes through its wheel
loops, the ideal model stops at once); a held robot ignores the commands that keep arriving, the other robots keep
moving; commands queued before a release are not replayed; HOLD works while the world is paused; a deleted and created
model starts held; a new world host starts with nothing held.

The package rostests were then run against an isolated prepared native world, its discovered ServiceRef and
the Python simulation-v1 client, now owned by `xgc2_gazebo_scene.simulation_client`:
`ideal_drive.test` passes all three cases, and `high_fidelity_drive.test` passes all six,
including repeated HOLD and model deletion/recreation through `test/native_simulation.py`.
Focal controlled-toolchain ABI, installed packages and calibration remain unverified.

2026-10-13 source correction: the former explicit wheel-speed P loop was unstable at the station's 4 ms
physics step. With the authored wheel-axis inertia of 0.00048 kg m^2 and P = 0.35, the unsupported wheel's
discrete multiplier was 1 - P * dt / inertia = -1.9167; the ground reaction adds further slip stiffness.
The old 1 ms HOLD receipt therefore does not establish braking at 4 ms.

The motor now evaluates P at the implicit end-of-step wheel speed, including the existing ground reaction.
For reaction torque R, current speed w and target wt, its torque before saturation is
`(P * (wt - w) + (P * dt / inertia) * R) / (1 + P * dt / inertia)`.
The same motor torque limit applies. Wheel-axis inertia comes from the loaded wheel link; the world's
physics step is used for the first/reset update, which has no elapsed simulation time yet. Steady-state
P/reaction balance, contact traction, model parameters and HOLD admission remain unchanged. Only joint
forces are written; Gazebo still integrates the high-fidelity wheel and body states.

For this correction, source syntax compilation and an offline numerical check of the compiled torque
expression at 1 ms and 4 ms passed. The reduced planar wheel/body plant checks decay, torque saturation
and nonzero-target equilibrium; it omits changing contacts and full rigid-body constraints. No new Gazebo
or station run was performed. Installation and real Wheeltec Start/Stop braking at 4 ms remain to be verified.

2026-10-13 native startup correction: in the failed Wheeltec source-forward session
`af292604-d31a-5e68-a759-31c7dc28fb1c`, Create rejected the chassis after the native model-load ack.
The deferred ROS registration callback had also deferred the physical initialization and HOLD `Ready()`;
the four later initialization messages do not establish that the chassis was bound when Create checked it.

`Load` now completes SDF validation, physical link/joint/contact binding, the HOLD seat, the physics update
connection and `Ready()` synchronously before it returns. The existing process-wide master watcher defers
only ROS parameter reads, publishers, TF broadcaster and the command subscription. The TF broadcaster is
also constructed there, rather than as an eager plugin member. No ROS master registration or parameter call
runs on the native loading thread. The physics update and HOLD drive law run independently of ROS IO;
telemetry starts only after its ROS resources have been registered. The existing HOLD command queue and
registration cancellation remain the owners of command admission and callback teardown.

Source syntax compilation passed for this correction; the implicit 4 ms motor expression and all model,
step-size and dynamics parameters are unchanged. No ROS/Gazebo process, station or Docker test was started.
Installation, actual late-master startup and Wheeltec Start/Stop braking at 4 ms still require the source-forward
E2E run owned by the deployment operator. The historical receipts above do not establish those results.

2026-10-13 deployment source-forward proof: the exact candidate built and linked with the Core Focal
GCC 9 / CMake toolchain. Session `175d434d-1a8e-5d07-a5d9-d2add494b957` created all four native chassis
with the original ODE step 0.004 s and update rate 250 Hz. A single Start moved the four bodies
1.2498, 0.5479, 1.2298 and 1.2160 m in the observed 5-second interval. After the single Stop, all four
cmd_vel streams were zero; during the final 5 seconds of the 8-second truth observation, maximum
horizontal speed was 5.147e-8 m/s and maximum yaw rate 3.648e-8 rad/s. These are Gazebo-native truth
observations, without ideal mode, body velocity writes, parameter tuning or a shorter world step.
The scene, model and motor/traction parameters were retained. Source build and stationary braking
are now verified for this configuration. This receipt does not establish actual late-master startup,
full repeated-HOLD coverage, hardware behavior or a new APT production release. Raw evidence:
`/tmp/xgc-tase-wheeltec-e2e/source-forward2-*` and `/tmp/xgc-mecanum-source-runtime-20261013/native-init-*`.

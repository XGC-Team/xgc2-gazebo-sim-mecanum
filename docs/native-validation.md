# Native validation

The Mecanum plugin, in ideal and in high-fidelity mode, is verified with the Scout plugins in one headless
`gzserver` by `scout/test/native/verify.py` of `gazebo-sim-scout`; see `docs/native-validation.md` of that package for
the world, the commands and the checks. The test builds the Mecanum assets from this checkout
(`models/xgc2_mecanum_ugv/model.sdf.xacro`, passed as `--mecanum-source`), without their visuals, and loads the plugin
that this package installs.

2026-10-10: all 24 checks passed in ten consecutive runs (240 checks total, 59.2-72.0 seconds per run),
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

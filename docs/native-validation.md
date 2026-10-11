# Native validation

The Mecanum plugin, in ideal and in high-fidelity mode, is verified with the Scout plugins in one headless
`gzserver` by `scout/test/native/verify.py` of `gazebo-sim-scout`; see `docs/native-validation.md` of that package for
the world, the commands and the checks. The test builds the Mecanum assets from this checkout
(`models/xgc2_mecanum_ugv/model.sdf.xacro`, passed as `--mecanum-source`), without their visuals, and loads the plugin
that this package installs.

What it establishes for this plugin, in both drive models: the plugin joins the HOLD roster of the world by its
entity ID and leaves it when it is deleted; engage writes zero (`zero_written`) and the robot comes to rest
(`stopped`, from the horizontal speed and yaw rate of the model; the high-fidelity model brakes through its wheel
loops, the ideal model stops at once); a held robot ignores the commands that keep arriving, the other robots keep
moving; commands queued before a release are not replayed; HOLD works while the world is paused; a deleted and created
model starts held; a new world host starts with nothing held.

Not covered: the rostests of this package (`ideal_drive.test`, `high_fidelity_drive.test`) need a prepared world, a
ServiceRef and `xgc2_scene_runtime`; their HOLD helper (`test/native_simulation.py`) calls the HOLD methods, but the tests
were not run for this migration. Focal controlled-toolchain ABI, installed packages and calibration are open.

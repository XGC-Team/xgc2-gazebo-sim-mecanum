# Chassis HOLD

The Mecanum simulation takes part in chassis HOLD. The contract is the one of `xgc2-chassis-hold`
(`contracts/chassis-hold.md` of `libxgc2-chassis-hold-dev`); the Gazebo binding (the `xgc2.chassis.hold` capability of the
world host, dynamic roster, command admission, native tick, feedback) is described in
`xgc2_gazebo_scene/docs/chassis-hold.md` of `ros-noetic-xgc2-gazebo-scene`. This page lists only what the Mecanum
plugin contributes.

* The plugin element `chassisRobotId` is the entity ID. The plugin reserves it with `xgc2_gazebo_scene::ChassisHold`
  when it loads and joins the HOLD roster when its native setup is complete; it leaves when it is destroyed. A model
  that is created again under the same ID starts held if it was held. The ROS namespace and the model name
  authorize no control.
* `cmd_vel` is subscribed on the world's command queue, which records when a message was queued. A command that
  is not admitted (the robot is held, or the command was queued before the last release) never reaches the
  plugin's command state, however late it is dispatched.
* The control step runs under the lock of the HOLD seat, so it never overlaps the zero output of the native
  tick. While the world is paused the tick runs on its own thread.
* Zero output, written on every tick while held: zero front, left and yaw commands and zero effort on the four
  wheel joints; in ideal mode also zero rigid-body velocity. In high-fidelity mode the control step keeps running
  its wheel-speed loops with the zero commands, so the wheels brake and the Mecanum forces still act.
* Feedback for `stopped` is the horizontal speed and yaw rate of the model.

Zero output is not a physical stop: inertia and contact response are separate facts. The test in
`gazebo-sim-scout` (`scout/test/native/verify.py`, see its `native-validation.md`) runs this plugin, both drive
models included, with the Scout plugins in one headless world.

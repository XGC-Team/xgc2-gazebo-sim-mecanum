# Native HOLD and lifetime

The ModelPlugin binds explicit chassisRobotId to its native world. The ID equals
the public simulation-v1 entity ID and must be declared in the prepared world's
at-most-16 chassis_robot_ids grant. ROS namespace/private model name authorize
no control. Ready() follows complete successful native setup. Creation fails
for absent authority, identity or readiness.

One world provider/listener owns /v1/chassis/hold. Read the applied revision, then
supply expected_revision and {robot_id, held} changes. Successful applied HOLD
includes native zero-sink completion. Per-robot UDP Gate/Hub and listener threads
are removed.

ROS command admission, command state and actual native wheel/velocity output
share ChassisBinding::with_command. It tries the fixed world domain lock once;
contention skips that sample. Domain lock precedes the command mutex. HOLD clears
state and all native wheel efforts; ideal mode also applies zero rigid-body
velocity. Release keeps zero cached targets until a fresh command. Physical
inertia/contact response is distinct from zero actuator output.

Models use one fixed shared world ROS dispatcher with no model spinner thread.
Owner-specific queue entries capture an epoch. HOLD/release/reset invalidate it
without native allocation; stale entries consume the underlying SubscriptionQueue
but skip their action. Admission rechecks the epoch inside the world transaction,
so dispatch delayed until after release cannot revive pre-HOLD commands.

Native deletion zeroes/retires before Gazebo link Fini. Teardown disconnects
updates, closes subscriptions, drains admitted callbacks and unregisters bindings.
Binding destruction never touches finalized joints. Gazebo start/stop remains
explicit process management; the provider controls the running domain only.
See native-validation.md for evidence and the unresolved process shutdown gate.

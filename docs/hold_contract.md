# HOLD execution and lifetime

Command receipt and the actual Gazebo actuator/velocity update execute within
the same Gate transaction used by HOLD transitions. A HOLD acknowledgement
therefore follows previously admitted command writes and the zero-target
callback. Subsequent updates use zero targets while held. Physical wheel
braking, inertia and contact response are distinct from instantaneous stopping.
Releasing HOLD does not restore the old command cache.

The Gate is registered before the ROS command spinner starts. Shutdown first
drains Gazebo updates and ROS command callbacks, then unregisters/drains UDP
callbacks, and only then destroys the Gate and its owner. Lock order is registry,
Gate, then command state. Zero callbacks must not re-enter Gate or registry APIs.

These changes do not alter the UDP endpoint or solve routing between multiple
processes sharing that endpoint. They also do not change the high_fidelity
traction/contact model or the deliberate latest-command retention policy.

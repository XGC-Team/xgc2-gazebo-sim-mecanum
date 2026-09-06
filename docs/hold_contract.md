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

## Simulator routing

Each simulated robot exclusively binds a loopback UDP port computed from its
UTF-8 robot ID: `20000 + FNV1a32(id) % 20000`. FNV-1a uses offset basis
2166136261 and prime 16777619, with 32-bit unsigned overflow. Core uses the same
mapping for simulation resources; physical firmware retains port 19520.
IDs must be nonempty and shorter than 32 bytes. The datagram still contains the
exact robot ID, and an endpoint rejects frames addressed to another ID.

This permits multiple processes without `SO_REUSEADDR` packet competition.
An occupied port (including a hash collision or duplicate robot ID) fails
registration explicitly; choose a distinct ID instead of sharing an endpoint.
Sender and simulator updates must be delivered together. This local simulator
control protocol is not an authenticated remote control interface.

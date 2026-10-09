# Native validation — 2026-10-09

Scope is isolated native semantics on host Ubuntu 24.04.5 amd64, GCC 13.3,
relocated ROS Noetic and Gazebo **Classic 11.15.1**. This is not Harmonic support,
Focal ABI acceptance, APT installation/publication, or physical calibration.
User hard stop forbids further host Gazebo/fault/GDB launches; no such retest was
run after the stop. New source fixes and metadata received static checks only.

The actual test owner is Scout `test/native/verify.py`, with CMake fixture
`test/native/CMakeLists.txt`. It compiles the real Scout skid/unicycle/implicit
and Mecanum ideal/physical ModelPlugins. It links the shared scene service at
`/tmp/sol6-gazebo-native/libxgc2_simulation_service.so`, the formal native SDK
installed at `/tmp/sol6-xrpc-install`, and common/math headers. The world loads
the formal ROS data plugin from the world-plugin directory. The Python consumer
uses formal common/xrpc source with dependencies from
`/tmp/sol6-camera-python-deps`, under explicitly selected Python 3.12.

The fixture owns a private temporary resource root, ROS master, Gazebo master
and UDS socket. It starts only gzserver, never gzclient. Its child environment
explicitly removes DISPLAY and WAYLAND_DISPLAY, and logs actual PID, resolved
executable, argv, version and rootfs. No rendering sensor is present. Earlier
headless attempts inherited host DISPLAY=:1; that was corrected before the
recorded five-model semantic proof. No desktop display is used by the corrected
fixture.

Actual semantic evidence: `/tmp/sol6-chassis-native-verify.log` and preserved
`/tmp/sol6-chassis-native-sigint-failure.log`. Five owners used distinct ROS data
namespaces and granted public chassis IDs. Thread counts after creating each
model were `[55,55,55,55,55]`. A test-only native world extension blocks the
single shared ROS dispatcher, admits old Twist messages, applies HOLD, queues
held messages, releases, then unblocks dispatch and steps physics. Old targets
and actual native wheel velocities remained zero; fresh commands subsequently
produced actual native motion. Paused HOLD zero completion, wrong identity and
stale-revision rejection, and native Delete during command publication passed.
The minimal asset verifies ownership/output, not calibrated ground contact.

**Graceful process shutdown is unresolved.** The SIGINT run used exact gzserver
PID 3723466 and roscore PID 3723450. Gazebo removed the socket, but exited -11;
ROS master exited 0. Earlier SIGTERM killed the process and did not establish
native Fini/drain. `/tmp/sol6-chassis-native-gdb.log` preserves a pre-stop
backtrace: DataSource destruction in the formal ROS data plugin releases a
strong WorldPtr late, reaching World::~World/World::Fini/DiagnosticManager::Fini;
a thread aborts during TLS deallocation with invalid-pointer diagnostics. GDB
wrapper exit 0 does not mean its inferior exited normally. These failures remain
visible and are not replaced by a successful cleanup claim. Exact processes
were already ended; no historical unknown PID is guessed or killed.

The fixture source now stops its precise gzserver PID with SIGINT, waits at most
10 seconds, kills only its own timed-out child and fails the result for timeout,
nonzero native exit, surviving socket or client-close error. It always stops
owned processes even if SDK close throws. This cleanup source was statically
checked after the hard stop; no new graceful pass is asserted.

Compile-only reproduction uses the Scout native CMake source and explicit
SCENE_INCLUDE, SIMULATION_LIBRARY, MATH_INCLUDE, MECANUM_SOURCE, XgcXrpc_DIR,
catkin_DIR and relocated rosconsole_DIR. The native model build completed in
`/tmp/sol6-chassis-models`; all four production owner DSOs and dispatcher fixture
linked. Existing Scout numerical tests passed six delay/stop/reverse/clock
scenarios. Existing wiring/default checks passed 9, 11 and 15 unittest cases;
they are source checks, not extra physics evidence. XML/Python syntax and scoped
git diff checks are recorded with final handoff.

Full scientific vehicle fixtures now require explicit prepared world,
simulation_service_ref_json and target_id. Their roster must grant all IDs used
by the test: Scout combined `scout1,scout2,mecanum1`; unicycle/implicit their
configured primary ID; Mecanum lifetime regression `ugv1,ugv2`. Generic objects
are native entities too. World start/stop stays an explicit launch workflow;
model creation, state and HOLD use the already-running domain. No RPC process
supervisor was introduced. Full wheel/contact/rollover/weak-network matrices,
Focal controlled-toolchain runtime and deployment remain open.

Packaging revisions are new unpublished source candidates. Models require
libxgc2-xrpc1>=0.1.0, scene/world packages>=1.4.1-4, and their existing descriptions.
Formal Python SDK has no Debian package: Python consumers require an explicitly
selected interpreter>=3.8 and the official wheel in that interpreter. The
system Python is not replaced. The deployment release-set snapshot is not
changed or claimed as a newly published compatible combination.

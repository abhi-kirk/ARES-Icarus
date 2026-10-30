# The landing

Three phases of a powered descent, in C++. The rocket is a point mass with a tank. Thrust is a force. There is no attitude in this tree, and no gimbal. The loop is finished before the vehicle is allowed to turn.

| Phase | What it shows |
|---|---|
| 1 | A vertical landing. Cascade PID, one degree of freedom. |
| 2 | The same loop in three dimensions: wind, drag, and a pad that moves. |
| 3 | Why that loop can land a rocket it cannot hover. The hover is linearly unstable. Touchdown works because the rocket does not stay there. |

<p align="center">
  <img src="images/trajectory_pid_3dof.png" alt="Phase 2 trajectory, altitude, velocity, and thrust" width="900"/>
</p>

Phase 2, from 300 m. The outer loops command a reference velocity. The inner loops turn velocity error into thrust, with gravity feedforward on the vertical channel. Fuel mass drops along the way. The landing mark is the pad, not the point the pad started on.

The writeups are in [docs/](docs/): stability, mechanics, and the build.

Rocket Dynamics scenes on [icarus-arc.com](https://icarus-arc.com) keep their own clock in the browser, so the picture and the chart are the same buffer. The Simulator streams this landing. The page sends a case, and `IcarusStream` writes each sample as the flight runs. `engine/listen/` is the HTTP listener in front of that binary. The public machine is the Fly app `icarus-arc-flight` in Dallas. It stops a few minutes after the last connection and starts on the next request. `make run` is still the Rerun landing on this computer. The flight image builds with Rerun off.

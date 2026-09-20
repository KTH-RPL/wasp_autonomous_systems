# Simulation speed under WSL2

WSL2 normally has no GPU acceleration, so Mesa falls back to its software
renderer and every frame of the simulator's 3D view is drawn on the CPU. That
view is by far the most expensive thing running.

Measured on one 16-core laptop running Assignment 1.1 under Webots:

| | real-time factor |
| --- | --- |
| rendering on | 0.12 |
| rendering off | ~0.8 |

The same assignment on a machine with a working GPU runs at 0.97, so this is
about software rendering rather than about Webots or WSL2 being slow in
themselves. Adding cores does not help; the machine above already had 16.

## What to do

Turn rendering off from Webots' **View** menu.

The simulation, the sensors and everything ROS sees keep working. The camera
image and the point cloud still update in RViz, so you lose only the
third-person view of the scene, not any data an assignment asks you to look
at. Turn it back on whenever you want to watch the robot.

Nothing desynchronises when the simulation runs slowly. Every node runs on
simulated time, so the robot simply responds more slowly in wall-clock terms.

## Checking whether this applies to you

```
sudo apt install mesa-utils
glxinfo -B | grep "OpenGL renderer"
```

If that reports `llvmpipe`, you are rendering on the CPU.

Updating your GPU driver on the Windows side sometimes enables hardware
rendering, but do not count on it. One laptop tested for this course had a
current NVIDIA driver, the GPU visible to `nvidia-smi` inside WSL2, `/dev/dxg`
present, Mesa's `d3d12` driver installed and WSL itself updated - and still had
no hardware OpenGL, because WSLg's X server reported no DRI3 support, leaving
Mesa no route to the GPU. Forcing the driver with
`MESA_LOADER_DRIVER_OVERRIDE=d3d12` changed nothing.

So treat turning rendering off as the fix, not as a workaround while you hunt
for a driver problem. It may well not be one.

Tasks that only play rosbags, such as Assignments 1.2 and 1.3, involve no
simulator and are unaffected.

The Gazebo alternative is affected by the same thing, and measured slower than
Webots-with-rendering-off on the same machine - see
[gazebo-alternative.md](gazebo-alternative.md).

# Note: why WSL2 gets no GPU acceleration

Maintainer note, not student-facing. `performance-wsl2.md` is the page students
read; this records why it tells them to turn rendering off rather than to fix
their drivers.

Tested 2026-09-20 on a Windows laptop with an NVIDIA GPU and an Intel iGPU,
which nonetheless ran Webots' 3D view entirely in software (`llvmpipe`) at 0.12x
real time. All of the following were true at once:

- the NVIDIA card was enabled and listed in Task Manager
- `nvidia-smi` worked inside WSL2, driver 590.60
- `/dev/dxg` existed with the right permissions
- `libd3d12.so` and `libdxcore.so` were present in `/usr/lib/wsl/lib` and found
  by `ldconfig -p`
- `/etc/ld.so.conf.d/ld.wsl.conf` was correct
- `d3d12_dri.so` was installed in `/usr/lib/x86_64-linux-gnu/dri/`
- `DISPLAY=:0` and `WAYLAND_DISPLAY=wayland-0`, i.e. WSLg's own display with no
  stale X-server override
- `wsl --update` had been run
- reproduced on both Ubuntu 26.04 and a clean Ubuntu 24.04

The blocking condition, from `LIBGL_DEBUG=verbose glxinfo -B`:

```
screen 0 does not appear to be DRI3 capable
```

Without DRI3 the GLX path cannot reach any hardware driver, so Mesa falls back
to software no matter what is installed. `MESA_LOADER_DRIVER_OVERRIDE=d3d12` is
ignored without error, which is consistent: Mesa never attempts the driver.

Conclusion: nothing in this repository, and nothing we can reasonably ask a
student to do, changes this. Do not send students driver-hunting. If someone
wants to revisit it, the open question is why WSLg's Xwayland comes up without
DRI3 on this hardware - not whether the GPU or the drivers are present, which
was ruled out above.

Measurements behind `performance-wsl2.md`, all Assignment 1.1 with the
simulator window open unless stated:

| configuration | real-time factor |
| --- | --- |
| Webots, macOS with a working GPU | 0.97 |
| Webots, WSL2, rendering on | 0.12 |
| Webots, WSL2, rendering off | ~0.8 |
| Gazebo, WSL2, window on | 0.45 |
| Gazebo, WSL2, window off | 0.63 |
| Gazebo, WSL2, `ass_2_collision` | 0.70 |
| Gazebo, WSL2, `ass_4_manual` | 0.80 |

Webots' own toolbar reading and a figure computed from `/clock` agreed to three
digits on macOS (0.97 vs 0.973), so the GUI readouts and the computed ones are
comparable.

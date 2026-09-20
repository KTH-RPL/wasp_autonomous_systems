#!/usr/bin/env python
"""Shared bits of the Gazebo launch files in this package."""

import glob
import hashlib
import os
import re
import sys
import tempfile

from launch.actions import ExecuteProcess, TimerAction


def gui_enabled(value):
    """Resolve the `gui` launch argument, including its "auto" default.

    "auto" means on. The argument exists so it can be turned off - most
    usefully under WSL2, where the 3D window falls back to software
    rendering and can be slower than it is worth.
    """
    value = str(value).strip().lower()
    if value in ('false', '0', 'no', 'off'):
        return False
    return True


def gz_args(world, verbosity=2, run=True):
    """Build the argument string for ros_gz_sim's gz_sim.launch.py.

    Always `-s`, server only. The 3D window is a second `gz sim -g` process
    (see gz_gui below) rather than the combined server+GUI form, because the
    combined form has never worked on macOS - that is what
    gazebosim/gz-sim#44 is about, and it is why everything in this repo used
    to run headless on every platform. Split into two processes it works on
    macOS as well (verified live), so the same launch files give every
    platform a real Gazebo window.

    run=False drops `-r`, so the world loads paused - see
    gz_unpause_when_gui_ready.
    """
    prefix = '-s -r' if run else '-s'
    return f'{prefix} -v{verbosity} {world}'


def _stock_gui_config():
    """Path to the gz-sim GUI config that ships with the installed Gazebo."""
    prefix = os.environ.get('CONDA_PREFIX') or sys.prefix
    matches = sorted(glob.glob(
        os.path.join(prefix, 'share', 'gz', 'gz-sim*', 'gui', 'gui.config')))
    return matches[-1] if matches else None


def _gui_config_with_camera(camera_pose):
    """A copy of the stock GUI config with the start-up camera moved.

    Gazebo's default view sits 6 m back and 6 m up, which is useless for
    something like Assignment 2's 1 x 1 m room - the arena is a speck and
    all you see is a small robot in the distance. The camera pose lives
    inside MinimalScene in the GUI config, and a world's own <gui> element
    replaces that config wholesale rather than merging into it, so setting
    the pose by hand would mean pasting Gazebo's ~260-line default plugin
    list into every world file.

    Instead: take whatever gui.config the installed Gazebo ships, swap the
    one <camera_pose> line, and hand the result to `gz sim -g --gui-config`.
    The whole stock UI is kept, nothing is vendored, and it follows the
    installed Gazebo if that config ever changes. The rewritten file goes to
    a temp path named after its contents, so repeated launches reuse one
    file per pose instead of piling up.
    """
    stock = _stock_gui_config()
    if not stock or not camera_pose:
        return None
    try:
        with open(stock, 'r') as f:
            config = f.read()
    except OSError:
        return None

    config, count = re.subn(r'<camera_pose>[^<]*</camera_pose>',
                            f'<camera_pose>{camera_pose}</camera_pose>',
                            config)
    if not count:
        return None

    name = 'wasp_as_gazebo_gui_%s.config' % hashlib.sha1(
        config.encode()).hexdigest()[:12]
    path = os.path.join(tempfile.gettempdir(), name)
    if not os.path.exists(path):
        try:
            with open(path, 'w') as f:
                f.write(config)
        except OSError:
            return None
    return path


def gz_gui(gui, camera_pose=None, verbosity=2, delay=3.0):
    """The Gazebo 3D window, as a list of launch actions (empty if disabled).

    camera_pose is an SDF pose string, "x y z roll pitch yaw", for where the
    view starts. A pitch near +pi/2 looks straight down; just under it
    (1.57) avoids sitting exactly on the orbit controller's gimbal.

    Delayed: `gz sim -g` looks for a running server to attach to, so it is
    given a moment to come up first.

    Deliberately no on_exit shutdown - closing the 3D window leaves RViz,
    rqt and the exercise nodes running, which is usually what you want,
    since the window is the one part of this that is purely decorative.
    """
    if not gui_enabled(gui):
        return []

    cmd = ['gz', 'sim', '-g', f'-v{verbosity}']
    config = _gui_config_with_camera(camera_pose)
    if config:
        cmd += ['--gui-config', config]

    return [TimerAction(period=delay, actions=[
        ExecuteProcess(cmd=cmd, output='screen'),
    ])]


def gz_unpause_when_gui_ready(world_name, gui, timeout=60.0):
    """Unpause the world once the 3D window is actually showing it.

    For a world where something starts moving on its own, loading unpaused
    means the interesting part is over before anyone can see it: Assignment
    2's Turtlebot reaches a wall about 1.6 s in, while the Gazebo window
    takes several seconds to appear. So that world loads paused and this
    releases it.

    The trigger is the GUI publishing /gui/camera/pose, which only exists
    once its 3D scene has been created - a real readiness signal rather than
    a guessed delay, which matters on a slow WSL2 machine where any fixed
    delay would be wrong in one direction or the other. Nothing advances
    while waiting, so being early or late costs nothing but the wait itself.

    Gives up after `timeout` and unpauses anyway, so a headed run whose
    window never appears still proceeds rather than hanging forever.

    Returns [] when the GUI is disabled - with no window to wait for, that
    case starts unpaused via gz_args(run=True) instead.
    """
    if not gui_enabled(gui):
        return []

    script = (
        'i=0; '
        f'while [ "$i" -lt {int(timeout / 0.5)} ]; do '
        '  gz topic -l 2>/dev/null | grep -q "^/gui/camera/pose$" && break; '
        '  i=$((i+1)); sleep 0.5; '
        'done; '
        f'gz service -s /world/{world_name}/control '
        '--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
        '--timeout 5000 --req "pause: false"'
    )
    return [ExecuteProcess(cmd=['sh', '-c', script], output='screen')]

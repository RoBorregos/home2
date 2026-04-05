#!/usr/bin/env python3
"""
Navigation launcher with switchable log views.
Press 0-5 to switch views, q to quit.

  0 = nav_central only (default)
  1 = nav2 container
  2 = rtabmap container
  3 = hardware (dashgo, lidar, ekf)
  4 = all logs
  5 = launch messages only
"""

import subprocess
import sys
import os
import select
import termios
import tty
import threading
import re
from collections import deque

VIEWS = {
    '0': ('nav_central', ['nav_central.py']),
    '1': ('nav2', ['nav2_container', 'controller_server', 'planner_server', 'smoother_server',
                    'behavior_server', 'bt_navigator', 'velocity_smoother', 'lifecycle_manager',
                    'local_costmap', 'global_costmap', 'MPPIController']),
    '2': ('rtabmap', ['rtabmap_container', 'rtabmap', 'rgbd_sync']),
    '3': ('hardware', ['dashgo', 'DashgoDriver', 'sllidar', 'ekf_node', 'PlayStation', 'joy']),
    '4': ('all', None),
    '5': ('launch', ['[INFO] [launch', 'process started', 'process died', 'ERROR', 'FATAL']),
}

HEADER = "\033[36m\033[1m[View {key}: {name}]\033[0m Press 0-5 to switch, q to quit"
CLEAR_LINE = "\033[2K\033[G"

current_view = '0'
buffer_size = 500
buffers = {k: deque(maxlen=buffer_size) for k in VIEWS}
buffers['raw'] = deque(maxlen=buffer_size * 3)


def matches_view(line, view_key):
    filters = VIEWS[view_key][1]
    if filters is None:  # 'all' view
        return True
    return any(f in line for f in filters)


def store_line(line):
    buffers['raw'].append(line)
    for key in VIEWS:
        if matches_view(line, key):
            buffers[key].append(line)


def print_buffer(view_key):
    os.write(sys.stdout.fileno(), b'\033[2J\033[H')  # clear screen
    header = HEADER.format(key=view_key, name=VIEWS[view_key][0])
    os.write(sys.stdout.fileno(), (header + '\n').encode())
    os.write(sys.stdout.fileno(), b'\033[36m' + b'-' * 60 + b'\033[0m\n')
    for line in buffers[view_key]:
        os.write(sys.stdout.fileno(), (line + '\n').encode())


def reader_thread(proc):
    while True:
        line = proc.stdout.readline()
        if not line:
            break
        line = line.rstrip('\n')
        store_line(line)
        if matches_view(line, current_view):
            os.write(sys.stdout.fileno(), (line + '\n').encode())


def main():
    global current_view

    args = ['ros2', 'launch', 'nav_main', 'general_navigation.launch.py'] + sys.argv[1:]

    proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        t = threading.Thread(target=reader_thread, args=(proc,), daemon=True)
        t.start()

        header = HEADER.format(key='0', name=VIEWS['0'][0])
        os.write(sys.stdout.fileno(), (header + '\n').encode())
        os.write(sys.stdout.fileno(), b'\033[36m' + b'-' * 60 + b'\033[0m\n')

        while proc.poll() is None:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 'q':
                    proc.terminate()
                    break
                elif key in VIEWS:
                    current_view = key
                    print_buffer(current_view)

    except KeyboardInterrupt:
        proc.terminate()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        proc.wait()


if __name__ == '__main__':
    main()

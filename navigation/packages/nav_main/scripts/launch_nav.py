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
import glob
import time
from collections import deque

VIEWS = {
    '0': 'nav_central',
    '1': 'nav2',
    '2': 'rtabmap',
    '3': 'hardware',
    '4': 'all',
    '5': 'launch',
}

# Which log file patterns belong to which view
LOG_FILE_VIEWS = {
    'nav2': ['component_container_mt-*nav2*', 'component_container_mt-7-*'],
    'rtabmap': ['component_container_mt-*rtabmap*', 'component_container_mt-6-*'],
    'hardware': ['dashgo*', 'sllidar*', 'ekf*', 'component_container-5-*'],
}

# Which stdout patterns belong to which view
STDOUT_VIEWS = {
    'nav_central': ['nav_central.py'],
    'nav2': ['nav2_container', 'controller_server', 'planner_server', 'smoother_server',
             'behavior_server', 'bt_navigator', 'velocity_smoother', 'lifecycle_manager',
             'local_costmap', 'global_costmap', 'MPPIController'],
    'rtabmap': ['rtabmap_container', 'rtabmap', 'rgbd_sync'],
    'hardware': ['dashgo', 'DashgoDriver', 'sllidar', 'ekf_node', 'PlayStation', 'joy'],
    'launch': ['[INFO] [launch', 'process started', 'process died', 'ERROR', 'FATAL'],
}

HEADER = "\033[36m\033[1m[View {key}: {name}]\033[0m Press 0-5 to switch, q to quit"

current_view = '0'
buffer_size = 500
buffers = {k: deque(maxlen=buffer_size) for k in VIEWS}
lock = threading.Lock()


def line_matches_view(line, view_name):
    if view_name == 'all':
        return True
    filters = STDOUT_VIEWS.get(view_name, [])
    return any(f in line for f in filters)


def store_and_print(line, source_view=None):
    """Store line in matching buffers and print if current view matches."""
    with lock:
        # Store in 'all'
        buffers['4'].append(line)

        matched = False
        for key, name in VIEWS.items():
            if name == 'all':
                continue
            if source_view and name == source_view:
                buffers[key].append(line)
                matched = True
            elif not source_view and line_matches_view(line, name):
                buffers[key].append(line)
                matched = True

        if not matched and not source_view:
            # Unmatched launch output goes to launch view
            buffers['5'].append(line)

        # Print if matches current view
        cur_name = VIEWS[current_view]
        should_print = False
        if cur_name == 'all':
            should_print = True
        elif source_view and cur_name == source_view:
            should_print = True
        elif not source_view and line_matches_view(line, cur_name):
            should_print = True

        if should_print:
            try:
                os.write(sys.stdout.fileno(), (line + '\n').encode())
            except OSError:
                pass


def print_buffer(view_key):
    """Clear screen and print buffer for the given view."""
    try:
        out = sys.stdout.fileno()
        os.write(out, b'\033[2J\033[H')
        header = HEADER.format(key=view_key, name=VIEWS[view_key])
        os.write(out, (header + '\n').encode())
        os.write(out, b'\033[36m' + b'-' * 60 + b'\033[0m\n')
        with lock:
            for line in buffers[view_key]:
                os.write(out, (line + '\n').encode())
    except OSError:
        pass


def launch_reader(proc):
    """Read launch process stdout."""
    while True:
        line = proc.stdout.readline()
        if not line:
            break
        store_and_print(line.rstrip('\n'))


def tail_file(filepath, view_name):
    """Tail a log file and feed lines to the appropriate view."""
    try:
        with open(filepath, 'r') as f:
            f.seek(0, 2)  # seek to end
            while True:
                line = f.readline()
                if line:
                    store_and_print(line.rstrip('\n'), source_view=view_name)
                else:
                    time.sleep(0.1)
    except (FileNotFoundError, OSError):
        pass


def start_log_tailers(log_dir):
    """Find and tail log files for each view."""
    # Wait for log directory to exist
    for _ in range(30):
        if os.path.isdir(log_dir):
            break
        time.sleep(0.5)
    else:
        return

    # Wait a bit for log files to be created
    time.sleep(3)

    started = set()
    for view_name, patterns in LOG_FILE_VIEWS.items():
        for pattern in patterns:
            for filepath in glob.glob(os.path.join(log_dir, pattern)):
                if filepath not in started and 'stderr' in filepath:
                    started.add(filepath)
                    t = threading.Thread(target=tail_file, args=(filepath, view_name), daemon=True)
                    t.start()

    # Also tail any remaining stderr logs for the 'all' view
    time.sleep(1)
    for filepath in glob.glob(os.path.join(log_dir, '*stderr.log')):
        if filepath not in started:
            started.add(filepath)
            t = threading.Thread(target=tail_file, args=(filepath, 'all'), daemon=True)
            t.start()


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

        # Start reading launch stdout
        t = threading.Thread(target=launch_reader, args=(proc,), daemon=True)
        t.start()

        # Print initial header
        print_buffer('0')

        # Find log directory from launch output — wait for it
        log_dir = os.path.expanduser('~/.ros/log/latest')
        log_tailer = threading.Thread(target=start_log_tailers, args=(log_dir,), daemon=True)
        log_tailer.start()

        while proc.poll() is None:
            if select.select([sys.stdin], [], [], 0.2)[0]:
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

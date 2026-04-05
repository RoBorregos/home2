#!/usr/bin/env python3
"""
Navigation launcher with switchable log views.

Interactive mode (tty): Press 0-5 to switch views, q to quit.
Non-interactive mode (docker compose): prints nav_central to stdout,
  writes all views to /tmp/nav_views/ — use 'tail -f /tmp/nav_views/nav2.log'

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

VIEW_LOG_DIR = '/tmp/nav_views'

# Keywords in log filenames to classify into views
LOG_FILE_KEYWORDS = {
    'nav2': ['nav2_container'],
    'rtabmap': ['rtabmap_container', 'rgbd_sync'],
    'hardware': ['dashgo', 'sllidar', 'ekf', 'joy_container'],
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
interactive = False
buffer_size = 500
buffers = {k: deque(maxlen=buffer_size) for k in VIEWS}
lock = threading.Lock()
log_dir_found = threading.Event()
log_dir_path = [None]
view_files = {}


def init_view_logs():
    """Create per-view log files in /tmp/nav_views/."""
    os.makedirs(VIEW_LOG_DIR, exist_ok=True)
    for name in VIEWS.values():
        path = os.path.join(VIEW_LOG_DIR, f'{name}.log')
        view_files[name] = open(path, 'w')


def write_view_log(view_name, line):
    """Append a line to the view's log file."""
    f = view_files.get(view_name)
    if f:
        f.write(line + '\n')
        f.flush()


def line_matches_view(line, view_name):
    if view_name == 'all':
        return True
    filters = STDOUT_VIEWS.get(view_name, [])
    return any(f in line for f in filters)


def store_and_print(line, source_view=None):
    """Store line in matching buffers, write to view files, and print if current view matches."""
    with lock:
        # Store in 'all'
        buffers['4'].append(line)
        write_view_log('all', line)

        matched = False
        for key, name in VIEWS.items():
            if name == 'all':
                continue
            if source_view and name == source_view:
                buffers[key].append(line)
                write_view_log(name, line)
                matched = True
            elif not source_view and line_matches_view(line, name):
                buffers[key].append(line)
                write_view_log(name, line)
                matched = True

        if not matched and not source_view:
            # Unmatched launch output goes to launch view
            buffers['5'].append(line)
            write_view_log('launch', line)

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
        line = line.rstrip('\n')
        # Detect log directory from launch output
        if 'All log files can be found below' in line and not log_dir_found.is_set():
            parts = line.split('below ')
            if len(parts) > 1:
                log_dir_path[0] = parts[-1].strip()
                log_dir_found.set()
        store_and_print(line)


def tail_file(filepath, view_name):
    """Tail a log file and feed lines to the appropriate view."""
    try:
        with open(filepath, 'r') as f:
            # Read from beginning to catch existing content
            while True:
                line = f.readline()
                if line:
                    stripped = line.rstrip('\n')
                    if stripped:
                        store_and_print(stripped, source_view=view_name)
                else:
                    time.sleep(0.1)
    except (FileNotFoundError, OSError):
        pass


def classify_log_file(filepath):
    """Determine which view a log file belongs to based on its filename."""
    basename = os.path.basename(filepath).lower()
    for view_name, keywords in LOG_FILE_KEYWORDS.items():
        for kw in keywords:
            if kw.lower() in basename:
                return view_name
    return None


def start_log_tailers():
    """Find and tail log files for each view."""
    # Wait for log directory to be detected from launch output
    log_dir_found.wait(timeout=30)
    log_dir = log_dir_path[0]
    if not log_dir:
        return

    started = set()
    # Rescan multiple times — nav2 nodes appear late due to TimerAction delay
    for _ in range(4):
        time.sleep(3)
        # Only tail stderr logs to avoid duplicates (stdout/stderr often overlap)
        for filepath in glob.glob(os.path.join(log_dir, '*stderr.log')):
            if filepath in started:
                continue
            started.add(filepath)
            view = classify_log_file(filepath)
            # Classified files: route all lines to that view
            # Unclassified files (e.g. component_container_mt): use content matching
            t = threading.Thread(target=tail_file, args=(filepath, view), daemon=True)
            t.start()


def main():
    global current_view, interactive

    init_view_logs()

    args = ['ros2', 'launch', 'nav_main', 'general_navigation.launch.py'] + sys.argv[1:]

    proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    interactive = sys.stdin.isatty()
    old_settings = None

    try:
        if interactive:
            import termios
            import tty
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        # Start reading launch stdout
        t = threading.Thread(target=launch_reader, args=(proc,), daemon=True)
        t.start()

        if interactive:
            print_buffer('0')

        # Start log file tailers — they wait for log dir to be detected
        log_tailer = threading.Thread(target=start_log_tailers, daemon=True)
        log_tailer.start()

        while proc.poll() is None:
            if interactive and select.select([sys.stdin], [], [], 0.2)[0]:
                key = sys.stdin.read(1)
                if key == 'q':
                    proc.terminate()
                    break
                elif key in VIEWS:
                    current_view = key
                    print_buffer(current_view)
            else:
                time.sleep(0.2)

    except KeyboardInterrupt:
        proc.terminate()
    finally:
        if interactive and old_settings is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        for f in view_files.values():
            f.close()
        proc.wait()


if __name__ == '__main__':
    main()

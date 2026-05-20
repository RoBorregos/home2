#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frida_interfaces.msg import MonitorReport, NodeStatus
import psutil
import os

try:
    from pynvml import *
    HAS_PYNVML = True
except ImportError:
    HAS_PYNVML = False

try:
    from jtop import jtop
    HAS_JTOP = True
except ImportError:
    HAS_JTOP = False

# Sysfs GPU load paths to try (value is 0-1000, divide by 10 for %)
GPU_SYSFS_PATHS = [
    '/sys/devices/platform/gpu.0/load',
    '/sys/devices/gpu.0/load',
    '/sys/devices/platform/bus@0/17000000.gpu/load',
]

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')

        self.declare_parameter('nodes_to_monitor', [
            'amcl',
            'bt_navigator',
            'controller_server',
            'planner_server',
            'map_server',
            'dashgo_driver',
            'rplidar_node'
        ])
        self.declare_parameter('update_period', 2.0)

        self.nodes_to_monitor = self.get_parameter('nodes_to_monitor').value
        self.update_period = self.get_parameter('update_period').value

        self.node_procs = {}
        self.newly_attached = set()  # skip CPU reading on first tick after attach

        # GPU initialization
        self.gpu_initialized = False
        self.is_jetson = os.path.exists('/sys/module/tegra_fuse') or os.path.exists('/proc/device-tree/model')

        # Find available sysfs GPU load path
        self.gpu_sysfs_path = None
        for path in GPU_SYSFS_PATHS:
            if os.path.exists(path):
                self.gpu_sysfs_path = path
                break

        if HAS_PYNVML and not self.is_jetson:
            try:
                nvmlInit()
                self.gpu_initialized = True
            except Exception:
                pass

        self.jtop_controller = None
        if self.is_jetson and HAS_JTOP:
            try:
                self.jtop_controller = jtop()
                self.jtop_controller.start()
                self.get_logger().info("jtop initialized for GPU monitoring")
            except Exception as e:
                self.get_logger().warning(f"Failed to start jtop: {e}")

        self.publisher = self.create_publisher(MonitorReport, 'system/node_monitor', 10)
        self.timer = self.create_timer(self.update_period, self.timer_callback)
        self.get_logger().info(
            f"Node Monitor started (Platform: {'Jetson' if self.is_jetson else 'PC'}). "
            f"GPU sysfs: {self.gpu_sysfs_path}. "
            f"Monitoring: {self.nodes_to_monitor}"
        )

    def find_pid(self, node_name):
        is_namespaced = node_name.startswith('/')
        if is_namespaced:
            parts = node_name.rsplit('/', 1)
            ns = parts[0] if parts[0] else '/'
            base_name = parts[1]

        candidates = []

        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = proc.info['cmdline']
                if not cmdline:
                    continue
                cmd_str = ' '.join(cmdline)

                if is_namespaced:
                    if f"__node:={base_name}" in cmd_str and f"__ns:={ns}" in cmd_str:
                        return proc.info['pid']
                else:
                    if f"__node:={node_name}" in cmd_str:
                        return proc.info['pid']
                    if node_name in cmd_str:
                        candidates.append((proc.info['pid'], cmdline))
                        continue

                if proc.info['name'] and (
                    proc.info['name'] == node_name or node_name in proc.info['name']
                ):
                    candidates.append((proc.info['pid'], cmdline))

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0][0]

        # Multiple matches: prefer the process where node_name is the direct script
        # (e.g. python3 /path/to/node_name.py) over a ros2 run launcher wrapper.
        for pid, cmdline in candidates:
            if len(cmdline) >= 2 and node_name in cmdline[1]:
                return pid

        return candidates[0][0]

    def get_gpu_load(self):
        """Returns overall GPU load % from the best available source."""
        # jtop (Jetson, if available in this environment)
        if self.jtop_controller and self.jtop_controller.ok():
            try:
                gpu_data = self.jtop_controller.gpu
                # jtop 4.x: {'gpu': {'status': {'load': float}, ...}}
                if isinstance(gpu_data, dict):
                    gpu_info = gpu_data.get('gpu', {})
                    if isinstance(gpu_info, dict):
                        load = gpu_info.get('status', {}).get('load')
                        if load is not None:
                            return float(load)
            except Exception:
                pass

        # Jetson sysfs (works inside containers without jtop)
        if self.gpu_sysfs_path:
            try:
                with open(self.gpu_sysfs_path, 'r') as f:
                    return float(f.read().strip()) / 10.0
            except Exception:
                pass

        # Discrete NVIDIA GPU via pynvml
        if self.gpu_initialized:
            try:
                handle = nvmlDeviceGetHandleByIndex(0)
                rates = nvmlDeviceGetUtilizationRates(handle)
                return float(rates.gpu)
            except Exception:
                pass

        return 0.0

    def process_uses_gpu(self, pid):
        """
        Returns True if the process (or any of its children) has nvhost GPU fds open.
        On Jetson the GPU is accessed via anon_inode:nvhost-* file descriptors.
        """
        pids = [pid]
        try:
            pids += [c.pid for c in psutil.Process(pid).children(recursive=True)]
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
        for p in pids:
            try:
                fd_dir = f'/proc/{p}/fd'
                for fd in os.listdir(fd_dir):
                    try:
                        if 'nvhost' in os.readlink(f'{fd_dir}/{fd}'):
                            return True
                    except OSError:
                        continue
            except OSError:
                continue
        return False

    def timer_callback(self):
        self.nodes_to_monitor = self.get_parameter('nodes_to_monitor').value
        report = MonitorReport()
        active_node_found = False

        # Read GPU load once per tick; assign only to processes that actually use the GPU
        gpu_load = self.get_gpu_load()

        for name in self.nodes_to_monitor:
            status = NodeStatus()
            status.name = name

            proc = self.node_procs.get(name)
            if proc is None or not proc.is_running():
                pid = self.find_pid(name)
                if pid:
                    try:
                        proc = psutil.Process(pid)
                        proc.cpu_percent()  # establish baseline; reading discarded
                        self.node_procs[name] = proc
                        self.newly_attached.add(name)
                        self.get_logger().info(f'Monitor attached to {name} (PID: {pid})')
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        proc = None

            if proc:
                active_node_found = True
                try:
                    if name in self.newly_attached:
                        # First tick after attach: no valid interval yet, skip CPU
                        self.newly_attached.discard(name)
                        status.cpu_usage = 0.0
                    else:
                        status.cpu_usage = proc.cpu_percent()

                    status.memory_usage = proc.memory_percent()
                    status.gpu_usage = gpu_load if self.process_uses_gpu(proc.pid) else 0.0
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    status.cpu_usage = 0.0
                    status.memory_usage = 0.0
                    status.gpu_usage = 0.0
                    self.node_procs.pop(name, None)
            else:
                status.cpu_usage = 0.0
                status.memory_usage = 0.0
                status.gpu_usage = 0.0

            report.nodes.append(status)

        if active_node_found:
            self.publisher.publish(report)


def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gpu_initialized:
            try:
                nvmlShutdown()
            except Exception:
                pass
        if node.jtop_controller:
            try:
                node.jtop_controller.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frida_interfaces.msg import MonitorReport, NodeStatus
import psutil
import os
import subprocess

try:
    import pynvml
    HAS_PYNVML = True
except ImportError:
    HAS_PYNVML = False

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')
        
        # Declare parameters
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
        
        # PID cache
        self.node_pids = {} # node_name -> pid
        
        # GPU Initialization
        self.gpu_initialized = False
        if HAS_PYNVML:
            try:
                pynvml.nvmlInit()
                self.gpu_initialized = True
                self.get_logger().info("NVML initialized successfully for GPU monitoring.")
            except Exception as e:
                self.get_logger().warning(f"Failed to initialize NVML: {e}")
        
        # Publisher
        self.publisher = self.create_publisher(MonitorReport, 'system/node_monitor', 10)
        
        # Timer
        self.timer = self.create_timer(self.update_period, self.timer_callback)
        
        self.get_logger().info(f"Node Monitor started. Monitoring: {self.nodes_to_monitor}")

    def find_pid(self, node_name):
        """Tries to find the PID for a given ROS2 node name."""
        # Try finding in current processes
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # 1. Check if node name is explicitly set in ROS2 args
                cmdline = proc.info['cmdline']
                if cmdline:
                    cmd_str = ' '.join(cmdline)
                    if f"__node:={node_name}" in cmd_str:
                        return proc.info['pid']
                
                # 2. Check if process name matches exactly (heuristic for simple nodes)
                if proc.info['name'] == node_name:
                    return proc.info['pid']
                
                # 3. Check if node name is in the path/executable (heuristic)
                # This could be risky if multiple nodes have similar names
                # but good for uniquely named executables
                if node_name in proc.info['name']:
                    # Double check it looks like a ROS node or matches well
                    return proc.info['pid']

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        return None

    def get_gpu_process_info(self, pid):
        """Gets GPU memory usage or utilization for a specific PID."""
        if not self.gpu_initialized:
            return 0.0
        
        try:
            device_count = pynvml.nvmlDeviceGetCount()
            total_gpu_mem_pct = 0.0
            for i in range(device_count):
                handle = pynvml.nvmlDeviceGetHandleByIndex(i)
                # Get processes using this GPU
                procs = pynvml.nvmlDeviceGetComputeRunningProcesses(handle)
                for p in procs:
                    if p.pid == pid:
                        # Memory used by process in bytes
                        mem = p.usedGpuMemory
                        # Total device memory
                        info = pynvml.nvmlDeviceGetMemoryInfo(handle)
                        total_gpu_mem_pct += (mem / info.total) * 100.0
            return total_gpu_mem_pct
        except Exception:
            return 0.0

    def timer_callback(self):
        # Refresh nodes list from param in case it changed
        self.nodes_to_monitor = self.get_parameter('nodes_to_monitor').value
        
        report = MonitorReport()
        
        for name in self.nodes_to_monitor:
            status = NodeStatus()
            status.name = name
            
            # Find or refresh PID
            pid = self.node_pids.get(name)
            if pid is None or not psutil.pid_exists(pid):
                pid = self.find_pid(name)
                if pid:
                    self.node_pids[name] = pid
            
            if pid:
                try:
                    p = psutil.Process(pid)
                    # CPU percentage (interval=None means since last call)
                    status.cpu_usage = p.cpu_percent()
                    # Memory percentage (RSS)
                    status.memory_usage = p.memory_percent()
                    # GPU percentage
                    status.gpu_usage = self.get_gpu_process_info(pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    status.cpu_usage = 0.0
                    status.memory_usage = 0.0
                    status.gpu_usage = 0.0
                    if name in self.node_pids:
                        del self.node_pids[name]
            else:
                status.cpu_usage = 0.0
                status.memory_usage = 0.0
                status.gpu_usage = 0.0
            
            report.nodes.append(status)
            
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
                pynvml.nvmlShutdown()
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

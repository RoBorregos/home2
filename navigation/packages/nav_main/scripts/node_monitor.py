#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frida_interfaces.msg import MonitorReport, NodeStatus
import psutil
import os
import subprocess

try:
    from pynvml import *
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
        
        self.node_procs = {} 
        
        # GPU Initialization
        self.gpu_initialized = False
        if HAS_PYNVML:
            try:
                nvmlInit()
                self.gpu_initialized = True
            except NVMLError as e:
                self.get_logger().warning(f"Failed to initialize NVML: {e}")
            except Exception as e:
                self.get_logger().warning(f"Unexpected error initializing NVML: {e}")
        
        self.publisher = self.create_publisher(MonitorReport, 'system/node_monitor', 10)
        
        self.timer = self.create_timer(self.update_period, self.timer_callback)
        
        self.get_logger().info(f"Node Monitor started. Monitoring: {self.nodes_to_monitor}")

    def find_pid(self, node_name):
        is_namespaced = node_name.startswith('/')
        if is_namespaced:
            parts = node_name.rsplit('/', 1)
            ns = parts[0] if parts[0] else '/'
            base_name = parts[1]

        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = proc.info['cmdline']
                if cmdline:
                    cmd_str = ' '.join(cmdline)
                    
                    if is_namespaced:
                        if f"__node:={base_name}" in cmd_str and f"__ns:={ns}" in cmd_str:
                            return proc.info['pid']
                    else:
                        if f"__node:={node_name}" in cmd_str:
                            return proc.info['pid']
                        if node_name in cmd_str:
                            return proc.info['pid']
                
                if proc.info['name'] and proc.info['name'] == node_name:
                    return proc.info['pid']
                
                if proc.info['name'] and node_name in proc.info['name']:
                    return proc.info['pid']

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        return None

    def get_gpu_process_info(self, pid):
        if not self.gpu_initialized:
            return 0.0
        
        try:
            device_count = nvmlDeviceGetCount()
            total_gpu_mem_pct = 0.0
            for i in range(device_count):
                handle = nvmlDeviceGetHandleByIndex(i)
                procs = nvmlDeviceGetComputeRunningProcesses(handle)
                for p in procs:
                    if p.pid == pid:
                        mem = p.usedGpuMemory
                        info = nvmlDeviceGetMemoryInfo(handle)
                        total_gpu_mem_pct += (mem / info.total) * 100.0
            return total_gpu_mem_pct
        except Exception:
            return 0.0

    def timer_callback(self):
        self.nodes_to_monitor = self.get_parameter('nodes_to_monitor').value
        
        report = MonitorReport()
        
        active_node_found = False
        for name in self.nodes_to_monitor:
            status = NodeStatus()
            status.name = name
            
 
            proc = self.node_procs.get(name)
            if proc is None or not proc.is_running():
                pid = self.find_pid(name)
                if pid:
                    try:
                        proc = psutil.Process(pid)
                        proc.cpu_percent()
                        self.node_procs[name] = proc
                        self.get_logger().info(f'Monitor attached to {name} (PID: {pid})')
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        proc = None
            
            if proc:
                active_node_found = True
                try:

                    status.cpu_usage = proc.cpu_percent()
                    status.memory_usage = proc.memory_percent()
                    status.gpu_usage = self.get_gpu_process_info(proc.pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    status.cpu_usage = 0.0
                    status.memory_usage = 0.0
                    status.gpu_usage = 0.0
                    if name in self.node_procs:
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
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

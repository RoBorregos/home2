# CycloneDDS Setup

Script to configure CycloneDDS as the ROS 2 DDS middleware with optimized settings for ZED camera and large data topics (point clouds, images).

## Why CycloneDDS?

- Better performance with large messages (point clouds, images)
- Lower latency for real-time navigation
- More predictable behavior under high throughput

## Script Location

```
home2/setup_cyclonedds.sh
```

## Usage

### Bare Metal (Orin, direct install)

Single command does everything — XML config, kernel tuning, and env vars:

```bash
sudo bash setup_cyclonedds.sh eno1
source ~/.bashrc
```

Replace `eno1` with your network interface. Omit it to auto-detect:

```bash
sudo bash setup_cyclonedds.sh
```

### Docker Setup

Docker containers share the host kernel, so kernel buffer settings must be applied on the **host** first.

**Step 1 — On the host PC (run once):**

```bash
sudo bash setup_cyclonedds.sh --host-only
```

**Step 2 — Inside the Docker container:**

```bash
sudo bash setup_cyclonedds.sh --docker eno1
source ~/.bashrc
```

### Revert to FastDDS

```bash
sudo bash setup_cyclonedds.sh --revert
source ~/.bashrc
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
```

This removes the XML config, kernel tuning, and env vars from `~/.bashrc`.

## What It Configures

### CycloneDDS XML (`/etc/cyclonedds.xml`)

| Setting | Value | Purpose |
|---|---|---|
| `MaxMessageSize` | 65500B | Larger UDP datagrams for point clouds |
| `SocketReceiveBufferSize` | 10MB | Large receive buffer |
| `WhcHigh` | 500kB | Writer history cache watermark |
| `AllowMulticast` | true | Multicast discovery |
| `EnableMulticastLoopback` | true | Same-machine communication |

### Kernel Tuning (`/etc/sysctl.d/60-cyclonedds-buffers.conf`)

| Setting | Value | Default | Purpose |
|---|---|---|---|
| `net.core.rmem_max` | 2 GiB | 4 MB | Max kernel receive buffer |
| `net.ipv4.ipfrag_time` | 3s | 30s | Faster stale fragment cleanup |
| `net.ipv4.ipfrag_high_thresh` | 128 MB | 4 MB | Fragment reassembly memory |

### Environment Variables (added to `~/.bashrc`)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
```

## Finding Your Network Interface

```bash
ip -br link show
```

Common interfaces:
- `eno1`, `eth0` — Wired ethernet
- `wlan0`, `wlp2s0` — WiFi
- `lo` — Loopback (same machine only)

## Reference

Based on [Stereolabs DDS and Network Tuning](https://www.stereolabs.com/docs/ros2/dds-and-network-tuning) recommendations.

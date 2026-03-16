# CycloneDDS Setup

Optimized CycloneDDS configuration for ROS 2 with ZED camera and large data topics.

## Architecture

```
Host PC                              Docker Container
┌─────────────────────┐              ┌──────────────────────────┐
│ setup_cyclonedds.sh  │              │ Dockerfile (per area)    │
│  --host-only eno1    │              │  installs rmw-cyclonedds │
│                      │              │  COPY cyclonedds_setup.sh│
│ Saves:               │              │  ENV RMW_IMPLEMENTATION  │
│  /etc/cyclonedds.env │              │  ENV CYCLONEDDS_URI      │
│  (CYCLONE_INTERFACE) │              └──────────────────────────┘
│  sysctl buffers      │                          │
└──────────┬───────────┘              ┌───────────▼──────────────┐
           │                          │ run.sh                   │
           │  reads env               │  reads /etc/cyclonedds.env│
           └─────────────────────────►│  writes to .env          │
                                      │  CYCLONE_INTERFACE=eno1  │
                                      └───────────┬─────────────┘
                                                  │
                                      ┌───────────▼─────────────┐
                                      │ docker-compose.yaml      │
                                      │  environment:            │
                                      │   CYCLONE_INTERFACE: ... │
                                      └───────────┬─────────────┘
                                                  │
                                      ┌───────────▼─────────────┐
                                      │ Container startup        │
                                      │  .bashrc sources         │
                                      │  cyclonedds_setup.sh     │
                                      │  → generates XML with    │
                                      │    correct interface      │
                                      └──────────────────────────┘
```

## Files

| File | Purpose |
|---|---|
| `setup_cyclonedds.sh` | Host setup script (sysctl + env file) |
| `docker/cyclonedds_setup.sh` | Container script (generates XML at startup) |
| `docker/<area>/Dockerfile.*` | Each area installs CycloneDDS |
| `docker/<area>/docker-compose*.yaml` | Passes `CYCLONE_INTERFACE` to container |
| `docker/<area>/run.sh` | Reads `/etc/cyclonedds.env` from host |

## Usage

### Bare Metal (Orin, direct install)

```bash
sudo bash setup_cyclonedds.sh eno1
source ~/.bashrc
```

### Docker Setup (PC)

**Step 1 — Host (run once):**

```bash
sudo bash setup_cyclonedds.sh --host-only eno1
```

This applies kernel sysctl buffers and saves `CYCLONE_INTERFACE=eno1` to `/etc/cyclonedds.env`.

**Step 2 — Just run your area normally:**

```bash
bash run.sh navigation cpu
```

The `run.sh` automatically reads `/etc/cyclonedds.env`, passes `CYCLONE_INTERFACE` through docker-compose, and the container generates the correct XML at startup.

### Revert to FastDDS

```bash
sudo bash setup_cyclonedds.sh --revert
source ~/.bashrc
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
```

### Override Interface at Runtime

```bash
# Via environment variable (overrides /etc/cyclonedds.env)
CYCLONE_INTERFACE=wlan0 bash run.sh navigation cpu
```

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

## Finding Your Network Interface

```bash
# On host
ip -br link show

# Inside Docker
ls /sys/class/net/
```

Common interfaces: `eno1`, `eth0` (wired), `wlan0` (wifi), `lo` (loopback)

## Reference

Based on [Stereolabs DDS and Network Tuning](https://www.stereolabs.com/docs/ros2/dds-and-network-tuning).

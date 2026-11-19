# CycloneDDS Setup

Optimized CycloneDDS configuration for ROS 2 with optional iceoryx shared memory (SHM) for zero-copy transport.

## Architecture

```
Host PC                              Docker Container
┌─────────────────────────────┐       ┌──────────────────────────┐
│ scripts/setup_cyclonedds.sh │       │ Dockerfile (per area)    │
│  --host-only eno1           │       │  builds iceoryx v2.0.6   │
│                             │       │  builds CycloneDDS v0.10 │
│ Saves:                      │       │  COPY cyclonedds_setup.sh│
│  /etc/cyclonedds.env        │       │  ENV RMW_IMPLEMENTATION  │
│  (CYCLONE_INTERFACE)        │       │  ENV CYCLONEDDS_URI      │
│  sysctl buffers             │       └──────────────────────────┘
└──────────┬──────────────────┘                   │
           │                          ┌───────────▼───────────────┐
           │  reads env               │  run.sh                   │
           └─────────────────────────►│  reads /etc/cyclonedds.env│
                                      │  writes to .env           │
                                      │  CYCLONE_INTERFACE=eno1   │
                                      └───────────┬───────────────┘
                                                  │
                                      ┌───────────▼──────────────┐
                                      │ docker-compose.yaml      │
                                      │  environment:            │
                                      │   CYCLONE_INTERFACE: ... │
                                      │   CYCLONE_SHM: 0 or 1    │
                                      └───────────┬──────────────┘
                                                  │
                                      ┌───────────▼───────────────┐
                                      │ Container startup         │
                                      │  .bashrc sources          │
                                      │  cyclonedds_setup.sh      │
                                      │  → generates XML with     │
                                      │    correct interface      │
                                      │  → includes <SharedMemory>│
                                      │    section only if        │
                                      │    CYCLONE_SHM=1          │
                                      └───────────────────────────┘

   (When CYCLONE_SHM=1)
┌───────────────────────────┐
│ RouDi container           │
│  home2-roudi              │
│  iox-roudi daemon         │
│  manages /dev/shm pools   │
│  started by lib.sh        │
│  before zed/vision/nav    │
└───────────────────────────┘
```

## Shared Memory (SHM) vs UDP

CycloneDDS supports two transport modes:

|               | SHM (iceoryx)                            | UDP (default)                  |
| ------------- | ---------------------------------------- | ------------------------------ |
| **Transport** | Zero-copy via `/dev/shm`                 | UDP loopback / network         |
| **Requires**  | RouDi daemon + ~3.2 GB `/dev/shm`        | Nothing extra                  |
| **Best for**  | Orin AGX (64 GB RAM), large image topics | Local dev, low-memory machines |
| **Env var**   | `CYCLONE_SHM=1`                          | `CYCLONE_SHM=0` (default)      |

**Default is SHM off.** All docker-compose files read `CYCLONE_SHM` from the environment, defaulting to `0`. On resource-constrained machines (< 16 GB RAM), SHM will fail because RouDi needs ~3.2 GB of `/dev/shm` for the management segment alone.

### SHM Memory Breakdown (when enabled)

RouDi reserves shared memory in two segments:

**`iceoryx_mgmt` (~3 GB)** — management overhead driven by compile-time limits:

- `IOX_MAX_PUBLISHERS=20000`
- `IOX_MAX_SUBSCRIBERS=20000`
- `IOX_MAX_CHUNKS_HELD_PER_SUBSCRIBER_SIMULTANEOUSLY=2048`
- `IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS=1024`

**`ros` (~1 GB)** — data pools defined in `roudi_config.toml`:

| Pool size | Count   | Subtotal | Use case                              |
| --------- | ------- | -------- | ------------------------------------- |
| 128 B     | 100,000 | ~13 MB   | Parameter events, tf, small services  |
| 1 KB      | 20,000  | ~20 MB   | Sensor data, joint states, detections |
| 16 KB     | 500     | ~8 MB    | Camera info, compressed data          |
| 512 KB    | 50      | ~26 MB   | Depth maps, smaller frames            |
| 2 MB      | 60      | ~126 MB  | Full camera frames (960x540 BGR)      |
| 4 MB      | 80      | ~336 MB  | Depth / pointcloud (~3.5 MB)          |
| 8 MB      | 60      | ~503 MB  | HD1080 full resolution (~6 MB)        |

### ZED SHM Workarounds

When SHM is enabled, the ZED camera node requires special handling to avoid `TOO_MANY_CHUNKS_HELD_IN_PARALLEL` errors during its startup burst (30+ topics advertised in milliseconds):

- `zed_shm_override.yaml` — disables `start_parameter_event_publisher` for all nodes
- `zed_shm_launch.py` — patches the ZED launch file to inject `--ros-args -p start_parameter_event_publisher:=false` into the composable node container
- `robot_state_publisher` runs without SHM: `CYCLONEDDS_URI='file:///etc/cyclonedds.xml,<SharedMemory><Enable>false</Enable></SharedMemory>'`, so it keeps the same network interface as the rest of the container

## Files

| File                                 | Purpose                                                                                 |
| ------------------------------------ | --------------------------------------------------------------------------------------- |
| `scripts/setup_cyclonedds.sh`        | Host setup script (sysctl + env file + optional bare-metal install)                     |
| `docker/cyclonedds_setup.sh`         | Container script (generates XML at startup based on `CYCLONE_SHM`)                      |
| `docker/roudi/Dockerfile`            | Builds iceoryx v2.0.6 with expanded pub/sub limits                                      |
| `docker/roudi/docker-compose.yaml`   | RouDi container (mounts `/dev/shm`, `ipc: host`)                                        |
| `docker/roudi/run.sh`                | Starts/stops the RouDi container                                                        |
| `roudi_config.toml`                  | Iceoryx memory pool configuration                                                       |
| `docker/<area>/Dockerfile.*`         | Each area builds iceoryx + CycloneDDS with `-DENABLE_SHM=ON`                            |
| `docker/<area>/docker-compose*.yaml` | Passes `CYCLONE_INTERFACE` and `CYCLONE_SHM` to container                               |
| `docker/<area>/run.sh`               | Writes the interface from `resolve_cyclone_interface` (`lib.sh`) to `.env`              |
| `zed_shm_override.yaml`              | Disables parameter events for ZED (SHM workaround)                                      |
| `zed_shm_launch.py`                  | Patches ZED launch file for SHM compatibility                                           |
| `lib.sh`                             | `ensure_roudi()` starts RouDi; `run_area()` conditionally calls it when `CYCLONE_SHM=1`; `resolve_cyclone_interface()` picks the interface |

## Usage

### Local Development (no SHM, default)

No extra setup needed for shared memory. Just configure the network interface:

**Step 1 — Host (run once):**

```bash
sudo bash scripts/setup_cyclonedds.sh --host-only eno1
```

**Step 2 — Run your area normally:**

```bash
bash run.sh navigation cpu
```

CycloneDDS uses UDP loopback. No RouDi container is started.

### Orin AGX / High-Memory Machine (with SHM)

**Step 1 — Host (run once):**

```bash
sudo bash scripts/setup_cyclonedds.sh --host-only eno1
```

**Step 2 — Enable SHM and run:**

```bash
export CYCLONE_SHM=1
bash run.sh navigation l4t
```

This will:

1. Start the `home2-roudi` container automatically (via `lib.sh`)
2. Generate CycloneDDS XML with `<SharedMemory><Enable>true</Enable></SharedMemory>`
3. All inter-container communication for large topics uses zero-copy via `/dev/shm`

### Bare Metal (Orin, direct install)

```bash
sudo bash scripts/setup_cyclonedds.sh eno1
source ~/.bashrc
```

### Revert to FastDDS

```bash
sudo bash scripts/setup_cyclonedds.sh --revert
source ~/.bashrc
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
```

### Override Interface at Runtime

```bash
CYCLONE_INTERFACE=wlan0 bash run.sh navigation cpu
```

`resolve_cyclone_interface()` in `lib.sh` picks the interface for every area:

1. `CYCLONE_INTERFACE` from the environment, if set (an empty value means autodetermine).
2. Otherwise `CYCLONE_INTERFACE` from `/etc/cyclonedds.env`.
3. If that interface has no IPv4 address, `lo` instead, with a warning.

### Running Without the Router

The interface in `/etc/cyclonedds.env` (e.g. `end0` on the Orin) gets its address from the router's DHCP. Without the router it has no address, and CycloneDDS would abort every node with `does not match an available interface` / `failed to create domain`. `run.sh` then falls back to `lo` and prints:

```
[CycloneDDS] WARNING: end0 has no IPv4 address (router disconnected?). Using lo: only this machine will see ROS topics.
```

On `lo`, areas on the same machine (zed, vision, navigation, display) still talk to each other; other machines (e.g. hri on the second Orin, a laptop) cannot reach them. The interface is chosen when an area starts, so restart areas after connecting or disconnecting the router; areas on different interfaces don't see each other.

## What It Configures

### CycloneDDS XML (`/etc/cyclonedds.xml`)

| Setting                   | Value      | Purpose                               |
| ------------------------- | ---------- | ------------------------------------- |
| `MaxMessageSize`          | 65500B     | Larger UDP datagrams for point clouds |
| `SocketReceiveBufferSize` | 10MB       | Large receive buffer                  |
| `WhcHigh`                 | 500kB      | Writer history cache watermark        |
| `AllowMulticast`          | true       | Multicast discovery                   |
| `EnableMulticastLoopback` | true       | Same-machine communication            |
| `SharedMemory.Enable`     | true/false | Controlled by `CYCLONE_SHM` env var   |

### Kernel Tuning (`/etc/sysctl.d/60-cyclonedds-buffers.conf`)

| Setting                       | Value  | Default | Purpose                       |
| ----------------------------- | ------ | ------- | ----------------------------- |
| `net.core.rmem_max`           | 2 GiB  | 4 MB    | Max kernel receive buffer     |
| `net.ipv4.ipfrag_time`        | 3s     | 30s     | Faster stale fragment cleanup |
| `net.ipv4.ipfrag_high_thresh` | 128 MB | 4 MB    | Fragment reassembly memory    |

### Iceoryx / RouDi (when `CYCLONE_SHM=1`)

| Setting                                             | Value               | Purpose                                             |
| --------------------------------------------------- | ------------------- | --------------------------------------------------- |
| Iceoryx version                                     | v2.0.6              | Built from source in Dockerfiles                    |
| `IOX_MAX_PUBLISHERS`                                | 20000               | Compile-time limit (all areas use SHM)              |
| `IOX_MAX_SUBSCRIBERS`                               | 20000               | Compile-time limit                                  |
| `IOX_MAX_CHUNKS_HELD_PER_SUBSCRIBER_SIMULTANEOUSLY` | 2048                | Prevents chunk exhaustion                           |
| `IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS`              | 1024                | Patched from default 256                            |
| RouDi container                                     | `home2-roudi`       | Dedicated container, `ipc: host`, mounts `/dev/shm` |
| Config file                                         | `roudi_config.toml` | 6 memory pools (128 B to 8 MB)                      |

## Finding Your Network Interface

```bash
# On host
ip -br link show

# Inside Docker
ls /sys/class/net/
```

Common interfaces: `eno1`, `eth0` (wired), `wlan0` (wifi), `lo` (loopback)

## Troubleshooting

### RouDi SIGBUS / fails to start

RouDi needs ~3.2 GB in `/dev/shm`. Check available space:

```bash
df -h /dev/shm
```

If insufficient, either increase it (`sudo mount -o remount,size=4G /dev/shm`) or disable SHM by unsetting `CYCLONE_SHM` (defaults to `0`).

### Stale iceoryx artifacts

If RouDi crashes, stale lock files may prevent restart:

```bash
rm -f /tmp/iox-unique-roudi.lock /tmp/roudi.lock
rm -f /dev/shm/iceoryx*
```

The RouDi container command already cleans these on startup. Running `--down` with `CYCLONE_SHM=0` also cleans them via `lib.sh`.

## Reference

Based on [Stereolabs DDS and Network Tuning](https://www.stereolabs.com/docs/ros2/dds-and-network-tuning).

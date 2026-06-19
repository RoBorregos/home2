"""Python port of the host-side checks in status/check_infra.sh.

Kept minimal; the bash version remains authoritative for the one-shot
`scripts/status.sh`. If this list grows, consolidate to a single source."""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

CYCLONE_XML = Path("/etc/cyclonedds.xml")
CYCLONE_SYSCTL = Path("/etc/sysctl.d/60-cyclonedds-buffers.conf")
CYCLONE_ENV = Path("/etc/cyclonedds.env")
EXPECTED_RMEM_MAX = 2_147_483_647
JETSON_MARKER = Path("/etc/nv_tegra_release")


@dataclass
class DdsHealth:
    cyclone_xml: bool
    sysctl_conf: bool
    rmem_max: int
    rmw_impl: str
    cyclone_iface: str
    iceoryx_roudi_status: str  # "" if not expected, "running", or "missing"

    @property
    def ok(self) -> bool:
        return (
            self.cyclone_xml
            and self.sysctl_conf
            and self.rmem_max >= EXPECTED_RMEM_MAX
            and self.rmw_impl == "rmw_cyclonedds_cpp"
            and self.iceoryx_roudi_status != "missing"
        )


@dataclass
class ContainerStatus:
    name: str
    state: str  # "up", "unhealthy", "exited", "missing", "docker-missing"
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.state == "up"


@dataclass
class InfraSnapshot:
    env_type: str
    dds: DdsHealth
    containers: dict[str, ContainerStatus] = field(default_factory=dict)


def _sysctl_rmem() -> int:
    try:
        out = subprocess.run(
            ["sysctl", "-n", "net.core.rmem_max"],
            capture_output=True,
            text=True,
            timeout=1,
        ).stdout.strip()
        return int(out or 0)
    except (subprocess.SubprocessError, ValueError):
        return 0


def _cyclone_iface() -> str:
    if not CYCLONE_ENV.is_file():
        return ""
    for line in CYCLONE_ENV.read_text().splitlines():
        if line.startswith("CYCLONE_INTERFACE="):
            return line.split("=", 1)[1].strip() or "autodetermine"
    return ""


def detect_env_type() -> str:
    if JETSON_MARKER.is_file():
        return "l4t"
    if subprocess.run(["which", "nvidia-smi"], capture_output=True).returncode == 0:
        if subprocess.run(["nvidia-smi"], capture_output=True).returncode == 0:
            return "cuda"
    return "cpu"


def _shm_expected() -> bool:
    shm = os.environ.get("CYCLONE_SHM")
    if shm == "1":
        return True
    if shm is None and JETSON_MARKER.is_file():
        return True
    return False


def _container_state(name: str) -> ContainerStatus:
    try:
        out = subprocess.run(
            [
                "docker",
                "ps",
                "-a",
                "--filter",
                f"name=^{name}$",
                "--format",
                "{{.Status}}",
            ],
            capture_output=True,
            text=True,
            timeout=2,
        ).stdout.strip()
    except FileNotFoundError:
        return ContainerStatus(name, "docker-missing", "docker CLI not found")

    if not out:
        return ContainerStatus(name, "missing", "not created")
    if out.startswith("Up"):
        if "(unhealthy)" in out:
            return ContainerStatus(name, "unhealthy", out)
        return ContainerStatus(name, "up", out)
    return ContainerStatus(name, "exited", out)


def check_dds() -> DdsHealth:
    roudi = ""
    if _shm_expected():
        roudi_state = _container_state("home2-roudi")
        roudi = "running" if roudi_state.ok else "missing"
    return DdsHealth(
        cyclone_xml=CYCLONE_XML.is_file(),
        sysctl_conf=CYCLONE_SYSCTL.is_file(),
        rmem_max=_sysctl_rmem(),
        rmw_impl=os.environ.get("RMW_IMPLEMENTATION", ""),
        cyclone_iface=_cyclone_iface(),
        iceoryx_roudi_status=roudi,
    )


def check_containers(names: list[str]) -> dict[str, ContainerStatus]:
    return {n: _container_state(n) for n in names}


def snapshot(container_names: list[str]) -> InfraSnapshot:
    return InfraSnapshot(
        env_type=detect_env_type(),
        dds=check_dds(),
        containers=check_containers(container_names),
    )

#!/bin/bash
# Generates CycloneDDS config based on CYCLONE_INTERFACE env var.
# If CYCLONE_INTERFACE is set, uses that interface. Otherwise, autodetermine.
# If CYCLONE_SHM=1, enables iceoryx shared memory and starts RouDi.
# Called at build time (default XML) and sourced from .bashrc at runtime (override).

CYCLONE_XML="/etc/cyclonedds.xml"

# At runtime (sourced from .bashrc), regenerate if CYCLONE_INTERFACE is set
REGENERATE_XML=0
if [ -n "${CYCLONE_INTERFACE:-}" ]; then
    REGENERATE_XML=1
    IFACE_LINE="        <NetworkInterface name=\"$CYCLONE_INTERFACE\" priority=\"default\" multicast=\"true\" autodetermine=\"false\"/>"
else
    # If XML already exists and no override, skip regeneration
    if [ ! -f "$CYCLONE_XML" ]; then
        REGENERATE_XML=1
        IFACE_LINE='        <NetworkInterface autodetermine="true" priority="default" multicast="default" />'
    fi
fi

if [ "$REGENERATE_XML" = "1" ]; then
    # SHM section (only if CYCLONE_SHM=1)
    if [ "${CYCLONE_SHM:-}" = "1" ]; then
        SHM_SECTION='    <SharedMemory>
          <Enable>true</Enable>
          <LogLevel>warn</LogLevel>
        </SharedMemory>'
    else
        SHM_SECTION=""
    fi

    sudo tee "$CYCLONE_XML" > /dev/null <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <Interfaces>
$IFACE_LINE
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
$SHM_SECTION
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
fi

# Start RouDi only if SHM is enabled AND START_ROUDI is set
if [ "${CYCLONE_SHM:-}" = "1" ] && [ "${START_ROUDI:-}" = "1" ]; then
    ROUDI_CONFIG="/etc/iceoryx/roudi_config.toml"
    if command -v iox-roudi > /dev/null 2>&1; then
        # Check if RouDi is functional (socket exists)
        if ! [ -S /tmp/roudi ]; then
            if pgrep -x iox-roudi > /dev/null 2>&1; then
                echo "[CycloneDDS] Found stalled iox-roudi process without socket. Killing it..."
                sudo pkill -9 -x iox-roudi || true
                sudo rm -f /tmp/roudi /dev/shm/iceoryx_mgmt /tmp/roudi.lock /tmp/iox-unique-roudi.lock 2>/dev/null
                sleep 1
            fi
        fi

        if ! pgrep -x iox-roudi > /dev/null 2>&1; then
            echo "[CycloneDDS] Starting iceoryx RouDi daemon for SHM..."
            if [ -f "$ROUDI_CONFIG" ]; then
                iox-roudi -c "$ROUDI_CONFIG" &
            else
                iox-roudi &
            fi
            sleep 2 # Give it time to initialize
        fi
    fi
fi

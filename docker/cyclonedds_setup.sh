#!/bin/bash
# Generates CycloneDDS config based on CYCLONE_INTERFACE env var.
# If CYCLONE_INTERFACE is set, uses that interface. Otherwise, autodetermine.
# Called at build time (default XML) and sourced from .bashrc at runtime (override).

CYCLONE_XML="/etc/cyclonedds.xml"

# At runtime (sourced from .bashrc), regenerate if CYCLONE_INTERFACE is set
if [ -n "${CYCLONE_INTERFACE:-}" ]; then
    IFACE_LINE="        <NetworkInterface name=\"$CYCLONE_INTERFACE\" priority=\"default\" multicast=\"true\" autodetermine=\"false\"/>"
else
    # If XML already exists and no override, skip
    if [ -f "$CYCLONE_XML" ]; then
        return 0 2>/dev/null || exit 0
    fi
    IFACE_LINE='        <NetworkInterface autodetermine="true" priority="default" multicast="default" />'
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
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>warn</LogLevel>
    </SharedMemory>
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

# Start RouDi if not already running (required for CycloneDDS SHM)
if command -v iox-roudi > /dev/null 2>&1; then
    if ! pgrep -x iox-roudi > /dev/null 2>&1; then
        echo "[CycloneDDS] Starting iceoryx RouDi daemon for SHM..."
        iox-roudi &
    fi
fi

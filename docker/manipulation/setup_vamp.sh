#!/bin/bash
# ==============================================================================
# setup_vamp.sh
# ------------------------------------------------------------------------------
# Runs INSIDE the container at startup.
# Ensures frida_real.hh is in place, compiles VAMP via colcon,
# copies the .so to the correct location, and exports PYTHONPATH.
#
# Prerequisites (run on HOST before building container):
#   bash docker/manipulation/generate_frida_collisions.sh
#
# What this does:
#   1. Installs frida_real.hh from vamp_moveit_plugin/resources/ if missing
#   2. Registers frida_real in vamp/cmake/Python.cmake if not already there
#   3. Builds VAMP via colcon (skips if already built with frida_real)
#   4. Copies the compiled .so to vamp/src/vamp/_core/
#   5. Exports PYTHONPATH so `import vamp` works from ros2 run
# ==============================================================================

set -e

WORKSPACE="/workspace"
VAMP_SRC="$WORKSPACE/src/manipulation/packages/vamp"
VAMP_ROBOTS="$VAMP_SRC/src/impl/vamp/robots"
VAMP_CMAKE="$VAMP_SRC/cmake/Python.cmake"
RESOURCES="$WORKSPACE/src/manipulation/packages/vamp_moveit_plugin/resources"
FRIDA_HH_SRC="$RESOURCES/frida_real.hh"
FRIDA_HH_DST="$VAMP_ROBOTS/frida_real.hh"
VAMP_CORE="$VAMP_SRC/src/vamp/_core"

echo "[VAMP Setup] ════════════════════════════════════════════"
echo "[VAMP Setup]  FRIDA VAMP Setup"
echo "[VAMP Setup] ════════════════════════════════════════════"

# ── 1. Install frida_real.hh if missing ────────────────────────
if [ ! -f "$FRIDA_HH_DST" ]; then
    if [ -f "$FRIDA_HH_SRC" ]; then
        echo "[VAMP Setup] Installing frida_real.hh from resources..."
        mkdir -p "$VAMP_ROBOTS"
        cp "$FRIDA_HH_SRC" "$FRIDA_HH_DST"
        echo "[VAMP Setup] ✓ frida_real.hh installed (82 spheres)"
    else
        echo "[VAMP Setup] ✗ frida_real.hh not found in resources!"
        echo "[VAMP Setup]   Run generate_frida_collisions.sh on the HOST first."
        echo "[VAMP Setup]   VAMP will compile WITHOUT FRIDA support."
    fi
else
    echo "[VAMP Setup] ✓ frida_real.hh already in place"
fi

# ── 2. Register frida_real in Python.cmake if missing ──────────
if [ -f "$VAMP_CMAKE" ]; then
    if ! grep -q "frida_real" "$VAMP_CMAKE"; then
        echo "[VAMP Setup] Registering frida_real in Python.cmake..."
        sed -i 's/      baxter$/      baxter\n      frida_real/' "$VAMP_CMAKE"
        sed -i 's/      Baxter$/      Baxter\n      FRIDA_Real/' "$VAMP_CMAKE"
        echo "[VAMP Setup] ✓ frida_real registered"
    else
        echo "[VAMP Setup] ✓ frida_real already registered in Python.cmake"
    fi
fi

# ── 3. Check if rebuild is needed ──────────────────────────────
NEEDS_BUILD=false

SO_FILE=$(find "$VAMP_CORE" -name "_core_ext.cpython-*.so" 2>/dev/null | head -1)
if [ -z "$SO_FILE" ]; then
    NEEDS_BUILD=true
    echo "[VAMP Setup] No compiled .so found. Will build..."
fi

if [ "$NEEDS_BUILD" = false ] && [ -n "$SO_FILE" ]; then
    if ! strings "$SO_FILE" | grep -q "frida_real"; then
        NEEDS_BUILD=true
        echo "[VAMP Setup] frida_real not in compiled .so. Rebuilding..."
    fi
fi

# ── 4. Build VAMP via colcon ────────────────────────────────────
if [ "$NEEDS_BUILD" = true ]; then
    echo "[VAMP Setup] Building VAMP with colcon..."
    cd "$WORKSPACE"
    rm -rf build/vamp install/vamp
    colcon build --packages-select vamp \
        --cmake-args -DVAMP_BUILD_PYTHON_BINDINGS=ON \
        2>&1 | tail -5

    if [ $? -ne 0 ]; then
        echo "[VAMP Setup] ✗ VAMP build failed!"
        exit 1
    fi
    echo "[VAMP Setup] ✓ VAMP built successfully"

    NEW_SO=$(find "$WORKSPACE/build/vamp" -name "_core_ext.cpython-*.so" 2>/dev/null | head -1)
    if [ -n "$NEW_SO" ]; then
        mkdir -p "$VAMP_CORE"
        cp "$NEW_SO" "$VAMP_CORE/"
        echo "[VAMP Setup] ✓ .so copied to $VAMP_CORE"
    fi
else
    echo "[VAMP Setup] ✓ VAMP already compiled with frida_real"
fi

# ── 5. Export PYTHONPATH ────────────────────────────────────────
export PYTHONPATH="$VAMP_SRC/src:$PYTHONPATH"

# ── 6. Verify ──────────────────────────────────────────────────
VERIFY=$(python3 -c "
import sys
sys.path.insert(0, '$VAMP_SRC/src')
try:
    import vamp
    r = getattr(vamp, 'frida_real', None)
    print('OK:' + str(r.n_spheres()) if r else 'NO_FRIDA')
except Exception as e:
    print('FAIL:' + str(e))
" 2>/dev/null)

if echo "$VERIFY" | grep -q "^OK:"; then
    N=$(echo "$VERIFY" | cut -d: -f2)
    echo "[VAMP Setup] ✓ VAMP ready — frida_real with $N spheres"
elif [ "$VERIFY" = "NO_FRIDA" ]; then
    echo "[VAMP Setup] ⚠ VAMP works but frida_real not available"
else
    echo "[VAMP Setup] ✗ VAMP import failed: $VERIFY"
fi

echo "[VAMP Setup] ════════════════════════════════════════════"

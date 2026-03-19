#!/bin/bash
# ==============================================================================
# generate_frida_collisions.sh
# ------------------------------------------------------------------------------
# Corre en el HOST.
# Toma el URDF original de FRIDA, lo esferiza con FOAM, usa Cricket (vía Docker)
# para generar las ecuaciones de colisión y deposita el archivo frida_real.hh
# en la carpeta de VAMP lista para ser compilada.
# ==============================================================================

set -e # Detener si hay error

# --- RUTAS (Ajusta según la estructura de tu repo) ---
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." >/dev/null 2>&1 && pwd)"
URDF_DIR="$REPO_ROOT/robot_description/frida_description/urdf"
ORIGINAL_URDF="$URDF_DIR/TMR2025/frida_real.urdf" # O el archivo base correcto
SPHERIZED_URDF="$URDF_DIR/xarm/spherized_xarm/xarm6_spherized.urdf"
JSON_CONFIG="$REPO_ROOT/manipulation/packages/frida_vamp_bridge/config/frida.json"
VAMP_ROBOTS_DIR="$REPO_ROOT/manipulation/packages/vamp/src/impl/src/robots"

echo "Iniciando generación de colisiones para VAMP..."

# 1. ESFERIZACIÓN (FOAM)
echo "1/3: Esferizando URDF con FOAM..."
# Asumimos que FOAM está instalado en el Host, o usamos su Docker
if command -v foam &> /dev/null; then
    foam "$ORIGINAL_URDF" "$SPHERIZED_URDF"
else
    echo "FOAM no encontrado localmente. Intenta usar un alias o contenedor de FOAM."
    exit 1
fi
echo "Esferización completa: $SPHERIZED_URDF"

# 2. GENERACIÓN DE C++ (CRICKET)
echo "2/3: Generando C++ con Cricket (vía Docker)..."
# Usamos el contenedor oficial de Cricket montando el repositorio
docker run --rm \
    --user $(id -u):$(id -g) \
    -v "$REPO_ROOT:/workspace" \
    -w /workspace/manipulation/packages/frida_vamp_bridge \
    kavrakilab/cricket-tool:latest \
    cricket-tool frida.json

# 3. MOVER Y LIMPIAR
echo "3/3: Moviendo frida_real.hh al código fuente de VAMP..."
# Cricket suele dejar el archivo cerca de donde se ejecutó o del JSON
GENERATED_HH="$REPO_ROOT/manipulation/packages/frida_vamp_bridge/out/frida_real.hh"

if [ -f "$GENERATED_HH" ]; then
    mkdir -p "$VAMP_ROBOTS_DIR"
    cp "$GENERATED_HH" "$VAMP_ROBOTS_DIR/frida_real.hh"
    echo "Archivo frida_real.hh depositado en: $VAMP_ROBOTS_DIR"
else
    echo "ERROR: No se encontró el archivo generado por Cricket."
    exit 1
fi

echo "¡Generación completada! Ahora ejecuta run.sh --build para inyectar y compilar VAMP en Docker."
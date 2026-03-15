#!/usr/bin/env bash
# Download Gazebo Fuel models and install them into worlds/models/.
#
# Run once from the repo root to replace stub geometry with proper meshes:
#   ./scripts/download_fuel_models.sh
#
# Requirements: curl, unzip, python3
#
# What it does per model:
#   1. Downloads the Fuel zip archive (OpenRobotics collection)
#   2. Extracts into a temp dir
#   3. Copies to worlds/models/<local_name>/   (meshes/ and materials/ included)
#   4. Patches model:// URIs so they use <local_name> (matching the directory)
#   5. Injects the gz-sim-label-system plugin if not already present
#      (needed for per-instance label override by WorldGenerator)
#
# Existing model.config files are KEPT (they carry our descriptions).
# model.sdf is replaced with the Fuel version (patched).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$SCRIPT_DIR/../worlds/models"
FUEL_BASE="https://fuel.gazebosim.org/1.0/OpenRobotics/models"

# Map: local_directory_name → Fuel model name (exact, URL-unsafe chars allowed)
declare -A FUEL_MODELS=(
    [fire_extinguisher]="Extinguisher"
    [exit_sign]="ExitSign"
    [trash_can]="FIRST 2015 trash can"
    [suitcase]="Small Case"
    [office_chair]="Office Chair"
    [person]="Rescue Randy Sitting"
    [drink_can]="Coke"
    [backpack]="JanSport Backpack Red"
    [phone]="Samsung J8 Black"
    [drill]="Black and Decker Cordless Drill"
)

# Python helper — patches a model.sdf in-place:
#   • Renames model:// URIs from <fuel_name>/ to <local_name>/
#   • Adds gz-sim-label-system plugin inside <model> if absent
PATCH_PY=$(cat <<'PYEOF'
import sys, re
from xml.etree import ElementTree as ET

sdf_path, local_name, fuel_name = sys.argv[1], sys.argv[2], sys.argv[3]

# ── URI rename ───────────────────────────────────────────────────────────────
text = open(sdf_path).read()
if fuel_name != local_name:
    text = text.replace(f"model://{fuel_name}/", f"model://{local_name}/")

# ── Label plugin injection ────────────────────────────────────────────────────
LABEL_PLUGIN = (
    '  <plugin name="gz::sim::systems::Label" filename="gz-sim-label-system">\n'
    '    <label>1</label>\n'
    '  </plugin>\n'
)
if "gz-sim-label-system" not in text:
    # Insert before closing </model> tag
    text = re.sub(r'([ \t]*</model>)', LABEL_PLUGIN + r'\1', text, count=1)

open(sdf_path, "w").write(text)
print(f"  Patched {sdf_path}")
PYEOF
)

# ── Main loop ────────────────────────────────────────────────────────────────
for local_name in "${!FUEL_MODELS[@]}"; do
    fuel_name="${FUEL_MODELS[$local_name]}"
    encoded=$(python3 -c "import urllib.parse, sys; print(urllib.parse.quote(sys.argv[1]))" "$fuel_name")
    url="${FUEL_BASE}/${encoded}/zip"
    dest="${MODELS_DIR}/${local_name}"

    echo "⬇  ${fuel_name}  →  worlds/models/${local_name}/"

    tmpdir=$(mktemp -d)
    trap "rm -rf '$tmpdir'" EXIT

    # Download
    if ! curl -fsSL --retry 3 "$url" -o "${tmpdir}/model.zip"; then
        echo "   ✗ Download failed for ${fuel_name} — keeping stub."
        rm -rf "$tmpdir"
        trap - EXIT
        continue
    fi

    # Extract
    unzip -q "${tmpdir}/model.zip" -d "${tmpdir}/extracted/"

    # Fuel zips may place files directly or inside a subdirectory
    # Find the directory containing model.sdf
    sdf_file=$(find "${tmpdir}/extracted" -name "model.sdf" | head -1)
    if [ -z "$sdf_file" ]; then
        echo "   ✗ No model.sdf found in zip — keeping stub."
        rm -rf "$tmpdir"
        trap - EXIT
        continue
    fi
    src_dir=$(dirname "$sdf_file")

    # Copy files (preserve existing model.config if we customised it)
    mkdir -p "$dest"
    # Sync everything except model.config (keep ours)
    rsync -a --exclude="model.config" "${src_dir}/" "${dest}/" 2>/dev/null \
        || cp -r "${src_dir}/"* "${dest}/"

    # Patch URIs + inject label plugin
    python3 -c "$PATCH_PY" "${dest}/model.sdf" "$local_name" "$fuel_name"

    rm -rf "$tmpdir"
    trap - EXIT
    echo "   ✓ ${local_name} installed"
done

echo ""
echo "Done. Meshes are in worlds/models/. Run a scenario to verify visuals."
echo "Note: GZ_SIM_RESOURCE_PATH already includes worlds/models/ (set by arst_sim.launch.py)."

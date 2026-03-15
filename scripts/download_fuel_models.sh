#!/usr/bin/env bash
# Download Gazebo Fuel models and install them into worlds/models/.
#
# Run once from the repo root to replace stub geometry with proper meshes:
#   ./scripts/download_fuel_models.sh
#
# Requirements: gz (gz-fuel-tools), python3
#
# What it does per model:
#   1. Downloads via `gz fuel download` into the Fuel cache (~/.gz/fuel/)
#   2. Finds the latest cached version
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
FUEL_CACHE="${HOME}/.gz/fuel/fuel.gazebosim.org/openrobotics/models"
FUEL_BASE="https://fuel.gazebosim.org/1.0/OpenRobotics/models"

# Map: local_directory_name → Fuel model name (exact, case-sensitive)
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

sdf_path, local_name, fuel_name = sys.argv[1], sys.argv[2], sys.argv[3]

text = open(sdf_path).read()

# ── URI rename ───────────────────────────────────────────────────────────────
if fuel_name != local_name:
    text = text.replace(f"model://{fuel_name}/", f"model://{local_name}/")

# ── Relative mesh URI → model:// ─────────────────────────────────────────────
# When WorldGenerator embeds model SDF into the world SDF, the parser loses the
# model root context, so bare relative URIs can't be resolved.
# Convert relative meshes/ and materials/ URIs to model://<local_name>/...
text = re.sub(
    r'<uri>((?:meshes|materials)/[^<]+)</uri>',
    f'<uri>model://{local_name}/\\1</uri>',
    text)
# Same fix for PBR map tags (albedo_map, normal_map, etc.)
for tag in ('albedo_map', 'normal_map', 'roughness_map', 'metalness_map', 'emissive_map'):
    text = re.sub(
        rf'<{tag}>((?:meshes|materials)/[^<]+)</{tag}>',
        f'<{tag}>model://{local_name}/\\1</{tag}>',
        text)

# ── Strip Fuel contest artefacts ─────────────────────────────────────────────
# Some Fuel models embed an <include> pointing to "Artifact Proximity Detector"
# and legacy <plugin filename="ignition-gazebo-thermal-system"> blocks.
# Both cause Gazebo Harmonic to stall on remote fetches or log errors on load.
text = re.sub(
    r'\s*<include>\s*<uri>[^<]*Artifact Proximity Detector[^<]*</uri>\s*</include>',
    '', text)
text = re.sub(
    r'\s*<plugin\s+filename="ignition-gazebo-thermal-system"[^>]*>.*?</plugin>',
    '', text, flags=re.DOTALL)

# ── Label plugin injection ────────────────────────────────────────────────────
LABEL_PLUGIN = (
    '  <plugin name="gz::sim::systems::Label" filename="gz-sim-label-system">\n'
    '    <label>1</label>\n'
    '  </plugin>\n'
)
if "gz-sim-label-system" not in text:
    text = re.sub(r'([ \t]*</model>)', LABEL_PLUGIN + r'\1', text, count=1)

open(sdf_path, "w").write(text)
print(f"  Patched {sdf_path}")
PYEOF
)

# ── Main loop ────────────────────────────────────────────────────────────────
for local_name in "${!FUEL_MODELS[@]}"; do
    fuel_name="${FUEL_MODELS[$local_name]}"
    fuel_url="${FUEL_BASE}/${fuel_name}"
    dest="${MODELS_DIR}/${local_name}"
    # Fuel cache uses lowercase model names
    fuel_name_lower=$(echo "$fuel_name" | tr '[:upper:]' '[:lower:]')
    cache_dir="${FUEL_CACHE}/${fuel_name_lower}"

    echo "⬇  ${fuel_name}  →  worlds/models/${local_name}/"

    # Download via gz fuel (idempotent — skips if already cached)
    if ! gz fuel download -u "${fuel_url}" 2>&1 | grep -qv "^$"; then
        gz fuel download -u "${fuel_url}" 2>&1 || true
    fi
    gz fuel download -u "${fuel_url}" 2>/dev/null || true

    # Find the highest version in the cache
    if [ ! -d "${cache_dir}" ]; then
        echo "   ✗ Not found in Fuel cache after download — keeping stub."
        continue
    fi

    version=$(ls -1 "${cache_dir}" | sort -n | tail -1)
    src_dir="${cache_dir}/${version}"

    if [ ! -f "${src_dir}/model.sdf" ]; then
        echo "   ✗ No model.sdf in cache at ${src_dir} — keeping stub."
        continue
    fi

    # Copy files (preserve our model.config)
    mkdir -p "$dest"
    rsync -a --exclude="model.config" "${src_dir}/" "${dest}/" 2>/dev/null \
        || { cp -r "${src_dir}/"* "${dest}/"; rm -f "${dest}/model.config" 2>/dev/null || true; }

    # Patch URIs + inject label plugin
    python3 -c "$PATCH_PY" "${dest}/model.sdf" "$local_name" "$fuel_name"

    echo "   ✓ ${local_name} installed (from cache v${version})"
done

echo ""
echo "Done. Meshes are in worlds/models/. Run a scenario to verify visuals."
echo "Note: GZ_SIM_RESOURCE_PATH already includes worlds/models/ (set by arst_sim.launch.py)."

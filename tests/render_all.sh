#!/usr/bin/env bash
# render_all.sh - re-render every example .pecl scene to PNG in tests/.

set -e
cd "$(dirname "$0")/.."

# pick the right binary name per os.
if [[ -x ./pencil ]]; then
  PENCIL=./pencil
elif [[ -x ./pencil_test ]]; then
  PENCIL=./pencil_test
elif [[ -x ./pencil.exe ]]; then
  PENCIL=./pencil.exe
else
  echo "ERROR: no ./pencil binary found." 1>&2
  exit 1
fi

mkdir -p tests

echo "rendering demo..."
$PENCIL --demo tests/demo.png --quiet

for pecl in examples/*.pecl; do
  name=$(basename "$pecl" .pecl)
  if [[ "$name" == "common_mat" ]]; then continue; fi
  echo "rendering $name..."
  extra=""
  if [[ "$name" == "dispersion" || "$name" == "surf" ]]; then
    extra="--samples 1 --width 480 --height 360 --depth 4"
  elif [[ "$name" == "glass" ]]; then
    extra="--samples 2 --width 480 --height 360"
  elif [[ "$name" == "mesh_test" ]]; then
    extra="--samples 2 --width 480 --height 640 --depth 4"
  elif [[ "$name" == "bvh_stress" ]]; then
    extra="--samples 2 --width 400 --height 300 --depth 3"
  fi
  $PENCIL "$pecl" "tests/${name}.png" --quiet $extra
  if [[ "$name" == "bvh_stress" ]]; then
    echo "rendering $name (no BVH)..."
    $PENCIL "$pecl" "tests/${name}_nobvh.png" --no-bvh --quiet $extra
  fi
done

echo "rendering normals (AOV)..."
$PENCIL examples/three_spheres.pecl tests/three_spheres_normals.png --normals --quiet

echo "rendering demo with bloom..."
$PENCIL --demo tests/demo_bloom.png --bloom --quiet

echo ""
echo "done. tests in tests/:"
ls -1 tests/

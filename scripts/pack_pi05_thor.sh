#!/usr/bin/env bash
# Run with bash so errors never exit the user's interactive shell.
set -Eeuo pipefail

if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: bash scripts/pack_pi05_thor.sh [USB_MOUNT] [MODEL_ROOT]"
  echo "Packages step-49999 and benchmark code; does not include an offline Python environment."
  exit 0
fi

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
USB="${1:-/media/yanziyu/6745-2301}"
MODEL="${2:-$HOME/models/pi05-encos-fold-towel-93}"
mkdir -p "$ROOT/outputs"
STAGING="$(mktemp -d "$ROOT/outputs/pi05-thor-transfer.XXXXXXXX")"
LOG="$STAGING/transfer.log"
DEST="$USB/pi05-thor-offline"

exec > >(tee -a "$LOG") 2>&1
trap 'status=$?; echo; echo "FAILED at line $LINENO (status=$status). Local files and log: $STAGING"; exit "$status"' ERR

echo "Local staging and log: $STAGING"
echo "USB destination: $DEST"
if ! mountpoint -q -- "$USB"; then
  echo "ERROR: USB mount point not found: $USB"
  exit 2
fi
if ! findmnt -rn -M "$USB" -O rw >/dev/null; then
  echo "ERROR: USB filesystem is not mounted read-write. Stop and check the device."
  exit 2
fi

MODEL_FILES=(
  inference.py runtime tokenizer step-49999
  requirements-inference.txt training_config.json
  README.md LICENSE LICENSE_GEMMA.txt NOTICE
)
CODE_FILES=(
  Makefile deployment scripts/run_policy_worker.py scripts/pack_pi05_thor.sh
  config/pi05_deployment.yaml config/recording.yaml
  environments/PI05_THOR.md environments/pi05-thor-extra-requirements.txt
  docs/PI05_THOR_BENCHMARK.md docs/PI05_DEPLOYMENT.md docs/DEPLOYMENT.md
  tests/test_pi05_benchmark.py
)
for file in "${MODEL_FILES[@]}"; do
  test -e "$MODEL/$file"
done
for file in "${CODE_FILES[@]}"; do
  test -e "$ROOT/$file"
done
test -f "$MODEL/step-49999/params/manifest.ocdbt"
test -d "$MODEL/step-49999/assets/encos_dual_arm_towel"

required=$(du -sb -- "${MODEL_FILES[@]/#/$MODEL/}" | awk '{total += $1} END {printf "%.0f", total}')
required=$((required + 1073741824))
for directory in "$STAGING" "$USB"; do
  available=$(df -B1 --output=avail -- "$directory" | tail -n 1)
  if (( available < required )); then
    echo "ERROR: Insufficient free space at $directory; need about $required bytes."
    exit 2
  fi
done

echo "[1/5] Packing model on the local disk (one dot per approximately 100 MiB)..."
tar --exclude='__pycache__' --checkpoint=10240 --checkpoint-action=dot \
  -cf "$STAGING/model-step49999.tar" -C "$MODEL" "${MODEL_FILES[@]}"
echo
echo "[2/5] Packing benchmark code and environment documentation..."
tar --exclude='__pycache__' -cf "$STAGING/pi05-test-code.tar" -C "$ROOT" "${CODE_FILES[@]}"

echo "[3/5] Computing checksums from the LOCAL archives..."
cd "$STAGING"
sha256sum model-step49999.tar pi05-test-code.tar > SHA256SUMS

echo "[4/5] Copying archives to USB; waiting here is normal..."
mkdir -p "$DEST"
for file in model-step49999.tar pi05-test-code.tar; do
  cp -v -- "$STAGING/$file" "$DEST/$file.partial"
done
sync -f "$DEST"

echo "[5/5] Checking USB archives against the LOCAL checksums..."
for file in model-step49999.tar pi05-test-code.tar; do
  expected=$(awk -v name="$file" '$2 == name {print $1}' "$STAGING/SHA256SUMS")
  actual=$(sha256sum "$DEST/$file.partial")
  if [[ "${actual%% *}" != "$expected" ]]; then
    echo "ERROR: USB checksum mismatch: $file. Do not use this transfer."
    exit 3
  fi
  echo "$file: OK"
done
for file in model-step49999.tar pi05-test-code.tar; do
  mv -f -- "$DEST/$file.partial" "$DEST/$file"
done
cp -- "$STAGING/SHA256SUMS" "$DEST/SHA256SUMS"
sync -f "$DEST"
echo "TRANSFER COMPLETE: both archives verified and USB filesystem synchronized."
echo "USB files: $DEST"
echo "Keep local archives for retry or comparison: $STAGING"
echo "Safely eject the USB drive before unplugging it."

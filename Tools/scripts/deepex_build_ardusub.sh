#!/usr/bin/env bash

# Build ArduSub with the standard Waf commands, then archive the binary as
# ardusub-<commit8>-<board> with deepex_package_ardusub.py.
#
# Waf is called exactly as usual; packaging runs only if configure and build
# both succeed (set -e), and it refuses a dirty working tree.
#
# Usage, from the repository root inside the build container:
#   DEEPEX_FIRMWARE_ARCHIVE=/archive bash Tools/scripts/deepex_build_ardusub.sh navigator64

set -euo pipefail

board=${1:?usage: deepex_build_ardusub.sh <board>}
: "${DEEPEX_FIRMWARE_ARCHIVE:?set DEEPEX_FIRMWARE_ARCHIVE to the persistent archive directory}"

cd "$(git rev-parse --show-toplevel)"

./waf configure --board "${board}"
./waf sub

python3 Tools/scripts/deepex_package_ardusub.py \
    --binary "build/${board}/bin/ardusub" \
    --target "${board}" \
    --archive-dir "${DEEPEX_FIRMWARE_ARCHIVE}"

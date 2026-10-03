#!/usr/bin/env bash

set -euo pipefail

interface=vcan0
test_binary=${1:-build/linux/tests/test_vesc_socketcan_rx}

if [[ ${EUID} -ne 0 ]]; then
    echo "this isolated vcan test must run as root" >&2
    exit 1
fi

if ip link show "${interface}" >/dev/null 2>&1; then
    echo "${interface} already exists; refusing to alter it" >&2
    exit 1
fi

cleanup()
{
    ip link del "${interface}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

modprobe vcan 2>/dev/null || true
ip link add dev "${interface}" type vcan
ip link set "${interface}" up
"${test_binary}"

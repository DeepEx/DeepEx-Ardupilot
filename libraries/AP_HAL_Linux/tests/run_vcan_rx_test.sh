#!/usr/bin/env bash

set -euo pipefail

interface=vcan_vesc

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
ARDUPILOT_VCAN_IFACE="${interface}" python3 "$(dirname "$0")/test_socketcan_rx.py"

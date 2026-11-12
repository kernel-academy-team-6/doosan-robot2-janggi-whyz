#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_dir="$workspace_dir/src/doosan-robot2/dsr_common2"
case "$(uname -m)" in
  x86_64) sdk_arch=x86_64 ;;
  aarch64|arm64) sdk_arch=arm64 ;;
  *) echo 'Unsupported architecture' >&2; exit 1 ;;
esac
binary_dir="$workspace_dir/build/recovery_tool"
mkdir -p "$binary_dir"
# This workspace's hardware driver defaults to DRCF v2 (the E0509 reports GF02...).
c++ -std=c++17 -DDRCF_VERSION=2 -pthread -I "$sdk_dir/include" \
  "$workspace_dir/tools/doosan_recovery.cpp" \
  "$sdk_dir/lib/jazzy/$sdk_arch/libDRFL.a" \
  -lPocoFoundation -lPocoNet -o "$binary_dir/doosan_recovery"
exec "$binary_dir/doosan_recovery" "$@"

#!/usr/bin/env bash
# Fairy-Stockfish(장기 지원 Stockfish 파생 엔진) 바이너리를 bin/에 받는다. 처음 한 번.
# 장기판은 9x10 = 90칸이라 largeboard 빌드가 필요하다.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
TAG="${FSF_TAG:-fairy_sf_14}"
ARCH=x86-64
grep -qw bmi2 /proc/cpuinfo && ARCH=x86-64-bmi2
URL="https://github.com/fairy-stockfish/Fairy-Stockfish/releases/download/$TAG/fairy-stockfish-largeboard_$ARCH"
mkdir -p "$HERE/bin"
echo "다운로드: $URL"
curl -fL --retry 3 -o "$HERE/bin/fairy-stockfish.part" "$URL"
chmod +x "$HERE/bin/fairy-stockfish.part"
mv "$HERE/bin/fairy-stockfish.part" "$HERE/bin/fairy-stockfish"
echo "uci" | "$HERE/bin/fairy-stockfish" | grep -m1 "^id name"

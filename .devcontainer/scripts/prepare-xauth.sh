#!/usr/bin/env bash
set -euo pipefail

xauth_file="${1:?xauth file path is required}"
display_value="${2:-${DISPLAY:-}}"
source_xauth="${3:-${XAUTHORITY:-$HOME/.Xauthority}}"

mkdir -p "$(dirname "$xauth_file")"
touch "$xauth_file"
chmod 600 "$xauth_file"

if ! command -v xauth >/dev/null 2>&1; then
  exit 0
fi

if [ -z "$display_value" ]; then
  exit 0
fi

display_candidates=("$display_value")
if [[ "$display_value" == localhost:* ]]; then
  display_candidates+=(":${display_value#localhost:}")
elif [[ "$display_value" == 127.0.0.1:* ]]; then
  display_candidates+=(":${display_value#127.0.0.1:}")
fi

for candidate in "${display_candidates[@]}"; do
  if [ -f "$source_xauth" ]; then
    xauth -f "$source_xauth" nlist "$candidate" 2>/dev/null \
      | sed -e 's/^..../ffff/' \
      | xauth -f "$xauth_file" nmerge - 2>/dev/null || true
  else
    xauth nlist "$candidate" 2>/dev/null \
      | sed -e 's/^..../ffff/' \
      | xauth -f "$xauth_file" nmerge - 2>/dev/null || true
  fi
done

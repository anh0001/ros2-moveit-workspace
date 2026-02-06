#!/usr/bin/env bash
set -euo pipefail

xauth_file="${1:?xauth file path is required}"
display_value="${2:-${DISPLAY:-}}"

mkdir -p "$(dirname "$xauth_file")"
touch "$xauth_file"
chmod 600 "$xauth_file"

if ! command -v xauth >/dev/null 2>&1; then
  exit 0
fi

if [ -z "$display_value" ]; then
  exit 0
fi

xauth nlist "$display_value" 2>/dev/null \
  | sed -e 's/^..../ffff/' \
  | xauth -f "$xauth_file" nmerge - 2>/dev/null || true

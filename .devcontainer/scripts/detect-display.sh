#!/usr/bin/env bash
# Auto-detect X11 display for devcontainer
set -euo pipefail

# Allow manual override via environment variable
if [ -n "${X11_DISPLAY:-}" ]; then
    echo "$X11_DISPLAY"
    exit 0
fi

# Method 1: Check for active X11 sockets
ACTIVE_DISPLAYS=()
if [ -d /tmp/.X11-unix ]; then
    for socket in /tmp/.X11-unix/X*; do
        if [ -S "$socket" ]; then
            display_num="${socket##*/X}"
            ACTIVE_DISPLAYS+=("$display_num")
        fi
    done
fi

# Method 2: Check running Xorg processes
XORG_DISPLAYS=$(ps aux | grep -E '[X]org.*vt[0-9]' | grep -oP '(?<=vt)\d+' | head -1 || echo "")
if [ -n "$XORG_DISPLAYS" ]; then
    # VT2 usually corresponds to display :1, VT1 to :0
    XORG_DISPLAY=$((XORG_DISPLAYS - 1))
    if [ $XORG_DISPLAY -lt 0 ]; then
        XORG_DISPLAY=0
    fi
fi

# Method 3: Check for display :0 or :1 socket
if [ -S /tmp/.X11-unix/X0 ]; then
    PREFERRED_DISPLAY=":0"
elif [ -S /tmp/.X11-unix/X1 ]; then
    PREFERRED_DISPLAY=":1"
elif [ ${#ACTIVE_DISPLAYS[@]} -gt 0 ]; then
    # Use the first active display found
    PREFERRED_DISPLAY=":${ACTIVE_DISPLAYS[0]}"
elif [ -n "${XORG_DISPLAY:-}" ]; then
    PREFERRED_DISPLAY=":$XORG_DISPLAY"
else
    # Default fallback
    PREFERRED_DISPLAY=":0"
fi

echo "$PREFERRED_DISPLAY"

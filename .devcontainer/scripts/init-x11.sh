#!/usr/bin/env bash
# Initialize X11 for devcontainer - auto-detects display
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XAUTH_FILE="${1:-${SCRIPT_DIR}/../.docker.xauth}"

# Auto-detect display
DETECTED_DISPLAY=$("${SCRIPT_DIR}/detect-display.sh")
echo "🖥️  Detected display: $DETECTED_DISPLAY"

# Get Xauthority file
SOURCE_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
echo "🔑 Using Xauthority: $SOURCE_XAUTH"

# Enable X11 access for current user
echo "✅ Enabling X11 access..."
DISPLAY="$DETECTED_DISPLAY" xhost +SI:localuser:$(id -un) 2>/dev/null || true

# Prepare xauth file for container
echo "📝 Preparing container Xauthority..."
"${SCRIPT_DIR}/prepare-xauth.sh" "$XAUTH_FILE" "$DETECTED_DISPLAY" "$SOURCE_XAUTH"

# Write display config for container to read
CONFIG_FILE="${SCRIPT_DIR}/../.display.env"
cat > "$CONFIG_FILE" << EOF
# Auto-generated X11 display configuration
# Generated on: $(date)
DISPLAY=$DETECTED_DISPLAY
XAUTHORITY=/tmp/.docker.xauth
EOF

echo "✅ X11 setup complete!"
echo "   Display: $DETECTED_DISPLAY"
echo "   Xauth: $XAUTH_FILE"
echo ""
echo "💡 To manually override, set X11_DISPLAY environment variable:"
echo "   export X11_DISPLAY=:0"

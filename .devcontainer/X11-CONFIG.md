# X11 Display Configuration

The devcontainer **automatically detects** your X11 display on startup.

## Current Configuration

**Detected Display:** `:1` (auto-detected from `/tmp/.X11-unix/X1`)

## How to Override Display

If you need to use a different display (e.g., `:0`), set the environment variable **before** rebuilding:

```bash
# On your host machine
export X11_DISPLAY=:0

# Then rebuild the container in VSCode
# F1 → "Dev Containers: Rebuild Container"
```

## Manual Display Setup

If auto-detection doesn't work, you can manually set the display:

### Option 1: Environment Variable (Recommended)
```bash
export X11_DISPLAY=:0
```

### Option 2: Edit devcontainer.json
Edit [.devcontainer/devcontainer.json](./devcontainer.json) line 22:
```json
"DISPLAY": ":1",  // Change to :0 or your display number
```

## Verify Your Display

Check available X11 displays:
```bash
ls -1 /tmp/.X11-unix/
```

Output example:
```
X0   ← Display :0
X1   ← Display :1
X77  ← Display :77
```

## Troubleshooting

### Can't open display
1. Make sure X11 access is enabled:
   ```bash
   xhost +SI:localuser:$(id -un)
   ```

2. Verify display socket exists:
   ```bash
   ls -la /tmp/.X11-unix/X1  # Change X1 to your display number
   ```

3. Test X11 from host:
   ```bash
   DISPLAY=:1 xeyes  # Should show a window
   ```

### Wrong display detected
Set `X11_DISPLAY` environment variable before rebuilding:
```bash
export X11_DISPLAY=:0
```

### Using Wayland instead of X11
If you're using Wayland, you might need XWayland. The auto-detection script will try to find the appropriate display.

---

**Scripts:**
- Auto-detection: [scripts/detect-display.sh](./scripts/detect-display.sh)
- X11 initialization: [scripts/init-x11.sh](./scripts/init-x11.sh)

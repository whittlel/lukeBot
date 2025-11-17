#!/bin/bash
# Helper script to run LukeBot with proper display settings

# Try to set DISPLAY if not already set
if [ -z "$DISPLAY" ]; then
    echo "DISPLAY not set, attempting to set to :0"
    export DISPLAY=:0
fi

# Check if X server is accessible
if ! xdpyinfo &>/dev/null; then
    echo "WARNING: Cannot connect to X display"
    echo ""
    echo "Options:"
    echo "1. If you have a monitor connected:"
    echo "   export DISPLAY=:0"
    echo ""
    echo "2. For headless operation (SSH/remote):"
    echo "   Use VNC or run in headless mode"
    echo ""
    echo "3. Start a virtual display:"
    echo "   Xvfb :1 -screen 0 1920x1080x24 &"
    echo "   export DISPLAY=:1"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Run LukeBot
echo "Starting LukeBot..."
python3 main.py

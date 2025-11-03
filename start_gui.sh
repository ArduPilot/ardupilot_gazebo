#!/bin/bash
# Start Gazebo GUI (connects to running server)

export PATH="/opt/homebrew/opt/ruby/bin:$PATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/lib"

echo "Starting Gazebo GUI..."
echo "This connects to the running Gazebo server."
echo ""
echo "Controls:"
echo "  - Right-click + drag: Rotate view"
echo "  - Scroll: Zoom"
echo "  - Shift + left-click: Pan"
echo ""

gz sim -g

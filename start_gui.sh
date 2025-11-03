#!/bin/bash
# Start Gazebo GUI (connects to running server)

set -e

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping Gazebo GUI..."
    # Only kill GUI processes, not the server
    pkill -9 -f "gz sim -g" 2>/dev/null || true
    echo "✓ GUI stopped"
}
trap cleanup EXIT INT TERM

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

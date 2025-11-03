#!/bin/bash

set -e

# Cleanup function - kills ALL Gazebo processes
cleanup() {
    echo ""
    echo "Cleaning up Gazebo processes..."

    # Kill by name
    pkill -9 -f "gz sim" 2>/dev/null || true

    # Kill by port (belt and suspenders)
    lsof -ti:9002 2>/dev/null | xargs kill -9 2>/dev/null || true

    sleep 1

    # Verify clean
    if pgrep -f "gz sim" > /dev/null; then
        echo "⚠ Warning: Some Gazebo processes still running"
        ps aux | grep "gz sim" | grep -v grep
    else
        echo "✓ All Gazebo processes stopped"
    fi
}

# Clean up ANY existing processes before starting
echo "Cleaning up old Gazebo processes..."
cleanup

# Ensure cleanup on exit
trap cleanup EXIT INT TERM

export PATH="/opt/homebrew/opt/ruby/bin:$PATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

echo "Starting Gazebo warehouse world..."
gz sim -r -s worlds/iris_warehouse.sdf -v 4

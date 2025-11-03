#!/bin/bash

# Clean up any existing Gazebo processes first
pkill -9 gz 2>/dev/null || true
sleep 2

# Trap to ensure cleanup on exit
cleanup() {
    echo "Stopping Gazebo..."
    pkill -9 gz 2>/dev/null || true
}
trap cleanup EXIT

export PATH="/opt/homebrew/opt/ruby/bin:$PATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

echo "Starting Gazebo warehouse world..."
gz sim -r -s worlds/iris_warehouse.sdf -v 4

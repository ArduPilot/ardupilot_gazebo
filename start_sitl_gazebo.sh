#!/bin/bash
#
# Gazebo SITL Launcher - Based on nerv-uas gold-standard pattern
# Integrates Gazebo physics while maintaining proven MAVLink configuration
#

set -e

# Configuration
ARDUPILOT_DIR="${ARDUPILOT_DIR:-/Users/pdfinn/github.com/NERVsystems/ardupilot}"
ANDROID_IP=""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Parse arguments
WIPE_PARAMS=""
SHOW_MAP="--map"
SHOW_CONSOLE="--console"

while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--wipe)
            WIPE_PARAMS="-w"
            shift
            ;;
        --android-ip)
            ANDROID_IP="$2"
            shift 2
            ;;
        --no-map)
            SHOW_MAP=""
            shift
            ;;
        --no-console)
            SHOW_CONSOLE=""
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}  Gazebo + ArduPilot SITL Launcher${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
echo ""

# Check Gazebo is running
if ! pgrep -f "gz sim.*iris_runway" > /dev/null; then
    echo -e "${RED}ERROR: Gazebo not running!${NC}"
    echo "Start Gazebo first in another terminal:"
    echo "  cd /Users/pdfinn/github.com/NERVsystems/ardupilot_gazebo"
    echo "  ./run_gazebo.sh"
    exit 1
fi
echo -e "${GREEN}✓ Gazebo is running${NC}"

# Auto-detect Android IP if not provided
if [ -z "$ANDROID_IP" ]; then
    echo -e "${YELLOW}Auto-detecting Android device IP...${NC}"

    DEVICES=$(adb devices | grep -v "^List" | grep -v "emulator" | grep "device$" | awk '{print $1}')
    DEVICE_COUNT=$(echo "$DEVICES" | grep -v "^$" | wc -l | tr -d ' ')

    if [ "$DEVICE_COUNT" -eq 0 ]; then
        echo -e "${YELLOW}No Android device found - using localhost:14551${NC}"
        ANDROID_IP="127.0.0.1"
        ANDROID_PORT="14551"
    elif [ "$DEVICE_COUNT" -gt 1 ]; then
        FIRST_DEVICE=$(echo "$DEVICES" | head -1)
        echo -e "${YELLOW}Using first device: $FIRST_DEVICE${NC}"
        ANDROID_IP=$(adb -s "$FIRST_DEVICE" shell ip addr show wlan0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d'/' -f1 | tr -d '\r\n ')
        ANDROID_PORT="14550"
    else
        DEVICE=$(echo "$DEVICES" | head -1)
        ANDROID_IP=$(adb -s "$DEVICE" shell ip addr show wlan0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d'/' -f1 | tr -d '\r\n ')
        ANDROID_PORT="14550"
    fi

    if [ -z "$ANDROID_IP" ]; then
        echo -e "${YELLOW}Could not detect IP - using localhost:14551${NC}"
        ANDROID_IP="127.0.0.1"
        ANDROID_PORT="14551"
    fi
else
    ANDROID_PORT="14550"
fi

# Display configuration
echo -e "${BLUE}Configuration:${NC}"
echo "  Vehicle: ArduCopter"
echo "  Frame: gazebo-iris (Gazebo physics)"
echo "  Model: JSON (Gazebo communication)"
echo "  Android IP: $ANDROID_IP"
echo "  MAVLink Port: $ANDROID_PORT (bidirectional UDP)"
echo "  Gazebo Port: 9002 (JSON protocol)"
echo ""

# MAVProxy configuration (prevents telemetry loops)
MAVINIT_FILE="$HOME/.mavinit.scr"
cat > "$MAVINIT_FILE" << 'MAVINIT_EOF'
set fwdpos True
set shownoise False
MAVINIT_EOF

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up SITL processes...${NC}"
    pkill -9 -f "arducopter|mavproxy" 2>/dev/null || true
    rm -f /tmp/ArduCopter.log 2>/dev/null || true
    echo -e "${GREEN}SITL stopped${NC}"
}
trap cleanup EXIT

# Clean up any stale processes from previous runs
echo -e "${YELLOW}Cleaning up old SITL processes...${NC}"
pkill -9 -f "arducopter|mavproxy" 2>/dev/null || true
sleep 2

echo -e "${YELLOW}✓ MAVProxy configured (no telemetry loops)${NC}"
echo ""

# Start SITL
cd "$ARDUPILOT_DIR"

echo -e "${GREEN}Starting SITL with Gazebo integration...${NC}"
echo ""

Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    $SHOW_CONSOLE \
    $SHOW_MAP \
    --out=udp:${ANDROID_IP}:${ANDROID_PORT} \
    --no-rebuild \
    $WIPE_PARAMS

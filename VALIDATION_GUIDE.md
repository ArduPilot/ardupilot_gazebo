# Validation Guide: macOS arm64 Fix

This guide helps you validate the ArduPilotPlugin fix works on your system.

## What Was Fixed

**Issue #149**: ArduPilotPlugin failed to link on macOS arm64 due to missing abseil dependencies required by protobuf 33.0.

**PR #150**: Added explicit abseil library linking for macOS.

---

## Quick Validation (5 minutes)

### Step 1: Clean Build

```bash
cd /Users/pdfinn/github.com/NERVsystems/ardupilot_gazebo
cd build
rm -rf *
export GZ_VERSION=ionic
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make ArduPilotPlugin -j4
```

**Expected result**:
```
[100%] Built target ArduPilotPlugin
```

**If you see linking errors** about abseil symbols, the fix didn't work.

### Step 2: Verify Plugin Architecture

```bash
file build/libArduPilotPlugin.dylib
```

**Expected output**:
```
libArduPilotPlugin.dylib: Mach-O 64-bit dynamically linked shared library arm64
```

### Step 3: Test Plugin Loads in Gazebo

```bash
# Set up environment
export PATH="/opt/homebrew/opt/ruby/bin:$PATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

# Start Gazebo with iris drone (server only for macOS)
gz sim -r -s worlds/iris_runway.sdf > /tmp/gz_validation.log 2>&1 &
GZ_PID=$!

# Wait for initialization
sleep 10

# Check if plugin loaded
grep "ArduPilotPlugin" /tmp/gz_validation.log

# Stop Gazebo
kill $GZ_PID
```

**Expected output** should include:
```
[info] [ArduPilotPlugin.cc:639] [iris_with_gimbal] Advertising on /gimbal/cmd_roll.
[debug] [SystemManager.cc:80] Loaded system [ArduPilotPlugin] for entity [15]
[info] [ArduPilotPlugin.cc:1147] Found IMU sensor with name [...]
```

---

## Full Integration Test (15 minutes)

Test the complete Gazebo + SITL + MAVLink chain:

###  Terminal 1: Start Gazebo

```bash
cd /Users/pdfinn/github.com/NERVsystems/ardupilot_gazebo

export PATH="/opt/homebrew/opt/ruby/bin:$PATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds"

gz sim -r -s worlds/iris_runway.sdf
```

Wait for:
```
[info] [ArduPilotPlugin.cc:639] [iris_with_gimbal] Advertising on /gimbal/cmd_roll.
```

### Terminal 2: Start SITL

```bash
cd /Users/pdfinn/github.com/NERVsystems/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
```

**Expected**: SITL connects to Gazebo, MAVProxy console appears

### Terminal 3: Test MAVLink Connection

```bash
python3 -c "
from pymavlink import mavutil
conn = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print('Waiting for heartbeat...')
msg = conn.wait_heartbeat(timeout=10)
if msg:
    print(f'✓ Connected! System ID: {msg.get_srcSystem()}')
else:
    print('✗ No heartbeat')
"
```

**Expected**: Receives heartbeat from simulated drone

---

## Troubleshooting

### "Could not find shared library" for CameraZoomPlugin/GstCameraPlugin

These are expected if you only built ArduPilotPlugin. They're optional plugins. The error messages are:
```
[error] [SystemLoader.cc:154] Failed to load system plugin [CameraZoomPlugin]
[error] [SystemLoader.cc:154] Failed to load system plugin [GstCameraPlugin]
```

**Solution**: Build all plugins:
```bash
make -j4
```

### "SocketUDP Bind failed: Address already in use"

Another Gazebo instance is running.

**Solution**:
```bash
killall -9 gz
# Wait 3 seconds
# Try again
```

### "Another world of the same name is running"

Stale Gazebo process.

**Solution**: Same as above

---

## Success Criteria

✅ **ArduPilotPlugin compiles** without linker errors
✅ **Plugin loads** in Gazebo (check logs)
✅ **Gimbal topics** advertised
✅ **IMU sensor** detected
✅ **SITL connects** to Gazebo
✅ **MAVLink** outputs on port 14550

If all checks pass, the fix is working!

---

## What's Been Done

- ✅ Fix committed to branch: `fix/macos-arm64-abseil-linking`
- ✅ Pushed to fork: https://github.com/pdfinn/ardupilot_gazebo
- ✅ PR submitted: https://github.com/ArduPilot/ardupilot_gazebo/pull/150
- ✅ Issue updated: https://github.com/ArduPilot/ardupilot_gazebo/issues/149

---

## Using It Now (Before PR Merge)

Your local build already has the fix! You can use Gazebo + ArduPilot SITL immediately:

1. Gazebo is already built with the fix
2. Follow "Full Integration Test" above to launch everything
3. Connect your ATAK GCS to `udp:127.0.0.1:14550`
4. Test mission planning with 3D visualization!

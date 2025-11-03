# How To Use Gazebo + ArduPilot SITL

## Quick Start (2 Terminals)

### Terminal 1: Gazebo Server
```bash
cd /Users/pdfinn/github.com/NERVsystems/ardupilot_gazebo
./run_gazebo.sh
```

Wait for: `Loaded system [ArduPilotPlugin]`

### Terminal 2: SITL
```bash
cd /Users/pdfinn/github.com/NERVsystems/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

Wait for MAVProxy console to appear.

### Terminal 3: Connect Your GCS
Your ATAK GCS can now connect to: `udp:127.0.0.1:14550`

## That's It!

- Gazebo shows 3D drone simulation
- SITL provides flight physics
- MAVLink on port 14550 for your GCS
- Test mission planning with 3D visualization

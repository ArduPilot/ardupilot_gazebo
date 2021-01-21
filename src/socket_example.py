#!/usr/bin/env python3
'''
A stripped down version of ardupilot/libraries/SITL/examples/JSON/pybullet/robot.py

There is no physics engine or robot model - just the SITL-JSON socket interface

Usage
-----

1. Run the example:

    $ socket_example.py  

2. Start SITL:

    $ sim_vehicle.py -v Rover -f JSON:127.0.0.1 --aircraft=sitl_json

'''

import socket
import struct
import json

# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion

import time

RATE_HZ = 50.0
TIME_STEP = 1.0 / RATE_HZ
GRAVITY_MSS = 9.80665

time_now = 0
last_velocity = None

def to_tuple(vec3):
    '''convert a Vector3 to a tuple'''
    return (vec3.x, vec3.y, vec3.z)

def init():
  global time_now
  time_now = 0

def physics_step(pwm_in):

  global time_now
  time_now += TIME_STEP

  quaternion = Quaternion([1, 0, 0, 0])
  roll, pitch, yaw = quaternion.euler
  velocity = Vector3(0, 0, 0)
  position = Vector3(0, 0, 0)
  gyro = Vector3(0, 0, 0)

  # calculate acceleration
  global last_velocity
  if last_velocity is None:
      last_velocity = velocity

  accel = (velocity - last_velocity) * (1.0 / TIME_STEP)
  last_velocity = velocity

  # add in gravity in earth frame
  accel.z -= GRAVITY_MSS

  # convert accel to body frame
  # accel = dcm.transposed() * accel

  # convert to tuples
  accel = to_tuple(accel)
  gyro = to_tuple(gyro)
  position = to_tuple(position)
  velocity = to_tuple(velocity)
  euler = (roll, pitch, yaw)

  return time_now,gyro,accel,position,euler,velocity

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))
sock.settimeout(0.1)

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count = 1000

while True:

  py_time = time.time()

  try:
      data,address = sock.recvfrom(100)
  except Exception as ex:
      time.sleep(0.01)
      continue

  parse_format = 'HHI16H'
  magic = 18458

  if len(data) != struct.calcsize(parse_format):
    print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
    continue

  decoded = struct.unpack(parse_format,data)

  if magic != decoded[0]:
      print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
      continue

  frame_rate_hz = decoded[1]
  frame_count = decoded[2]
  pwm = decoded[3:]

  if frame_rate_hz != RATE_HZ:
      print("New frame rate %u" % frame_rate_hz)
      RATE_HZ = frame_rate_hz
      TIME_STEP = 1.0 / RATE_HZ

  # Check if the fame is in expected order
  if frame_count < last_SITL_frame:
    # Controller has reset, reset physics also
    init()
    print('Controller reset')
  elif frame_count == last_SITL_frame:
    # duplicate frame, skip
    print('Duplicate input frame')
    continue
  elif frame_count != last_SITL_frame + 1 and connected:
    print('Missed {0} input frames'.format(frame_count - last_SITL_frame - 1))
  last_SITL_frame = frame_count

  if not connected:
    connected = True
    print('Connected to {0}'.format(str(address)))
  frame_count += 1

  # physics time step
  phys_time,gyro,accel,pos,euler,velo = physics_step(pwm)

  # build JSON format
  IMU_fmt = {
    "gyro" : gyro,
    "accel_body" : accel
  }
  JSON_fmt = {
    "timestamp" : phys_time,
    "imu" : IMU_fmt,
    "position" : pos,
    "attitude" : euler,
    "velocity" : velo
  }
  JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

  # Send to AP
  sock.sendto(bytes(JSON_string,"ascii"), address)

  # Track frame rate
  if frame_count % print_frame_count == 0:
    now = time.time()
    total_time = now - frame_time
    print("%.2f fps T=%.3f dt=%.3f" % (print_frame_count/total_time, phys_time, total_time))
    frame_time = now


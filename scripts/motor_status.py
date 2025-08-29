#!/usr/bin/env python3

# To run this script set PYTHONPATH environment variable to register custom motor_stats message. 
# Run this in main directory:- export PYTHONPATH=$PYTHONPATH:`pwd`/build/ardupilot_gazebo-msgs_genmsg/python/

# Basic setup for dronecan:-
# modprobe vcan
# ip link add dev vcan0 type vcan
# ip link set up vcan0
# To check:- ip link show vcan0


import time
import argparse
import threading
import dronecan
from gz.transport13 import Node


try:
    from ardupilot_gazebo.msgs.motor_stats_pb2 import MotorStats
except ImportError:
    print("Error: Could not import the MotorStats protobuf message.")
    print("Please ensure you have compiled MotorStats.proto and the resulting")
    print("Python module is in your PYTHONPATH.")
    exit(1)

class MotorStatsSubscriber:
    def __init__(self, node_id, dronecan_port, model_name, joint_name):
        self._model_name = model_name
        self._joint_name = joint_name
        self._node_id = node_id
        self._dronecan_port = dronecan_port
        self._lock = threading.Lock()
        self._node = Node()
        self._dronecan_node = None
        self._initialized = False
        

        print(f"Initializing DroneCAN node {node_id} on '{dronecan_port}'...")
        try:
            self._dronecan_node = dronecan.make_node(dronecan_port, node_id=node_id, bitrate=1000000)
        except Exception as e:
            print(f"Error initializing DroneCAN node: {e}")
            print("Please ensure the CAN interface exists and is configured correctly.")
            return


        self._topic = f"/model/{self._model_name}/joint/{self._joint_name}/motor_stats"
        if self._node.subscribe(MotorStats, self._topic, self.motor_stats_cb):
            print("Subscribing to type {} on topic [{}]".format(MotorStats, self._topic))
            self._initialized = True 
        else:
            print("Error subscribing to topic [{}]".format(self._topic))
            return

    def motor_stats_cb(self, msg: MotorStats):
        if not self._initialized or self._dronecan_node is None:
            return
            
        try:
            with self._lock:
                dronecan_msg = dronecan.uavcan.equipment.esc.Status()
                dronecan_msg.esc_index = int(msg.motor_id)
                dronecan_msg.rpm = int(msg.rpm)
                dronecan_msg.voltage = float(msg.voltage)
                dronecan_msg.current = float(msg.current)
                dronecan_msg.temperature = float(msg.temperature)
                
                self._dronecan_node.broadcast(dronecan_msg)
        except Exception as e:
            print(f"Error broadcasting DroneCAN message: {e}")

def main():
    parser = argparse.ArgumentParser(description="Gazebo to DroneCAN ESC bridge.")
    parser.add_argument('dronecan_port', type=str, help="CAN interface name (e.g., vcan0, slcan:/dev/ttyACM0).")
    parser.add_argument('--node-id', type=int, default=100, help="DroneCAN node ID for this bridge.")
    parser.add_argument('--model-name', type=str, default="iris_with_gimbal", help="Name of the model in Gazebo.")
    parser.add_argument('--num-motors', type=int, default=4, help="Number of motors on the model.")
    args = parser.parse_args()

    print(f"Number of motors: {args.num_motors}")
    
    for i in range(args.num_motors):
        joint_name = f"iris_with_standoffs::rotor_{i}_joint"
        MotorStatsSubscriber(args.node_id, args.dronecan_port, args.model_name, joint_name)
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == '__main__':
    main()
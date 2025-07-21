#!/usr/bin/python3
import rclpy
from rclpy.node import Node as node
from std_msgs.msg import Float64

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as GzNode
import time
import threading


class GzROS2Bridge(node):
    def __init__(self):
        super().__init__('gz_ros2_bridge')
        
        self.current_publisher = self.create_publisher(
            Float64, 
            '/iris_rotor_0_current', 
            10
        )

        self.voltage_publisher = self.create_publisher(
            Float64, 
            '/iris_rotor_0_voltage', 
            10
        )
        
        self.gz_node = GzNode()
        self.gz_current_topic = "/iris_with_standoffs::rotor_0_joint/current"
        self.gz_voltage_topic = "/iris_with_standoffs::rotor_0_joint/voltage"

        
        if self.gz_node.subscribe(Double, self.gz_current_topic, self.current_callback):
            self.get_logger().info(f"Subscribing to current topic: {self.gz_current_topic}")
        else:
            self.get_logger().error(f"Failed to subscribe to voltage topic: {self.gz_current_topic}")

        if self.gz_node.subscribe(Double, self.gz_voltage_topic, self.voltage_callback):
            self.get_logger().info(f"Subscribing to voltage topic: {self.gz_voltage_topic}")
        else:
            self.get_logger().error(f"Failed to subscribe to voltage topic: {self.gz_voltage_topic}")
    
    def current_callback(self, msg: Double):
        current_msg = Float64()
        current_msg.data = msg.data
        
        self.current_publisher.publish(current_msg)
        
        self.get_logger().info(f"Bridged current data: {msg.data}")

    def voltage_callback(self, msg: Double):
        current_msg = Float64()
        current_msg.data = msg.data
        
        # Publish to ROS2
        self.voltage_publisher.publish(current_msg)
        
        # Log the data (optional)
        self.get_logger().info(f"Bridged voltage data: {msg.data}")


def main():
    rclpy.init()
    
    bridge = GzROS2Bridge()
    
    spin_thread = threading.Thread(target=lambda: rclpy.spin(bridge), daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(0.001)
    except KeyboardInterrupt:
        bridge.get_logger().info("Shutting down bridge...")
    
    # Cleanup
    bridge.destroy_node()
    rclpy.shutdown()
    print("Bridge shutdown complete")

if __name__ == "__main__":
    main()
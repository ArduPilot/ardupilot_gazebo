# ArduPilot Gazebo SITL Viewer

This is a simpler approach to Gazebo where ArduPilot SITL runs the FDM.
Gazebo is a 3D viewer for the vehicle compared to MAVProxy or a GCS.
This approach allows for easy extension of the Gazebo environment to support other
aspects of the environment and is useful if you don't want to simulate the vehicle physics
or collisions with terrain.

ArduPilot SITL outputs state data via DDS to the SITL Viewer.
SITL viewer converts the data to Gazebo's protobuf transport.
Gazebo currently does not send any data back to ArduPilot.

## Message Architecture

* NavSatFix
    * Source: Ardupilot ROS sensor_msgs::msg::NavSatFix 
    * Destination: Call the Gazebo TODO plugin `set_spherical_coordinates` service
    * This updates the position of the "vehicle"
* tf_static
    * Source: ArduPilot ROS static transforms tf2_msgs::msg::TFMessage
    * Destination: TODO
    * TODO this should update the home position of the world
    * TODO can we dynamically load terrain too from the AP terrain server?


# ArduPilot Ignition Gazebo plugin
This is ArduPilot official plugin for Ignition Gazebo.
It replaces the old Gazebo plugin to bring support of the next gen Gazebo simulator.
It also brings support for more features :
- more flexible data exchange between SITL and Ignition with JSON data sharing.
- more sensors supported.
- true Simulation lockstepping. It is now possible to use GDB to stop the Ignition time for debugging.
- Better 3D rendering

## Disclaimer :
The plugin is currently working, but we are working into bringing support for more feature and refine the API.

## Requirements :
Ubuntu able to run full 3D graphics: Native, VM, WSL, Docker.
Ignition Gazebo version 7.x (Ignition Garden) or greater. See https://ignitionrobotics.org/docs/garden/install_ubuntu

## Usage :

Install Ignition Gazebo Garden development libs and rapidjson:
````
sudo apt install rapidjson-dev libignition-gazebo7-dev
````

````
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
````

## Troubleshooting

See https://ignitionrobotics.org/docs/edifice/troubleshooting#ubuntu

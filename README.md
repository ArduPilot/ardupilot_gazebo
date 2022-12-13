# ArduPilot Gazebo Plugin

[![ubuntu-build](https://github.com/srmainwaring/ardupilot_gazebo-1/actions/workflows/ubuntu-build.yml/badge.svg)](https://github.com/srmainwaring/ardupilot_gazebo-1/actions/workflows/ubuntu-build.yml)

This is the official ArduPilot plugin for [Gazebo Sim](https://gazebosim.org/home).
It replaces the previous
[`ardupilot_gazebo`](https://github.com/khancyr/ardupilot_gazebo)
plugin and provides support for the latest release of the Gazebo simulator
[(Gazebo Garden)](https://gazebosim.org/docs/garden/install).

It also adds the following features:

- More flexible data exchange between SITL and Gazebo using JSON.
- Additional sensors supported.
- True simulation lockstepping. It is now possible to use GDB to stop
  the Gazebo time for debugging.
- Improved 3D rendering using the `ogre2` rendering engine.

The project comprises a Gazebo Sim plugin to connect to ArduPilot SITL
(Software In The Loop) and some example models and worlds.

## Prerequisites

Gazebo Sim Garden is supported on Ubuntu 20.04 (Focal) and (22.04) Jammy.
If you are running Ubuntu as a virtual machine you will need at least
Ubuntu 20.04 (Focal) in order to have the OpenGL support required for the
`ogre2` render engine. Gazebo Sim and ArduPilot SITL will also run on macOS
(Big Sur and Monterey; Intel and M1 devices).

Follow the instructions for a
[binary install of Gazebo Garden](https://gazebosim.org/docs/garden/install)
and verify that Gazebo is running correctly.

Set up an [ArduPilot development environment](https://ardupilot.org/dev/index.html).
In the following it is assumed that you are able to run ArduPilot SITL using
the [MAVProxy GCS](https://ardupilot.org/mavproxy/index.html).

## Installation

Install additional dependencies:

### Ubuntu

```bash
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
```

### macOS

```bash
brew update
brew install rapidjson
```

Clone the repo and build:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

## Usage

### Configure environment

Set the Gazebo environment variables in your `.bashrc` or `.zshrc` or in 
the terminal used to run Gazebo.

#### Terminal

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

#### .bashrc or .zshrc

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Reload your terminal with `source ~/.bashrc` (or `source ~/.zshrc` on macOS).

### Run Gazebo

```bash
gz sim -v4 -r iris_arducopter_runway.world
```

The `-v4` parameter is not mandatory, it shows additional information and is
useful for troubleshooting.

### Run ArduPilot SITL

To run an ArduPilot simulation with Gazebo, the frame should have `gazebo-`
in it and have `JSON` as model. Other commandline parameters are the same
as usal on SITL.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### Arm and takeoff

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

## Troubleshooting

For issues concerning installing and running Gazebo on your platform please
consult the Gazebo Sim documentation for [troubleshooting frequent issues](https://gazebosim.org/docs/garden/troubleshooting#ubuntu).

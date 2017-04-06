# Ardupilot Gazebo plugin 

## Requirements :
Gazebo version 7.x or 8.x  
The dev branch will works on gazebo >= 9.x  

## Disclamer : 
This is a playground until I get some time to push the correct patch to gazebo master (I got hard time to work with mercurial..)!  
So you can expect things to not be up-to-date.  
This assume that your are using Ubuntu 16.04

## Usage :
I assume you already have Gazebo installed with ROS (or without).  
If you don't have it yet, install ROS with sudo apt install ros-kinetic-desktop-full (follow instruction here http://wiki.ros.org/kinetic/Installation/Ubuntu).  
Or install directly gazebo8 from http://gazebosim.org/tutorials?tut=install_ubuntu  
libgazebo7-dev or libgazebo8-dev must be installed.

````
git clone the repo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
````
DONE !

Now launch a world file with a copter/rover/plane and ardupilot plugin, and it should work! 
(I will try to add some world file and model later)

## HELP 

launch cmd :  
ROVER
````
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --map --console -I0
````
COPTER
````
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I0
````

### Missing libArduPilotPlugin.so  
In case you see this message, check you have no error after sudo make install.  
If no error use "ls" on the install path given to see if the plugin is really here.  
If this is correct, check with "cat /usr/share/gazebo/setup.sh" the variable GAZEBO_PLUGIN_PATH. It should be the same as the install path. If not use "cp" to copy the lib to right path. (example : cp /usr/lib/x86_64-linux-gnu/gazebo-7/plugins/libArduPilotPlugin.so /usr/lib/x86_64-linux-gnu/gazebo-7.0/plugins/ ).  
I don't know why it can have path mistmatch ... but it could append if you install gazebo from sasc-gazebo-sitl .


### Future(not activated yet)
To use Gazebo gps, you must offset the heading of +90° as gazebo gps is NWU and ardupilot is NED 
(I don't use GPS altitude for now)  
example : for SITL default location
````
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-35.363261</latitude_deg>
      <longitude_deg>149.165230</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>87</heading_deg>
    </spherical_coordinates>
````

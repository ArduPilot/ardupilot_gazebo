# ardupilot_gazebo

## Requirements :
Gazebo version 7.x or 8.x  
The dev branch will works on gazebo >= 9.x  
my branch https://github.com/khancyr/ardupilot/tree/multi_sitl_improve_rover (will be merge soon)

## Disclamer : 
This is a playground until I get some time to push the correct patch to gazebo master (I got hard time to work with mercurial..)!
So you can expect things to not be up-to-date. 

IRLOCK PLUGIN ISN'T WORKING NOW!

## Usage :
I assume you already have Gazebo installed with ROS (or without)  
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

launch cmd :  
ROVER
````
sim_vehicle.py -v APMrover2 -f gazebo-rover  -m --mav10 --map --console -I0
````
COPTER
````
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --map --console -I1
````
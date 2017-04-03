# ardupilot_gazebo

## Requirements :
Gazebo version 7.x or 8.x
The dev branch will works on gazebo >= 9.x

## Disclamer : 
This is a playground until I get some time to push the correct patch to gazebo master (I got hard time to work with mercurial..)!
So you can expect things to not be up-to-date 

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

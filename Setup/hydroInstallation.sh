#!/bin/bash
#Setup your sources.list
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

#Set up your keys
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

#Installation
apt-get update
apt-get install -y ros-hydro-desktop-full

#Initialize rosdep
rosdep init
rosdep update

#Environment setup
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc

#Getting rosinstall
apt-get install -y python-rosinstall

#.ros change owner
chown -R $USER:$USER ~/.ros

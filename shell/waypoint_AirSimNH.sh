#!/bin/bash

cd /home/amsl/AirSim_projects/AirSimNH/LinuxNoEditor
./AirSimNH.sh -ResX=640 -ResY=480 -windowed -opengl -settings="/home/amsl/catkin_ws/src/RGBD_LiDAR_airsim_controller/setting_files/waypoint_flight.json"

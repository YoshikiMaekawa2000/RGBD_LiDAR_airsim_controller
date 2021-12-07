#!/bin/bash

image_name='rgbd_lidar_airsim_controller'
image_tag='noetic'

docker build -t $image_name:$image_tag .
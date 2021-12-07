#!/bin/bash

echo "=====RUN DOCKER====="

image_name="rgbd_lidar_airsim_controller"
tag_name="noetic"
script_dir=$(cd $(dirname $0); pwd)

docker run -it \
    --net="host" \
    --name="rgbd_lidar_airsim_controller" \
    --volume="$script_dir/:/home/airsim_ws/$image_name/" \
    $image_name:$tag_name \
    bash -c "cd /home/airsim_ws/$image_name && cd build && cmake .. && make -j && bash"
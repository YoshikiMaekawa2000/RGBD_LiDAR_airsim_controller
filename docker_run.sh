#!/bin/bash

echo "=====RUN DOCKER====="

image_name="rgbd_lidar_airsim_controller"
tag_name="noetic"
script_dir=$(cd $(dirname $0); pwd)

docker run -it \
    --net="host" \
    --name="rgbd_lidar_airsim_controller" \
    --volume="$script_dir/:/home/airsim_ws/$image_name/" \
    --volume="/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/:/home/ssd_dir/" \
    $image_name:$tag_name \
    bash -c "cd /home/airsim_ws/$image_name && cd build && cmake .. && make -j && bash"
# RGBD_LiDAR_airsim_controller

## Requirements
* Unreal Engine
* Microsoft Airsim
* Point Cloud Library

## How to install and build
```
git clone https://github.com/Hibiki1020/RGBD_LiDAR_airsim_controller.git
cd RGBD_LiDAR_airsim_controller
mkdir Build
cd build
cmake .. && make -j
```

## How to run
```
cd RGBD_LiDAR_airsim_controller
cd build
```
If you want to save camera image and depth image and lidar scan, change parameter and file path in include file and build, then
```
./get_lidar_rgbd_image
```
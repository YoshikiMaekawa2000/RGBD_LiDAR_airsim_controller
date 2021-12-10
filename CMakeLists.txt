cmake_minimum_required(VERSION 3.5.0)
project(RGBD_LiDAR_airsim_controller)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(OpenCV)
find_package(PCL)

## for "cnpy"
link_directories(/usr/local/lib)

# set(AIRSIM_ROOT ../AirSim)
find_path(AIRSIM_ROOT
	NAMES AirSim.sln 
	PATHS "./AirSim" "../AirSim" "../../AirSim" "../../../AirSim" "../../../../AirSim"
)
message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")

add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

include_directories(
    include
	${AIRSIM_ROOT}/AirLib/include
	${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include
	${AIRSIM_ROOT}/MavLinkCom/include
	${AIRSIM_ROOT}/MavLinkCom/common_utils
	${AIRSIM_ROOT}/AirLib/deps/eigen3
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(get_lidar_rgbd_image src/get_lidar_rgbd_image.cpp)
target_link_libraries(get_lidar_rgbd_image AirLib rpc ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} cnpy)
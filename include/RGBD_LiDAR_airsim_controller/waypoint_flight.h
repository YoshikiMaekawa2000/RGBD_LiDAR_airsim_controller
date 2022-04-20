#ifndef __WAYPOINT_FLIGHT_H
#define __WAYPOINT_FLIGHT_H

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <unistd.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

struct waypoint{
    float x;
    float y;
    float z;

    int self_idx; //Index of waypoint
    int idx[4]; //Index of neighbor waypoint
};

class WaypointFlight{
    private:
         /*client*/
        msr::airlib::MultirotorRpcLibClient _client;

        /*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;

        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;

        //Waypoint Data
        std::vector<waypoint> waypoints;
};


#endif
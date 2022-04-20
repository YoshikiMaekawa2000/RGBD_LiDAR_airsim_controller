#ifndef __WAYPOINT_FLIGHT_H
#define __WAYPOINT_FLIGHT_H

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <unistd.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <fstream>
#include <sstream>
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
        std::vector<waypoint> target_points;
        std::vector<Eigen::Vector3f> _waypoints;
		std::vector<Eigen::Vector3f> _path;
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size = 0;
        int num_target_points = 20;
		double _height = -3.0;
		double _path_resolution = 2.5;
		double _noise_xy = 1.5;
		double _noise_z = 0.1;
		double _velocity = 15.0;
		double _cutting_corner = 3.0;
        int before_idx = 0;

        std::string waypoint_file = "/home/amsl/cpp/RGBD_LiDAR_airsim_controller/waypoint_data/waypoint.csv";
        //std::string waypoint_file = "/home/amsl/cpp/RGBD_LiDAR_airsim_controller/place_data/RGBD_LiDAR_Attitude_Estimation/Building99/random_place2022_02_07_2.csv";

    public:
        WaypointFlight();
        ~WaypointFlight();
        void setWayPoints();
        bool load_csv();
        std::vector<std::string> split(std::string& input, char delimiter);
        std::vector<float> string_to_float(std::vector<std::string> string_data);
        waypoint create_waypoint(std::vector<float> farray);
        int chooseWaypoint(int waypoint_idx, std::vector<waypoint> target_points);
        int random_int(int min, int max);
        void devidePath(void);
        void printPath(void);
        void clientInitialization(void);
        void addNoise(Eigen::Vector3f& point);
        void startFlight(void);
};


#endif
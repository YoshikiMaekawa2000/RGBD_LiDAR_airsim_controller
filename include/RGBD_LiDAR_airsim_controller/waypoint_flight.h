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

#include <thread>
#include <cstdio>
#include <cstdint>

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
		double _height = -4.5;
		double _path_resolution = 2.5;
		double _noise_xy = 1.5;
		double _noise_z = 0.1;
		double _velocity = 10.0;
		double _cutting_corner = 3.0;
        int before_idx = 0;

        std::string waypoint_file = "/home/amsl/cpp/RGBD_LiDAR_airsim_controller/waypoint_data/waypoint.csv";

        //Collect parameter
        bool end_checker = false;
        int interval_seconds = 10; //miliseconds
        int pic_size = 224;
        int num_target_points = 100;
        float standard_deviation = 3.0;

        std::string save_data_top_path = "/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/flight_airsim_image/new_sequence2";
        std::string rgb_image_directory = "/camera_image";
        std::string save_csv_file_name = "data_list.csv";

    public:
        WaypointFlight();
        ~WaypointFlight();
        void setWayPoints();
        bool load_csv();
        std::vector<std::string> split(std::string& input, char delimiter);
        std::vector<float> string_to_float(std::vector<std::string> string_data);
        waypoint create_waypoint(std::vector<float> farray);
        waypoint create_slide_waypoint(waypoint selected_waypoint);
        int chooseWaypoint(int waypoint_idx, std::vector<waypoint> target_points);
        int random_int(int min, int max);
        void devidePath(void);
        void printPath(void);
        void clientInitialization(void);
        void addNoise(Eigen::Vector3f& point);
        void startFlight(void);
        void collectData(void);
        void spin();
        msr::airlib::Pose getPose(void);
        cv::Mat get_image(void);
        std::string save_camera_image(cv::Mat camera_image, int num);
        void save_csv(int num_count, float process_time, std::string camera_image_file_name, float x, float y, float z, float roll, float pitch, float yaw);
};


#endif
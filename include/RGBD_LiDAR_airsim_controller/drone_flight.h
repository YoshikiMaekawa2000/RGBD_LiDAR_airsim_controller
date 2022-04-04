#ifndef __DRONE_FLIGHT_H
#define __DRONE_FLIGHT_H


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

class DroneFlight{
    private:
        /*client*/
        msr::airlib::MultirotorRpcLibClient _client;

        /*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;

        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;

        //place csv data
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size = 0; //csvファイル内のデータ数

        //config
        std::string save_data_top_path = "/media/hello";

        int max_sequence = 100;

        //Parameters
        const bool _random_weather = true;
        const int _wait_time_millisec = 270;
        const bool color_check = true; //save image as mono or RGB

        /*parameter-pose*/
        const float _roll_max = M_PI/180.0 * 30.0;
        const float _roll_min = -1.0 * M_PI/180.0 * 30.0;

        const float _pitch_max = M_PI/180.0 * 30.0;
        const float _pitch_min = -1.0 * M_PI/180.0 * 30.0;

        const float _yaw_max = M_PI/180.0 * 30.0;
        const float _yaw_min = -1.0 * M_PI/180.0 * 30.0;

        const float resolution = M_PI/180.0 * 1.0;

        
    public:
        DroneFlight();
        ~DroneFlight();
        void client_initialization();
        void update_state();
        void spin();
        std::vector<std::string> split(std::string& input, char delimiter);
        void set_drone_random_point();

};

#endif
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
        std::string save_csv_file_name = "data_list.csv";
        std::string place_csv_root_path = "/home/amsl/cpp/RGBD_LiDAR_airsim_controller/place_data/RGBD_LiDAR_Attitude_Estimation/Building99/";
        std::string place_csv_name = "random_place2022_02_07_2.csv";

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

        bool check_1_deg_increment = false;


    public:
        DroneFlight();
        ~DroneFlight();
        void client_initialization();
        void update_state();
        bool load_csv();
        void spin();
        std::vector<std::string> split(std::string& input, char delimiter);
        void set_drone_random_point(std::mt19937& mt);
        int random_int(int min, int max);
        std::vector<float> string_to_float(std::vector<std::string> tmp_place_csv_data);
        float check_deg(float deg);
        float convert_angle(float rad);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);

};

#endif
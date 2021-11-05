#ifndef __GET_LIDAR_RGBD_IMAGE_H
#define __GET_LIDAR_RGBD_IMAGE_H


#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <unistd.h>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

class GetLidarRGBDImage{
    private:
        /*client*/
        msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;
        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;
        /*pc*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};

        //config
        std::string save_data_top_path = "/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/lidar_rgbd_image_data/test";
        std::string rgb_image_directory = "/rgb_image";
        std::string depth_image_directory = "/depth_image";
        std::string lidar_data_directory = "/lidar";

        int num_total = 100;

        //place csv data
        std::string place_csv_root_path = "/home/amsl/cpp/kawai_airsim_controller/place_data/Building99/";
        std::string place_csv_name = "random_place.csv";
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size = 0; //csvファイル内のデータ数
        int image_counter = 0;
        bool save_color_checker = false;

        //Parameters
        const bool _random_weather = true;
        const int _wait_time_millisec = 20;

        /*parameter-pose*/
        const float _roll_max = M_PI/180.0 * 30.0;
        const float _roll_min = -1.0 * M_PI/180.0 * 30.0;

        const float _pitch_max = M_PI/180.0 * 30.0;
        const float _pitch_min = -1.0 * M_PI/180.0 * 30.0;

        const float _yaw_max = M_PI/180.0 * 40.0;
        const float _yaw_min = -1.0 * M_PI/180.0 * 40.0;

        const float resolution = M_PI/180.0 * 1.0;

        bool check_1_deg_increment = false;

    public:
        GetLidarRGBDImage();
        ~GetLidarRGBDImage();
        void client_initialization();
        void update_state();
        void spin();
        bool load_csv();
        void save_data();
        int random_int(int min, int max);
        std::vector<float> string_to_float(std::vector<std::string> tmp_place_csv_data);
        float check_deg(float deg);
        float convert_angle(float rad);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
        std::vector<std::string> split(std::string& input, char delimiter);
};

#endif
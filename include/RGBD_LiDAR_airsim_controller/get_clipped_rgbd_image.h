#ifndef __GET_CLIPPED_RGBD_IMAGE_H
#define __GET_CLIPPED_RGBD_IMAGE_H


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

class GetLidarRGBDImage{
    private:
        /*client*/
        //msr::airlib::MultirotorRpcLibClient _client;


		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;
        /*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;

        //config
        std::string save_data_top_path = "/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/rgbd_airsim_image/color_depth_images/test50";
        std::string rgb_image_directory = "/camera_image";
        std::string depth_image_directory = "/depth_image";

        std::string save_csv_file_name = "data_list.csv";

        std::string place_csv_root_path = "/home/amsl/cpp/RGBD_LiDAR_airsim_controller/place_data/SII2022_Waypoint/AirSimNH/";
        std::string place_csv_name = "random_place.csv";

        int num_total = 10;
        int clip_num = 5;
        const int size_image = 480;

        //place csv data
        std::vector< std::vector<std::string> > csv_data;
        size_t csv_data_size = 0; //csvファイル内のデータ数
        int image_counter = 0;
        const bool save_color_checker = true;

        /*Point Cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};

        //Parameters
        const bool _random_weather = true;
        const int _wait_time_millisec = 10;
        const bool color_check = false; //save image as mono or RGB

        /*parameter-pose*/
        const float _roll_max = M_PI/180.0 * 30.0;
        const float _roll_min = -1.0 * M_PI/180.0 * 30.0;

        const float _pitch_max = M_PI/180.0 * 30.0;
        const float _pitch_min = -1.0 * M_PI/180.0 * 30.0;

        const float _yaw_max = M_PI/180.0 * 45.0;
        const float _yaw_min = -1.0 * M_PI/180.0 * 45.0;

        const float resolution = M_PI/180.0 * 1.0;

        bool check_1_deg_increment = false;

    public:
        GetLidarRGBDImage(msr::airlib::MultirotorRpcLibClient &_client);
        ~GetLidarRGBDImage();
        void client_initialization(msr::airlib::MultirotorRpcLibClient &_client);
        void update_state(msr::airlib::MultirotorRpcLibClient &_client);
        void spin(msr::airlib::MultirotorRpcLibClient &_client);
        bool load_csv();
        void save_data(msr::airlib::MultirotorRpcLibClient &_client);
        int random_int(int min, int max);
        std::vector<float> string_to_float(std::vector<std::string> tmp_place_csv_data);
        float check_deg(float deg);
        float convert_angle(float rad);
        void eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q);
        std::vector<std::string> split(std::string& input, char delimiter);
        cv::Mat get_image(msr::airlib::MultirotorRpcLibClient &_client, bool save_color_checker);
        cv::Mat get_depth_image(msr::airlib::MultirotorRpcLibClient &_client);
        std::vector< std::vector<int> > get_clip_place(cv::Mat image);
        std::vector<cv::Mat> get_clipped_camera_images(std::vector<std::vector<int> > clip_place, cv::Mat camera_image);
        void save_csv(std::string camera_image_file_name, std::string depth_image_file_name, float x, float y, float z, float roll, float pitch, float yaw);
        std::vector<std::string> save_camera_images(std::vector<cv::Mat> clipped_camera_images, int num);
        std::vector<std::string> save_depth_images(std::vector<cv::Mat> clipped_depth_images, int num);
};

#endif
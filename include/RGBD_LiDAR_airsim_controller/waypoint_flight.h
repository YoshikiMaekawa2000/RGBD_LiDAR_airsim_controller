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

#include "common/VectorMath.hpp"

#include <thread>
#include <cstdio>
#include <cstdint>

class Vector3 {
public:
  float x;
  float y;
  float z;
  Vector3(float x, float y, float z): x(x), y(y), z(z) {}
};

enum class EulerOrder {
  XYZ,
  XZY,
  YXZ,
  YZX,
  ZXY,
  ZYX
};

class EulerAngle {
public:
  float x;
  float y;
  float z;
  EulerOrder order;
  EulerAngle(float x, float y, float z, EulerOrder order): x(x), y(y), z(z), order(order) {}
};

class Quaternion{
public:
  float x;
  float y;
  float z;
  float w;
  Quaternion(float x, float y, float z, float w): x(x), y(y),z(z), w(w) {}
  Quaternion operator*(const Quaternion q) const;
  Vector3 rotate(const Vector3 v) const;
};

Quaternion conjugate(const Quaternion q) {
  return Quaternion(-q.x, -q.y, -q.z, q.w);
}

Quaternion Quaternion::operator*(const Quaternion q) const {
  return Quaternion(
    w * q.x - z * q.y + y * q.z + x * q.w,
    z * q.x + w * q.y - x * q.z + y * q.w,
    -y * q.x + x * q.y + w * q.z + z * q.w,
    -x * q.x - y * q.y - z * q.z + w * q.w
  );
}

Vector3 Quaternion::rotate(const Vector3 v) const {
  auto vq = Quaternion(v.x, v.y, v.z, 0);
  auto cq = conjugate(*this);
  auto mq = *this * vq * cq;
  return Vector3(mq.x, mq.y, mq.z);
}

EulerAngle toEulerAngle(Quaternion q, EulerOrder order) {
  if (order == EulerOrder::XYZ) {
    auto sy = 2 * q.x * q.z + 2 * q.y * q.w;
    auto unlocked = std::abs(sy) < 0.99999f;
    return EulerAngle(
      unlocked ? std::atan2(-(2 * q.y * q.z - 2 * q.x * q.w), 2 * q.w * q.w + 2 * q.z * q.z - 1)
        : std::atan2(2 * q.y * q.z + 2 * q.x * q.w, 2 * q.w * q.w + 2 * q.y * q.y - 1),
      std::asin(sy),
      unlocked ? std::atan2(-(2 * q.x * q.y - 2 * q.z * q.w), 2 * q.w * q.w + 2 * q.x * q.x - 1) : 0,
      order
    );
  } else if (order == EulerOrder::XZY) {
    auto sz = -(2 * q.x * q.y - 2 * q.z * q.w);
    auto unlocked = std::abs(sz) < 0.99999f;
    return EulerAngle(
      unlocked ? std::atan2(2 * q.y * q.z + 2 * q.x * q.w, 2 * q.w * q.w + 2 * q.y * q.y - 1)
        : std::atan2(-(2 * q.y * q.z - 2 * q.x * q.w), 2 * q.w * q.w + 2 * q.z * q.z - 1),
      unlocked ? std::atan2(2 * q.x * q.z + 2 * q.y * q.w, 2 * q.w * q.w + 2 * q.x * q.x - 1) : 0,
      std::asin(sz),
      order
    );
  } else if (order == EulerOrder::YXZ) {
    auto sx = -(2 * q.y * q.z - 2 * q.x * q.w);
    auto unlocked = std::abs(sx) < 0.99999f;
    return EulerAngle(
      std::asin(sx),
      unlocked ? std::atan2(2 * q.x * q.z + 2 * q.y * q.w, 2 * q.w * q.w + 2 * q.z * q.z - 1)
        : std::atan2(-(2 * q.x * q.z - 2 * q.y * q.w), 2 * q.w * q.w + 2 * q.x * q.x - 1),
      unlocked ? std::atan2(2 * q.x * q.y + 2 * q.z * q.w, 2 * q.w * q.w + 2 * q.y * q.y - 1) : 0,
      order
    );
  } else if (order == EulerOrder::YZX) {
    auto sz = 2 * q.x * q.y + 2 * q.z * q.w;
    auto unlocked = std::abs(sz) < 0.99999f;
    return EulerAngle(
      unlocked ? atan2(-(2 * q.y * q.z - 2 * q.x * q.w), 2 * q.w * q.w + 2 * q.y * q.y - 1) : 0,
      unlocked ? atan2(-(2 * q.x * q.z - 2 * q.y * q.w), 2 * q.w * q.w + 2 * q.x * q.x - 1)
        : atan2(2 * q.x * q.z + 2 * q.y * q.w, 2 * q.w * q.w + 2 * q.z * q.z - 1),
      std::asin(sz),
      order
    );
  } else if (order == EulerOrder::ZXY) {
    auto sx = 2 * q.y * q.z + 2 * q.x * q.w;
    auto unlocked = std::abs(sx) < 0.99999f;
    return EulerAngle(
      std::asin(sx),
      unlocked ? std::atan2(-(2 * q.x * q.z - 2 * q.y * q.w), 2 * q.w * q.w + 2 * q.z * q.z - 1) : 0,
      unlocked ? std::atan2(-(2 * q.x * q.y - 2 * q.z * q.w), 2 * q.w * q.w + 2 * q.y * q.y - 1)
        : std::atan2(2 * q.x * q.y + 2 * q.z * q.w, 2 * q.w * q.w + 2 * q.x * q.x - 1),
      order
    );
  } else if (order == EulerOrder::ZYX) {
    auto sy = -(2 * q.x * q.z - 2 * q.y * q.w);
    auto unlocked = std::abs(sy) < 0.99999f;
    return EulerAngle(
      unlocked ? std::atan2(2 * q.y * q.z + 2 * q.x * q.w, 2 * q.w * q.w + 2 * q.z * q.z - 1) : 0,
      std::asin(sy),
      unlocked ? std::atan2(2 * q.x * q.y + 2 * q.z * q.w, 2 * q.w * q.w + 2 * q.x * q.x - 1)
        : std::atan2(-(2 * q.x * q.y - 2 * q.z * q.w), 2 * q.w * q.w + 2 * q.y * q.y - 1),
      order
    );
  }
  throw "conversion of quaternion to euler angle is failed.";
}


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

        std::string save_data_top_path = "/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/flight_airsim_image/new_sequence1";
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
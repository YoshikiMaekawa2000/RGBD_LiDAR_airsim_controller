#include "RGBD_LiDAR_airsim_controller/get_lidar_rgbd_image.h"

GetLidarRGBDImage::GetLidarRGBDImage(){
    std::cout << "Get Lidar RGBD Image" << std::endl;
    client_initialization();

    _list_camera = {
		"camera_0",
        "camera_1"
	};
}

GetLidarRGBDImage::~GetLidarRGBDImage(){
    std::cout << "End Get Lidar RGBD Image" << std::endl;
}

void GetLidarRGBDImage::client_initialization(){
    //connect
    _client.confirmConnection(); //msr::airlib::RpcLibClientBase's function

    //reset
    std::cout << "Reset" << std::endl;
    _client.reset();

    //pose
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    update_state();

    /*weather*/
	if(_random_weather)	_client.simEnableWeather(true);
	_list_weather = {
		msr::airlib::WorldSimApiBase::WeatherParameter::Rain,
		msr::airlib::WorldSimApiBase::WeatherParameter::Roadwetness,
		msr::airlib::WorldSimApiBase::WeatherParameter::Snow,
		msr::airlib::WorldSimApiBase::WeatherParameter::RoadSnow,
		msr::airlib::WorldSimApiBase::WeatherParameter::MapleLeaf,
		msr::airlib::WorldSimApiBase::WeatherParameter::RoadLeaf,
		msr::airlib::WorldSimApiBase::WeatherParameter::Dust,
		msr::airlib::WorldSimApiBase::WeatherParameter::Fog
		// msr::airlib::WorldSimApiBase::WeatherParameter::Enabled
	};
}

void GetLidarRGBDImage::update_state(){
    /*pose*/
	_pose = _client.simGetVehiclePose();
	std::cout << "Pose: " << std::endl;
	std::cout << " Position: "	//Eigen::Vector3f
		<< _pose.position.x() << ", "
		<< _pose.position.y() << ", "
		<< _pose.position.z() << std::endl;
	std::cout << " Orientation: "	//Eigen::Quaternionf
		<< _pose.orientation.w() << ", "
		<< _pose.orientation.x() << ", "
		<< _pose.orientation.y() << ", "
		<< _pose.orientation.z() << std::endl;
	/*imu*/
	_imu = _client.getImuData();
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< _imu.linear_acceleration.x() << ", "
		<< _imu.linear_acceleration.y() << ", "
		<< _imu.linear_acceleration.z() << std::endl;
}

void GetLidarRGBDImage::spin(){
    bool csv_checker = load_csv();
    if(!csv_checker){
        printf("ERROR LOADING CSV PLACE DATA FILE\n");
        exit(1);
    }

    save_data();
}

bool GetLidarRGBDImage::load_csv(){
    bool checker = true;
    std::string place_csv_path = place_csv_root_path + place_csv_name;

    std::ifstream csv_file(place_csv_path);
    if(!csv_file){
        std::cout << "CSV File " << place_csv_path << " not found. Exit..." << std::endl;
        checker = false;
    }
    else{
        std::string line;
        while( getline(csv_file, line)){
            std::vector<std::string> tmp_data = split(line, ',');
            csv_data.push_back(tmp_data);
        }

        csv_data_size = csv_data.size();
    }
    return checker;
}

int GetLidarRGBDImage::random_int(int min, int max){
    int num = 0;
    std::mt19937 mt{ std::random_device{}() };
    std::uniform_int_distribution<int> dist(min, max);

    num = dist(mt);
    return num;
}

std::vector<float> GetLidarRGBDImage::string_to_float(std::vector<std::string> tmp_place_csv_data){
    std::vector<float> farray;

    for(size_t i=0; i < tmp_place_csv_data.size(); ++i){
        std::string tmp_string = tmp_place_csv_data[i];
        float tmp_f = std::stof(tmp_string);

        farray.push_back(tmp_f);
    }

    return farray;
}

std::vector<std::string> GetLidarRGBDImage::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

float GetLidarRGBDImage::check_deg(float deg){
    float result = 0;
    if(deg > M_PI){
        float tmp = deg - M_PI;
        result = -1.0 * M_PI + tmp;
        std::cout << "upper" << std::endl;
    }
    else if( deg < -1.0 * M_PI){
        float tmp = deg + M_PI;
        result = M_PI + tmp;
        std::cout << "lower" << std::endl;
    }
    else{
        result = deg;
    }

    return result;
}

float GetLidarRGBDImage::convert_angle(float rad){
    float eular = (rad/M_PI) * 180.0;
    float int_eular = int(eular);
    float return_rad = int_eular/180.0 *M_PI;
    /*
    std::cout << eular << std::endl;
    std::cout << int_eular << std::endl;
    std::cout << return_rad << std::endl;
    */
    return return_rad;
}

void GetLidarRGBDImage::eular_to_quat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}

cv::Mat GetLidarRGBDImage::get_image(bool save_color_checker){
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size()-1);

    for(size_t i=0; i<_list_camera.size()-1; ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[0], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}

    std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
    
    cv::Mat img_cv = cv::Mat(list_response[0].height, list_response[0].width, CV_8UC3);
    for(int row=0; row<list_response[0].height; ++row){
		for(int col=0; col<list_response[0].width; ++col){
			img_cv.at<cv::Vec3b>(row, col)[0] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 0];
			img_cv.at<cv::Vec3b>(row, col)[1] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 1];
			img_cv.at<cv::Vec3b>(row, col)[2] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 2];
		}
    }

    cv::Mat return_img;

    if( save_color_checker == false){
        cvtColor(img_cv, return_img, cv::COLOR_BGR2GRAY);
    }
    else{
        return_img = img_cv;
    }

    return return_img;
}

cv::Mat GetLidarRGBDImage::get_depth_image(){
    std::cout << "List Request" << std::endl;
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size()-1);

    std::cout << "Image Request" << std::endl;
	for(size_t i=0; i<_list_camera.size()-1; ++i){
		list_request[0] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[1], msr::airlib::ImageCaptureBase::ImageType::DepthPlanar, true);
	}

    std::cout << "get Image" << std::endl;
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);

    std::cout << "Access pixel" << std::endl;
    //cv::Mat img_cv(list_response[0].height, list_response[0].width, CV_8UC1, list_response[0].image_data_float.data());

    std::cout << "Response" << std::endl;
    //std::cout << list_response[0].image_data_uint8.size() << std::endl;
    //std::cout << list_response[0].image_data_float.size() << std::endl;

    /*
    for(size_t i=0; i <list_response[0].image_data_float.size(); i++){
        std::cout << list_response[0].image_data_float[i] << std::endl;
    }*/

    //std::cout << list_response[0].image_data_uint8 << std::endl;
    
    
    cv::Mat img_cv(list_response[0].height, list_response[0].width, CV_32F, (void*)list_response[0].image_data_float.data());
    
    //cv::imshow("image",img_cv);
    //cv::waitKey(0);
    
    //cv::normalize(img_cv, img_cv, cv::NORM_MINMAX);
    

    /*
    for(int row=0; row<list_response[0].height; ++row){
		for(int col=0; col<list_response[0].width; ++col){
			img_cv.at<cv::Vec3b>(row, col)[0] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 0];
			img_cv.at<cv::Vec3b>(row, col)[1] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 1];
			img_cv.at<cv::Vec3b>(row, col)[2] = list_response[0].image_data_uint8[3*row*list_response[0].width + 3*col + 2];
		}
    }
    */
    return img_cv;
}

void GetLidarRGBDImage::input_data_to_pointcloud(msr::airlib::LidarData lidar_data)
{
	/*input*/
	for(size_t i=0; i<lidar_data.point_cloud.size(); i+=3){
		/*
        std::cout
			<< "i = " << i << "~" << i+2 << ": "
			"x = " << lidar_data.point_cloud[i] << ", "
			"y = " << lidar_data.point_cloud[i+1] << ", "
			"z = " << lidar_data.point_cloud[i+2] << std::endl;
        */

		pcl::PointXYZ tmp;
		tmp.x = lidar_data.point_cloud[i];
		tmp.y = lidar_data.point_cloud[i+1];
		tmp.z = lidar_data.point_cloud[i+2];
		_pc->points.push_back(tmp);
	}
	/*width x height*/
	_pc->width = _pc->points.size();
	_pc->height = 1;
	/*print*/
	//std::cout << "_pc->points.size() = " << _pc->points.size() << std::endl;

    //std::cout << "NED -> NEU" << std::endl;
	for(size_t i=0; i<_pc->points.size(); ++i){
		_pc->points[i].y *= -1;
		_pc->points[i].z *= -1;
	}
}

std::string GetLidarRGBDImage::save_camera_image(cv::Mat camera_image, int num){
    std::string filename = "image" + std::to_string(num) + ".png";
    std::string save_path = save_data_top_path + rgb_image_directory + "/" + filename;

    cv::imwrite(save_path, camera_image);

    return filename;
}

std::string GetLidarRGBDImage::save_depth_image(cv::Mat depth_image, int num){
    std::string filename = "depth" + std::to_string(num) + ".png";
    std::string save_path = save_data_top_path + depth_image_directory + "/" + filename;

    //cv::imshow("Depth", depth_image);
    //cv::waitKey(0);

    cv::imwrite(save_path, depth_image);

    return filename;
}

std::string GetLidarRGBDImage::save_lidar_data(int num){
    std::string filename = "scan" + std::to_string(num) + ".pcd";
    std::string save_path = save_data_top_path + lidar_data_directory + "/" + filename;
    pcl::io::savePCDFileASCII(save_path, *_pc);

    return filename;
}

void GetLidarRGBDImage::save_csv(std::string camera_image_file_name, std::string depth_image_file_name, float x, float y, float z, float roll, float pitch, float yaw)
{
    //Save CSV
    std::string csv_path = save_data_top_path + "/" + save_csv_file_name;
    std::ofstream final_csvfile(csv_path, std::ios::app); //ios::app で追記モードで開ける

    std::string tmp_x = std::to_string(x);
    std::string tmp_y = std::to_string(y);
    std::string tmp_z = std::to_string(z);
    
    std::string tmp_roll = std::to_string(roll);
    std::string tmp_pitch = std::to_string(pitch);
    std::string tmp_yaw = std::to_string(yaw);

    final_csvfile << camera_image_file_name << ","
        << depth_image_file_name << ","
        << tmp_x << ","
        << tmp_y << ","
        << tmp_z << ","
        << tmp_roll << ","
        << tmp_pitch << ","
        << tmp_yaw <<  ","
        << tmp_roll << ","
        << _imu.linear_acceleration.x() << "," 
		<< -_imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z() << std::endl;

    final_csvfile.close();
}

void GetLidarRGBDImage::save_data(){
    int num = 0;
    std::cout << "Saving data..." << std::endl;

    std::random_device rd;
    std::mt19937 mt(rd());

    std::cout << "CSV Data Size: " << csv_data_size << std::endl;

    while(num < num_total){
        std::cout << "random int" << std::endl;
        int place_index = random_int(0, csv_data_size-1);
        std::cout << "place_index: " << place_index << std::endl;

        std::cout << "access csv" << std::endl;
        std::vector<std::string> tmp_place_csv_data = csv_data[place_index];
        std::vector<float> place_data = string_to_float(tmp_place_csv_data);

        std::cout << "random place and attitude" << std::endl;
        std::uniform_real_distribution<float> urd_roll(_roll_min , _roll_max);
        std::uniform_real_distribution<float> urd_pitch( _pitch_min, _pitch_max);
        std::uniform_real_distribution<float> urd_yaw( place_data[5] + _yaw_min, place_data[5] + _yaw_max);

        Eigen::Vector3f position(place_data[0], place_data[1], place_data[2]); //x y z

        float roll = 0.0;
        float pitch = 0.0;
        float yaw = 0.0;
        
        if(check_1_deg_increment == false){
            roll = check_deg( urd_roll(mt) );
            pitch = check_deg( urd_pitch(mt) );
            yaw = check_deg( urd_yaw(mt) );
        }
        else{
            roll = check_deg( urd_roll(mt) );
            pitch = check_deg( urd_pitch(mt) );
            yaw = check_deg( urd_yaw(mt) );

            roll = convert_angle(roll);
            pitch = convert_angle(pitch);
            yaw = convert_angle(yaw);
        }

        Eigen::Quaternionf orientation;
        eular_to_quat(roll, pitch, yaw, orientation);

        //std::cout << position[0] << position[1] << position[2] << std::endl;

        msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);

        std::cout << "Set place" << std::endl;
         _client.simSetVehiclePose(goal, true);
	    std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));
        _client.simPause(true);

        _imu = _client.getImuData();

        //Get Camera Image
        std::cout << "get camera image" << std::endl;
        cv::Mat camera_image = get_image(save_color_checker);
        
        //Get Depth Data
        std::cout << "get depth image" << std::endl;
        cv::Mat depth_image = get_depth_image();

        //Get LiDAR Data
        /*
        std::cout << "get lidar data" << std::endl;
        msr::airlib::LidarData lidar_data = _client.getLidarData("");
        input_data_to_pointcloud(lidar_data);
        */

        //Save Camera Image
        std::cout << "save camera image" << std::endl;
        std::string camera_image_file_name = save_camera_image(camera_image, num);

        //Save depth Image
        std::cout << "save depth image" << std::endl;
        std::string depth_image_file_name = save_depth_image(depth_image, num);

        //Save LiDAR Scan
        /*
        std::cout << "save lidar scan" << std::endl;
        std::string lidar_scan_file_name = save_lidar_data(num);
        */

        //Save List as CSV file
        std::cout << "save csv" << std::endl;
        save_csv(camera_image_file_name, depth_image_file_name,
                    place_data[0], place_data[1], place_data[2],
                    roll, pitch, yaw);

        num += 1;

    }

}

int main(){
    GetLidarRGBDImage get_lidar_rgbd_image;

    get_lidar_rgbd_image.spin();

    return 0;
}
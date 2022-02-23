#include "RGBD_LiDAR_airsim_controller/get_clipped_rgbd_image.h"

GetLidarRGBDImage::GetLidarRGBDImage(msr::airlib::MultirotorRpcLibClient &_client){
    std::cout << "Get Lidar RGBD Image" << std::endl;
    client_initialization(_client);

    _list_camera = {
		"camera_rgb",
        "camera_depth",
	};
}

GetLidarRGBDImage::~GetLidarRGBDImage(){
    std::cout << "End Get Lidar RGBD Image" << std::endl;
}

void GetLidarRGBDImage::client_initialization(msr::airlib::MultirotorRpcLibClient &_client){
    //connect
    _client.confirmConnection(); //msr::airlib::MultirotorRpcLibClient's function

    //reset
    std::cout << "Reset" << std::endl;
    _client.reset();

    //pose
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    update_state(_client);

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

void GetLidarRGBDImage::update_state(msr::airlib::MultirotorRpcLibClient &_client){
    Eigen::Quaternionf init_quat(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3f init_position(0.0, 0.0, 1.0);
    msr::airlib::Pose goal = msr::airlib::Pose(init_position, init_quat);
    _client.simSetVehiclePose(goal, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));
    
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

void GetLidarRGBDImage::spin(msr::airlib::MultirotorRpcLibClient &_client){
    bool csv_checker = load_csv();
    if(!csv_checker){
        printf("ERROR LOADING CSV PLACE DATA FILE\n");
        exit(1);
    }

    save_data(_client);
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

int GetLidarRGBDImage::random_int(int min, int max){
    int num = 0;
    std::mt19937 mt{ std::random_device{}() };
    std::uniform_int_distribution<int> dist(min, max);

    num = dist(mt);
    return num;
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


cv::Mat GetLidarRGBDImage::get_image(msr::airlib::MultirotorRpcLibClient &_client, bool save_color_checker){
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

cv::Mat GetLidarRGBDImage::get_depth_image(msr::airlib::MultirotorRpcLibClient &_client){

    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size()-1);
	for(size_t i=0; i<_list_camera.size()-1; ++i){
		list_request[0] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[1], msr::airlib::ImageCaptureBase::ImageType::DepthVis, true);
	}

    std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
    
    
    cv::Mat img_cv(list_response[0].height, list_response[0].width, CV_32F, (void*)list_response[0].image_data_float.data());
    cv::Mat return_cv;
    img_cv.convertTo(return_cv, CV_8U, 255.0);

    return return_cv;
}

std::vector< std::vector<int> > GetLidarRGBDImage::get_clip_place(cv::Mat image){
    std::vector< std::vector<int> > result_array;

    int row_max = image.rows - size_image;
    int col_max = image.cols - size_image;

    std::random_device rd2;
    std::mt19937 mt2(rd2());

    std::uniform_real_distribution<> urd_row( 0, row_max);
    std::uniform_real_distribution<> urd_col( 0, col_max);

    for(int i=0; i<clip_num; i++){
        int row_value = urd_row(mt2);
        int col_value = urd_col(mt2);

        std::vector<int> tmp_vector{row_value, col_value};
        result_array.push_back(tmp_vector);
    }

    return result_array;
}

std::vector<cv::Mat> GetLidarRGBDImage::get_clipped_camera_images(std::vector<std::vector<int> > clip_place, cv::Mat camera_image){
    std::vector<cv::Mat> result;
    for(int i=0; i<clip_num; i++){
        std::vector<int> tmp_vector = clip_place[i];
        cv::Mat window = cv::Mat(camera_image, cv::Rect(tmp_vector[1], tmp_vector[0], size_image, size_image));

        result.push_back(window);
    }

    return result;
}

std::vector<std::string> GetLidarRGBDImage::save_camera_images(std::vector<cv::Mat> camera_images, int num){
    std::vector<std::string> filenames;
    for(int i=0; i<clip_num; i++){
        std::string filename = "image" + std::to_string(num) + "_" + std::to_string(i) + ".png";
        std::string save_path = save_data_top_path + rgb_image_directory + "/" + filename;

        cv::imwrite(save_path, camera_images[i]);
        filenames.push_back(filename);
    }

    return filenames;
}

std::vector<std::string> GetLidarRGBDImage::save_depth_images(std::vector<cv::Mat> depth_images, int num){
    std::vector<std::string> filenames;
    for(int i=0; i<clip_num; i++){
        std::string filename = "depth" + std::to_string(num) + "_" + std::to_string(i) + ".png";
        std::string save_path = save_data_top_path + depth_image_directory + "/" + filename;

        cv::imwrite(save_path, depth_images[i]);
        filenames.push_back(filename);
    }

    return filenames;
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
        << _imu.linear_acceleration.x() << "," 
		<< -_imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z() << std::endl;

    final_csvfile.close();
}


void GetLidarRGBDImage::save_data(msr::airlib::MultirotorRpcLibClient &_client){
    int num = 0;
    std::cout << "Start Saving Data" << std::endl;

    std::random_device rd;
    std::mt19937 mt(rd());

    std::cout << "CSV Data Size: "  << std::endl;

    while(num < num_total){
        std::cout << "Sampling: " << num << std::endl;
        int place_index = random_int(0, csv_data.size()-1);

        //std::cout << "Choosen Index: " << place_index << std::endl;

        //Set Place
        std::vector<std::string> tmp_place_csv_data = csv_data[place_index];
        std::vector<float> place_data = string_to_float(tmp_place_csv_data);

        /*
        std::cout << "Place Data: " << std::endl
        << place_data[0] << std::endl
        << place_data[1] << std::endl
        << place_data[2] << std::endl
        << place_data[3] << std::endl
        << place_data[4] << std::endl
        << place_data[5] << std::endl
        << place_data[6] << std::endl << std::endl;
        */


        float qx = place_data[3];
        float qy = place_data[4];
        float qz = place_data[5];
        float qw = place_data[6];

        Eigen::Quaternionf tmp_q(qw, qx, qy, qz);
        Eigen::Vector3f euler = tmp_q.toRotationMatrix().eulerAngles(0, 1, 2);

        //std::cout << "Yaw: " << euler[2] << std::endl;

        std::uniform_real_distribution<float> urd_roll(_roll_min , _roll_max);
        std::uniform_real_distribution<float> urd_pitch( _pitch_min, _pitch_max);
        std::uniform_real_distribution<float> urd_yaw( euler[2] + _yaw_min, euler[2] + _yaw_max);

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

        //std::cout << "Yaw2: " << yaw << std::endl;

        msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);
        _client.simSetVehiclePose(goal, true);
	    std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));
        _client.simPause(true);
        _pose = _client.simGetVehiclePose();

        double drone_roll, drone_pitch, drone_yaw;
        Eigen::Quaterniond tmp_quat(_pose.orientation.w(),
                                    _pose.orientation.x(),
                                    _pose.orientation.y(),
                                    _pose.orientation.z() );
        
        Eigen::Vector3d euler_d = tmp_quat.toRotationMatrix().eulerAngles(2, 1, 0);
        drone_yaw = euler_d[0]; 
        drone_pitch = euler_d[1]; 
        drone_roll = euler_d[2];

        std::cout << "Pose of Data: " << std::endl;
	    std::cout << " Position: "	//Eigen::Vector3f
		<< place_data[0] << ", "
		<< place_data[1] << ", "
		<< place_data[2] << std::endl;
	    std::cout << " Orientation: "	//Eigen::Quaternionf
		<< roll/M_PI*180.0 << ", "
		<< pitch/M_PI*180.0 << ", "
        << yaw/M_PI*180.0 << std::endl << std::endl;

        std::cout << "Pose of Drone: " << std::endl;
	    std::cout << " Position: "	//Eigen::Vector3f
		<< _pose.position.x() << ", "
		<< _pose.position.y() << ", "
		<< _pose.position.z() << std::endl;
	    std::cout << " Orientation: "	//Eigen::Quaternionf
		<< drone_roll/M_PI*180.0 << ", "
		<< drone_pitch/M_PI*180.0 << ", "
		<< drone_yaw/M_PI*180.0 << std::endl << std::endl;

        std::cout << "------------------" << std::endl;


        //Get Original Camera Image
        cv::Mat camera_image = get_image(_client, save_color_checker);
        
        //Get Original Depth Data
        cv::Mat depth_image = get_depth_image(_client);

        std::vector< std::vector<int> > clip_place;

        if(camera_image.rows==depth_image.rows && camera_image.cols==depth_image.cols){
            clip_place = get_clip_place(camera_image);
        }
        else{
            std::cout << "Invalid Image Size. Exit..." << std::endl;
            exit(1);
        }

        std::vector<cv::Mat> clipped_camera_images = get_clipped_camera_images(clip_place, camera_image);
        std::vector<cv::Mat> clipped_depth_images = get_clipped_camera_images(clip_place, depth_image);

        //Save Camera Image
        std::vector<std::string> camera_image_file_names = save_camera_images(clipped_camera_images, num);

        //Save Camera Image
        std::vector<std::string> depth_image_file_names = save_depth_images(clipped_depth_images, num);

        for(int i=0; i<clip_num; i++){
            save_csv(camera_image_file_names[i], depth_image_file_names[i],
                    place_data[0], place_data[1], place_data[2],
                    roll, pitch, yaw);
        }

        num += 1;
        _client.simPause(false);
    }
}

int main(){
    msr::airlib::MultirotorRpcLibClient _client;
    GetLidarRGBDImage get_clipped_rgbd_image(_client);
    get_clipped_rgbd_image.spin(_client);

    return 0;
}
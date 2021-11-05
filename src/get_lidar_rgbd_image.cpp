#include "RGBD_LiDAR_airsim_controller/get_lidar_rgbd_image.h"

GetLidarRGBDImage::GetLidarRGBDImage(){
    std::cout << "Get Lidar RGBD Image" << std::endl;
    client_initialization();
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

void GetLidarRGBDImage::save_data(){
    int num = 0;
    std::cout << "Saving data..." << std::endl;

    std::random_device rd;
    std::mt19937 mt(rd());

    std::cout << "CSV Dat Size: " << csv_data_size << std::endl;

    while(num < num_total){
        int place_index = random_int(0, csv_data_size-1);
        std::vector<std::string> tmp_place_csv_data = csv_data[place_index];

        std::vector<float> place_data = string_to_float(tmp_place_csv_data);

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
        msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);

         _client.simSetVehiclePose(goal, true);
	    std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));
        _client.simPause(true);

        _imu = _client.getImuData();


    }

}

int main(){
    GetLidarRGBDImage get_lidar_rgbd_image;

    get_lidar_rgbd_image.spin();

    return 0;
}
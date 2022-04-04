#include "RGBD_LiDAR_airsim_controller/drone_flight.h"

DroneFlight::DroneFlight(){
    std::cout << "Drone Flight" << std::endl;

    client_initialization();

    /*camera list*/
	_list_camera = {
		"camera_rgb",
        "camera_depth",
	};

    load_csv();
    std::cout << "Initialization Done" << std::endl;
}


void DroneFlight::client_initialization(){
    //connect
    _client.confirmConnection();

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
	/*time*/
	// _client.simSetTimeOfDay(true, "2020-01-01 17:00:00", false, 0.01, 1.0, true);
}


void DroneFlight::update_state(){
    /*pose*/
	_pose = _client.simGetVehiclePose();

    /*
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
    */

	/*imu*/
	_imu = _client.getImuData();

    /*
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< _imu.linear_acceleration.x() << ", "
		<< _imu.linear_acceleration.y() << ", "
		<< _imu.linear_acceleration.z() << std::endl;
    */
}


std::vector<std::string> DroneFlight::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

bool DroneFlight::load_csv(){
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

void DroneFlight::set_drone_random_point(){
    int place_index = random_int(0, csv_data_size - 1);
    
    std::vector<std::string> tmp_place_csv_data = csv_data[place_index];
    std::vector<float> place_data = string_to_float(tmp_place_csv_data);

    float qx = place_data[3];
        float qy = place_data[4];
        float qz = place_data[5];
        float qw = place_data[6];

        Eigen::Quaternionf tmp_q(qw, qx, qy, qz);
        Eigen::Vector3f euler = tmp_q.toRotationMatrix().eulerAngles(0, 1, 2);

        //std::cout << "Yaw: " << euler[2] << std::endl;

        //std::cout << "random place and attitude" << std::endl;
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

        msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);

        //std::cout << "Set place" << std::endl;
         _client.simSetVehiclePose(goal, true);
	    std::this_thread::sleep_for(std::chrono::milliseconds(_wait_time_millisec));


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
}

void DroneFlight::spin(){
    std::cout << "Start Random Flight" << std::endl;
    
    int count_num = 0;

    std::random_device rd;
    std::mt19937 mt(rd());

    while(count_num < max_sequence){
        std::cout << "Select Drone Point" << std::endl;
        set_drone_random_point();



        count_num += 1;
    }
}

int main(int argc, char **argv){
    DroneFlight drone_flight;
    drone_flight.spin();

    return 0;
}
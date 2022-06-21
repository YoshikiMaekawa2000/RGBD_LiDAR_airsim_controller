#include "RGBD_LiDAR_airsim_controller/waypoint_flight.h"

WaypointFlight::WaypointFlight(){
	std::cout << "----- drone_waypoint_flight_withnoise -----" << std::endl;
    _list_camera = {
		"camera_rgb",
        "camera_depth",
	};
	/*initialize*/
	setWayPoints();
	devidePath();
	// addNoise();
	// printPath();
	clientInitialization();
}

WaypointFlight::~WaypointFlight(){
    std::cout << "End WaypointFlight" << std::endl;
}

void WaypointFlight::setWayPoints(){
    std::cout << "Load CSV" << std::endl;
    bool csv_checker = load_csv();

    //Set Waypoint
    _waypoints.push_back(Eigen::Vector3f(target_points[0].x, target_points[0].y, _height));

    int waypoint_idx = 0;

    std::cout << "Waypoint number: " << 1 << std::endl;
        std::cout << "waypoint_index: " << waypoint_idx << std::endl;
        std::cout << "Waypoint: " << "x: " <<  target_points[waypoint_idx].x << " y: " << target_points[waypoint_idx].y << std::endl << std::endl;

    for(int i=0; i<num_target_points-1; i++){
        waypoint_idx = chooseWaypoint(waypoint_idx, target_points);

        waypoint target_waypoint = create_slide_waypoint(target_points[waypoint_idx]);
        
        //_waypoints.push_back(Eigen::Vector3f(target_points[waypoint_idx].x, target_points[waypoint_idx].y, _height));
        
        _waypoints.push_back(Eigen::Vector3f( target_waypoint.x, target_waypoint.y, _height));
        
        std::cout << "Waypoint number: " << i+2 << std::endl;
        std::cout << "waypoint_index: " << waypoint_idx << std::endl;
        std::cout << "Waypoint: " << "x: " <<  target_waypoint.x << " y: " << target_waypoint.y << std::endl << std::endl;
    }
}

waypoint WaypointFlight::create_slide_waypoint(waypoint selected_waypoint){
    waypoint return_waypoint;

    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    
    std::normal_distribution<float> dist_x(selected_waypoint.x, standard_deviation);
    std::normal_distribution<float> dist_y(selected_waypoint.y, standard_deviation);

    return_waypoint.x = dist_x(engine);
    return_waypoint.y = dist_y(engine);

    return_waypoint.self_idx = selected_waypoint.self_idx;
    //return_waypoint.idx = selected_waypoint.idx;

    return return_waypoint;
}

int WaypointFlight::chooseWaypoint(int waypoint_idx, std::vector<waypoint> target_points){
    int next_idx = -1;
    while(1){
        int id = random_int(0, 3);
        //std::cout << "id: " << id << std::endl;
        next_idx = target_points[waypoint_idx].idx[id];
        //std::cout << "next_idx: " << next_idx << std::endl;
        
        if(next_idx==before_idx || next_idx== -1){
            next_idx = -1;
        }
        else{
            before_idx = waypoint_idx;
            break;
        }
    }

    return next_idx;
}

int WaypointFlight::random_int(int min, int max){
    int num = 0;
    std::mt19937 mt{ std::random_device{}() };
    std::uniform_int_distribution<int> dist(min, max);

    num = dist(mt);
    return num;
}

bool WaypointFlight::load_csv(){
    bool checker = true;
    std::ifstream csv_file(waypoint_file);
    if(!csv_file){
        std::cout << "CSV File " << waypoint_file << " not found. Exit..." << std::endl;
        checker = false;
    }
    else{
        std::string line;
        while( getline(csv_file, line)){
            std::vector<std::string> tmp_data = split(line, ',');
            csv_data.push_back(tmp_data);
            //std::vector<float> tmp_data_float = string_to_float(tmp_data);
            //waypoint tmp_waypoint = create_waypoint(tmp_data_float);
            //target_points.push_back(tmp_waypoint);
        }

        csv_data_size = csv_data.size();

        for(size_t i = 0; i<csv_data_size; i++){
            std::vector<float> tmp_data_float = string_to_float(csv_data[i]);
            waypoint tmp_waypoint = create_waypoint(tmp_data_float);
            target_points.push_back(tmp_waypoint);
        }
    }
    return checker;
}

std::vector<std::string> WaypointFlight::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

std::vector<float> WaypointFlight::string_to_float(std::vector<std::string> string_data){
    std::vector<float> farray;

    for(size_t i=0; i < string_data.size(); ++i){
        try{
            std::string tmp_string = string_data[i];
            float tmp_f = std::stof(tmp_string);

            farray.push_back(tmp_f);
        }
        catch(const std::invalid_argument& e){
            cout << "invalid argument" << endl;
            exit(1);
        }
        catch(const std::out_of_range& e){
            cout << "Out of range" <<endl;
            exit(1);
        }
    }

    return farray;
}

waypoint WaypointFlight::create_waypoint(std::vector<float> farray){
    waypoint result;
    result.self_idx = int(farray[0]);

    result.x = farray[1];
    result.y = farray[2];

    result.idx[0] = int(farray[3]);
    result.idx[1] = int(farray[4]);
    result.idx[2] = int(farray[5]);
    result.idx[3] = int(farray[6]);

    return result;
}


void WaypointFlight::devidePath(void)
{
	for(size_t i=0; i<_waypoints.size()-1; ++i){
		Eigen::Vector3f delta = _waypoints[i+1] - _waypoints[i];
		int points_per_line = int(delta.norm()/_path_resolution);
		if(points_per_line > 0){
			Eigen::Vector3f step = delta/(double)points_per_line;
			for(size_t j=1; j<points_per_line; ++j){
				Eigen::Vector3f point = _waypoints[i] + j*step;
				addNoise(point);
				double dist_to_next = (_waypoints[i+1] - point).norm();
				if(dist_to_next > _cutting_corner)	_path.push_back(point);
			}
		}
		_path.push_back(_waypoints[i+1]);
		_path[_path.size()-1] -= (_waypoints[i+1] - _waypoints[i]).normalized()*_cutting_corner;
	}
}

void WaypointFlight::printPath(void)
{
	for(size_t i=0; i<_path.size(); ++i){
		std::cout << i << ": ("
			<< _path[i](0) << ", "
			<< _path[i](1) << ", "
			<< _path[i](2) << ")"
		<< std::endl;
	}
	std::cout << "_path.size() = " << _path.size() << std::endl;
}

void WaypointFlight::clientInitialization(void)
{
	/*connect*/
	_client.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	_client.reset();
	/*control*/
	std::cout << "Enable API control" << std::endl;
	_client.enableApiControl(true);
	std::cout << "Arm the drone" << std::endl;
	_client.armDisarm(true);
	std::cout << "Take off" << std::endl;
	_client.takeoffAsync()->waitOnLastTask();
}

void WaypointFlight::addNoise(Eigen::Vector3f& point)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_xy(-_noise_xy, _noise_xy);
	std::uniform_real_distribution<> urd_z(-_noise_z, _noise_z);

	point(0) += urd_xy(mt);
	point(1) += urd_xy(mt);
	point(2) += urd_z(mt);
}

void WaypointFlight::startFlight(void)
{
	std::cout << "startFlight" << std::endl;

	_client.moveOnPathAsync(
		_path,
		_velocity,
		Utils::max<float>(),
		DrivetrainType::ForwardOnly,
		YawMode(false, 0.0)
		//-1,
		//0
	)->waitOnLastTask();

    std::cout << "Cruise Flight has done" << std::endl;
    end_checker = true;

	std::cout << "Go home" << std::endl;
	_client.goHomeAsync()->waitOnLastTask();
	//std::cout << "Land" << std::endl;
	//_client.landAsync()->waitOnLastTask();
}

void WaypointFlight::collectData(void)
{
    sleep(3); //Wait 4 seconds
    std::cout << "Start CollectData" << std::endl;

    int num_count = 0;
    clock_t start_time = clock();

    while(end_checker==false){
        clock_t process_start_time = clock();

        //Get Data
        cv::Mat camera_image = get_image();
        msr::airlib::Pose _pose = getPose();

        //Convert Data
        float drone_roll, drone_pitch, drone_yaw;
        Eigen::Quaterniond tmp_quat(_pose.orientation.w(),
                                    _pose.orientation.x(),
                                    _pose.orientation.y(),
                                    _pose.orientation.z() );
        
        /*
        Eigen::Vector3d euler_d = tmp_quat.toRotationMatrix().eulerAngles(2, 1, 0);
        drone_yaw = euler_d[0]; 
        drone_pitch = euler_d[1]; 
        drone_roll = euler_d[2];
        */

        Quaternion q_drone(_pose.orientation.x(),_pose.orientation.y(),_pose.orientation.z(),_pose.orientation.w());

        EulerAngle drone_eular = toEulerAngle(q_drone, EulerOrder::XYZ);

        drone_roll = drone_eular.x;
        drone_pitch = drone_eular.y;
        drone_yaw = drone_eular.z;

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

        std::string camera_image_file_name = save_camera_image(camera_image, num_count);

        clock_t end_time = clock();
        float duration = (float)(end_time - process_start_time) / CLOCKS_PER_SEC;
        float process_time = (float)(end_time - start_time) / CLOCKS_PER_SEC;

        save_csv(num_count, 
                process_time, 
                camera_image_file_name, 
                _pose.position.x(), 
                _pose.position.y(), 
                _pose.position.z(),
                drone_roll, 
                drone_pitch, 
                drone_yaw);

        std::cout << "Image" << num_count << ", " << "Duration: " << duration << std::endl;

        num_count += 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_seconds));
    }

}

void WaypointFlight::save_csv(int num_count, float process_time, std::string camera_image_file_name, float x, float y, float z, float roll, float pitch, float yaw){
    //Save CSV
    std::string csv_path = save_data_top_path + "/" + save_csv_file_name;
    std::ofstream final_csvfile(csv_path, std::ios::app); //ios::app で追記モードで開ける

    std::string tmp_count = std::to_string(num_count);
    std::string tmp_process_time = std::to_string(process_time);

    std::string tmp_x = std::to_string(x);
    std::string tmp_y = std::to_string(y);
    std::string tmp_z = std::to_string(z);
    
    std::string tmp_roll = std::to_string(roll);
    std::string tmp_pitch = std::to_string(pitch);
    std::string tmp_yaw = std::to_string(yaw);

    final_csvfile << tmp_count << ","
        << camera_image_file_name << ","
        << tmp_process_time << ","
        << tmp_x << ","
        << tmp_y << ","
        << tmp_z << ","
        << tmp_roll << ","
        << tmp_pitch << ","
        << tmp_yaw << std::endl;

    final_csvfile.close();
}

std::string WaypointFlight::save_camera_image(cv::Mat camera_image, int num){
    std::string filename = "image" + std::to_string(num) + ".png";
    std::string save_path = save_data_top_path + rgb_image_directory + "/" + filename;

    cv::imwrite(save_path, camera_image);

    return filename;
}

cv::Mat WaypointFlight::get_image(){
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

    int original_row = img_cv.rows;
    int original_col = img_cv.cols;

    cv::Mat clipped_image = cv::Mat(img_cv, cv::Rect( original_col/2-original_row/2, 0, original_row, original_row));

    double clipped_row = clipped_image.rows;
    double clipped_col = clipped_image.cols;

    double row_ratio = (double)(pic_size)/clipped_row;
    double col_ratio = (double)(pic_size)/clipped_col;

    cv::Mat resized_image;
    cv::resize(clipped_image, resized_image, cv::Size(), row_ratio, col_ratio);

    /*
    cv::imshow("aaa", resized_image);
    cv::waitKey(0);
    */

    return resized_image;
}

msr::airlib::Pose WaypointFlight::getPose(){
    msr::airlib::Pose _pose = _client.simGetVehiclePose();
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
    return _pose;
}

void WaypointFlight::spin(){
    std::thread thread_flight(&WaypointFlight::startFlight, this);
    std::thread thread_collection(&WaypointFlight::collectData, this);

    thread_flight.join();
    thread_collection.join();
}

int main(){
    WaypointFlight waypoint_flight;
    waypoint_flight.spin();

    return 0;
}
#include "RGBD_LiDAR_airsim_controller/waypoint_flight.h"

WaypointFlight::WaypointFlight(){
	std::cout << "----- drone_waypoint_flight_withnoise -----" << std::endl;
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
        _waypoints.push_back(Eigen::Vector3f(target_points[waypoint_idx].x, target_points[waypoint_idx].y, _height));
        std::cout << "Waypoint number: " << i+2 << std::endl;
        std::cout << "waypoint_index: " << waypoint_idx << std::endl;
        std::cout << "Waypoint: " << "x: " <<  target_points[waypoint_idx].x << " y: " << target_points[waypoint_idx].y << std::endl << std::endl;
    }
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

int main(){
    WaypointFlight waypoint_flight;


    return 0;
}
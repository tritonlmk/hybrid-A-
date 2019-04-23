#include "hybrid_a_star.h"
// construnction function of the base class AStar
// note: all derived classes should obey the interface of the base class
const double lmk_astar::AStarInterface::pi_ = 3.1416;
// construction/deconstruction function of struct VehiclePose
lmk_astar::VehiclePose::VehiclePose() {
  cost_g = total_cost = -1.0;
  x_pose = y_pose = heading_angle = -1.0;
  x_index = y_index = -1;
}
lmk_astar::VehiclePose::VehiclePose(double x, double y, double angle, double temp_cost_g, double temp_total_cost) {
  x_pose = x;
  y_pose = y;
  x_index = static_cast<int>(x);
  y_index = static_cast<int>(y);
  heading_angle = angle;
  cost_g = temp_cost_g;
  total_cost = temp_total_cost;
}
lmk_astar::VehiclePose::VehiclePose(double x, double y, double angle, double temp_cost_g) {
  x_pose = x;
  y_pose = y;
  x_index = static_cast<int>(x);
  y_index = static_cast<int>(y);
  heading_angle = angle;
  cost_g = temp_cost_g;
  total_cost = -1.0;
}
lmk_astar::VehiclePose::VehiclePose(double x, double y, double angle) {
  x_pose = x;
  y_pose = y;
  x_index = static_cast<int>(x);
  y_index = static_cast<int>(y);
  heading_angle = angle;
  cost_g = total_cost = -1.0;
}
//lmk_astar::VehiclePose::VehiclePose(const VehiclePose&) {}
// note: another way to compare a struct is to define a "comparer" outside the struct
// and then use it as the input of functions/data_structures like: sort(), priority_queue
bool lmk_astar::VehiclePose::operator < (const VehiclePose& temp_node) const {
  return (total_cost < temp_node.total_cost);
}
bool lmk_astar::VehiclePose::operator > (const VehiclePose& temp_node) const {
  return (total_cost > temp_node.total_cost);
}
bool lmk_astar::VehiclePose::operator == (const VehiclePose& temp_node) const {
  if (x_index != temp_node.x_index || y_index != temp_node.y_index) {
    return false;
  } else {return true;}
}
bool lmk_astar::VehiclePose::operator != (const VehiclePose& temp_node) const {
  if (x_index == temp_node.x_index && y_index == temp_node.y_index) {
    return false;
  } else {return true;}
}
void lmk_astar::VehiclePose::operator = (const VehiclePose& temp_node) {
  parent = temp_node.parent;
  x_pose = temp_node.x_pose;
  y_pose = temp_node.y_pose;
  x_index = temp_node.x_index;
  y_index = temp_node.y_index;
  heading_angle = temp_node.heading_angle;
  cost_g = temp_node.cost_g;
  total_cost = temp_node.total_cost;
}
// construction/deconstruction function of struct ROSMapData
lmk_astar::ROSMapData::ROSMapData():metadata_flag(false),mapdata_flag(false) {
  map_height = map_length = -1;
}
// functions in class OpenList
// binary heap
lmk_astar::OpenList::OpenList():heap_size_(0), cost_infinity_(5000.0) {};
void lmk_astar::OpenList::min_heapify(int index) {
  int left((index+1)*2),right((index+1)*2+1), smallest(index+1);
  VehiclePose swap_temp;
  while (index < heap_size_) {
    left = (index+1)*2;
    right = (index+1)*2+1;
    if (left <= heap_size_ && openlist_data_[left-1] < openlist_data_[index]) {
      smallest = left;
    } else if (right <= heap_size_ && openlist_data_[right-1] < openlist_data_[smallest-1]) {
      smallest = right;
    } else {}
    if ((index+1) != smallest) {
      swap_temp = openlist_data_[index];
      openlist_data_[index] = openlist_data_[smallest-1];
      openlist_data_[smallest-1] = swap_temp;
      index = smallest-1;
    } else {break;}
  }
};
// if -1 returned, target pose is not found
inline int lmk_astar::OpenList::find(VehiclePose search_pose) const {
  int size = openlist_data_.size();
  for (int i = 0; i < size; ++i) {
    if (search_pose == openlist_data_[i]) {
      return i;
    }
  }
  return -1;
}
void lmk_astar::OpenList::insert(VehiclePose insert_pose) {
  heap_size_ = heap_size_ + 1;
  VehiclePose temp_pose(-1.0, -1.0, -1.0, cost_infinity_, cost_infinity_);
  if (heap_size_ > openlist_data_.size()) {
    openlist_data_.push_back(temp_pose);
  } else {
    openlist_data_[heap_size_-1] = temp_pose;
  }
  decrease(heap_size_-1, insert_pose);
}
// keywords "inline" should be put here, it will not work if placed in the .h file, function statement
inline bool lmk_astar::OpenList::check_nearer(int index, VehiclePose check_pose) {
  if (index < 0 || index > (heap_size_-1)) {
    ROS_ERROR("error: invalid index, could not compare");
    return false;
  }
  if (check_pose.cost_g < openlist_data_[index].cost_g) {
    return true;
  } else {return false;}
}
inline void lmk_astar::OpenList::pop() {
  if (heap_size_ < 1) {
    ROS_ERROR("error: empty openlist, could not pop");
    return;
  }
  openlist_data_[0] = openlist_data_[heap_size_-1];
  //openlist_data_.pop_back();
  heap_size_ = heap_size_ - 1;
  min_heapify(0);
}
void lmk_astar::OpenList::decrease(int index, VehiclePose change_pose) {
  openlist_data_[index] = change_pose;
  VehiclePose temp_pose;
  while (index > 0 && openlist_data_[(index+1)/2-1] > openlist_data_[index]) {
    temp_pose = openlist_data_[index];
    openlist_data_[index] = openlist_data_[(index+1)/2-1];
    openlist_data_[(index+1)/2-1] = temp_pose;
    index = (index+1)/2-1;
  }
}
inline bool lmk_astar::OpenList::empty() {
  return ((heap_size_ > 0) ? false:true);
}
inline lmk_astar::VehiclePose lmk_astar::OpenList::top() {
  if (openlist_data_.empty()) {
    ROS_ERROR("error: empty openlist, random pose returned");
    VehiclePose temp;
    return temp;
  }
  return openlist_data_[0];
}
// construction/deconstruction function of derived class should be via vase class
void lmk_astar::HybridAStar::Init() {
#ifdef _DEBUG__
  std::cout << "Debug mode" << std::endl;
#endif
  acquire_mapdata();
  int height(map_data_.map_height), length(map_data_.map_length);
  status_lookuptable.resize(height, std::vector<int>(length, 0));
  closed_list_.resize(height, std::vector<VehiclePose>(length, VehiclePose()));
  car_parameters_ = tiguan_model_.get_parameter();
  DP_Search_.Init(std::make_shared<VehiclePose>(initial_), std::make_shared<VehiclePose>(destination_) ,std::make_shared<ROSMapData>(map_data_));
  pre_compute_heuristic_cost();
  //draw_baseimg();
}
void lmk_astar::HybridAStar::Proc() {
  Init();
  //lmk_libraries::NHCurve testdubins;
  //testdubins.DCurve(car_parameters_[3]/map_data_.resolution);
  //DP_Search_.normal_astar_search();
  ROS_INFO("Total number of physical cores is %d",boost::thread::physical_concurrency());
  get_position();
#ifdef _DEBUG__
  set_destination_initial_test();
#endif
  set_destination_initial_test();
  publish_trajectory();
}
lmk_astar::HybridAStar::~HybridAStar() {}
// public member function
// main logic
void lmk_astar::HybridAStar::publish_trajectory() {
  ros::NodeHandle hybrid_astar_nh("astar_planner");
  ros::Publisher trajectory_pub = hybrid_astar_nh.advertise<a_star::Trajectory>("/trajectory", 100);
  hybrid_astar_search();
  geometry_msgs::PointStamped Stamped_Point;
  a_star::Trajectory current_trajectory;
  int path_size = path_found_.size();
  for (int i = path_size - 1; i > 0; --i) {
    Stamped_Point.point.x = path_found_[i].x_pose;
    Stamped_Point.point.y = path_found_[i].y_pose;
    // z is the heading angle of the vehicle pose
    Stamped_Point.point.z = path_found_[i].heading_angle;
    current_trajectory.trajectory.push_back(Stamped_Point);
  }
  trajectory_pub.publish(current_trajectory);
}
void lmk_astar::HybridAStar::hybrid_astar_search() {
  ROS_INFO("start Hybrid A* planning");
  VehiclePose temp_pose(initial_.x_pose, initial_.y_pose, initial_.heading_angle, 0);
  temp_pose.total_cost = temp_pose.cost_g + heuristic_func(temp_pose);
  open_list_.insert(temp_pose);
  while (!open_list_.empty()) {
    temp_pose = open_list_.top();
    open_list_.pop();
    // inset into closed list
    status_lookuptable[temp_pose.y_index][temp_pose.x_index] = 2;
    closed_list_[temp_pose.y_index][temp_pose.x_index] = temp_pose;
    if (reach_destination(temp_pose)) {
      destination_.parent = std::make_shared<VehiclePose>(closed_list_[temp_pose.y_index][temp_pose.x_index]);
      status_lookuptable[destination_.y_index][destination_.x_index] = 2;
      closed_list_[destination_.y_index][destination_.x_index] = destination_;
      path_generator();
      draw_demo();
      return;
    } else {update_neighbour(temp_pose);}
  }
  ROS_INFO("no path found");
}
double lmk_astar::HybridAStar::heuristic_func(VehiclePose cal_pose) {
  int heuristic_length;
  if (destination_.y_pose == -1 || destination_.x_pose == -1) {
    ROS_ERROR("error: no destination set, could not calculate heuristic");
    return 0;
  } else {
    //NormalAStar DP_Search_;
    return std::sqrt(std::pow((cal_pose.x_index - destination_.x_index), 2) + std::pow((cal_pose.y_index - destination_.y_index), 2));
  }
}
void lmk_astar::HybridAStar::update_neighbour(VehiclePose& cur_pose) {
  std::vector<std::vector<double>> neighbour_nodes = motion_primitive(cur_pose);
  VehiclePose temp_pose;
  int open_index(-1);
  for (auto i : neighbour_nodes) {
    temp_pose = {i[1], i[0], i[2]};
    // check if the pose is valid: valid index and collision free
    if (i[1] < 0 || i[1] >= map_data_.map_length || i[0] < 0 || i[0] >= map_data_.map_height || !collision_detection(temp_pose)) {continue;}
    // if in closed list, do nothing
    if (status_lookuptable[temp_pose.y_index][temp_pose.x_index] == 2) {continue;}
    // update cost_g
    temp_pose.cost_g = cur_pose.cost_g + i[3];
    // calculate the heuristic distance and update the current total_cost
    temp_pose.total_cost = heuristic_func(temp_pose) + temp_pose.cost_g;
    temp_pose.parent = std::make_shared<VehiclePose>(closed_list_[cur_pose.y_index][cur_pose.x_index]);
    // if it is in a node not met yet
    if (status_lookuptable[temp_pose.y_index][temp_pose.x_index] == 0) {
      open_list_.insert(temp_pose);
      status_lookuptable[temp_pose.y_index][temp_pose.x_index] = 1;
    } else {
      open_index = open_list_.find(temp_pose);
      if (open_list_.check_nearer(open_index, temp_pose)) {open_list_.decrease(open_index, temp_pose);}
    }
  }
  return;
}
bool lmk_astar::HybridAStar::reach_destination(VehiclePose temp_pose) {
  if (destination_.x_index == -1 || destination_.y_index == -1) {
    ROS_ERROR("error: no destination set");
    return false;
  } else if (5 >= std::sqrt(std::pow((temp_pose.y_pose - destination_.y_pose), 2) + std::pow((temp_pose.x_pose - destination_.x_pose), 2))) {
    return true;
  } else {
    return false;}
}
// 5m length, 2m width
bool lmk_astar::HybridAStar::collision_detection(VehiclePose check_pose) {
  if (map_data_.map_occupancy[check_pose.y_index][check_pose.x_index] == 1) {return false;}
  double heading_angle = check_pose.heading_angle;
  double fixed_angle = std::atan2(2,5);
  double hypotenuse = std::sqrt(2*2+5*5)/2;
  double resolution = map_data_.resolution;
  int x_index, y_index;
  Eigen::ArrayXXf sixpoint_array(6, 2);
  Eigen::ArrayXXf multiple_array(6, 2);
  for (int i = 0; i < 6; ++i) {
    sixpoint_array(i, 0) = check_pose.y_index;
    sixpoint_array(i, 1) = check_pose.x_index;
  }
  multiple_array << hypotenuse*std::sin(heading_angle - fixed_angle)/resolution, hypotenuse*std::cos(heading_angle - fixed_angle)/resolution,
                    sin(heading_angle - pi_/2)/resolution, cos(heading_angle - pi_/2)/resolution,
                    hypotenuse*std::sin(heading_angle + fixed_angle - pi_)/resolution, hypotenuse*std::cos(heading_angle + fixed_angle - pi_)/resolution,
                    hypotenuse*std::sin(heading_angle - pi_ - fixed_angle)/resolution, hypotenuse*std::cos(heading_angle - pi_ - fixed_angle)/resolution,
                    std::sin(heading_angle + pi_/2)/resolution, std::cos(heading_angle + pi_/2)/resolution,
                    hypotenuse*std::sin(heading_angle + fixed_angle)/resolution, hypotenuse*std::cos(heading_angle + fixed_angle)/resolution;
  sixpoint_array = sixpoint_array + multiple_array;
  for (int j = 0; j < sixpoint_array.rows(); ++j) {
    y_index = static_cast<int>(sixpoint_array(j, 0));
    x_index = static_cast<int>(sixpoint_array(j, 1));
    if (y_index < 0 || y_index >= map_data_.map_height || x_index < 0 || x_index >= map_data_.map_length) {return false;}
    if (map_data_.map_occupancy[y_index][x_index] == 0) {return true;}
  }
  return false;
}
// private member function
// get nessary information from other ros package
// 0 for free space, 1 for obstacle
void lmk_astar::HybridAStar::acquire_mapdata() {
  ros::NodeHandle hybrid_astar_nh("astar_planner");
  int size(0);
  while(!map_data_.metadata_flag) {
    ros::ServiceClient map_meta_client = hybrid_astar_nh.serviceClient<map_proc::MapMetaP>("/map_metadata_planner");
    map_proc::MapMetaP map_meta;
    if (map_meta_client.call(map_meta)) {
      map_data_.map_length = map_meta.response.length;
      map_data_.map_height = map_meta.response.height;
      map_data_.resolution = map_meta.response.resolution;
    } else {ROS_ERROR("error: failed to call map metadata");}
    if (map_data_.map_height != -1 && map_data_.map_length != -1) {
      map_data_.metadata_flag = true;
      ROS_INFO("map data successfully received");
    } else {map_data_.metadata_flag = false;}
  }
  map_data_.map_occupancy.resize(map_data_.map_height, std::vector<int>(map_data_.map_length, 0));
  ros::ServiceClient map_client = hybrid_astar_nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap map_rawdata;
  while(!map_data_.mapdata_flag) {
    if (map_client.call(map_rawdata)) {
      size = (map_rawdata.response.map.data).size();
      map_data_.mapdata_flag = true;
      for (int i = 0; i < size; ++i) {
        int x_index = i%map_data_.map_length;
        int y_index = i/map_data_.map_length;
        if (map_rawdata.response.map.data[i] == -1) {
          map_data_.map_occupancy[y_index][x_index] = 1;
        } else if (map_rawdata.response.map.data[i] == 0) {
          map_data_.map_occupancy[y_index][x_index] = 0;
        } else if (map_rawdata.response.map.data[i] == 100) {
          map_data_.map_occupancy[y_index][x_index] = 1;
        } else {}
      }
    } else {ROS_ERROR("error: falided to receive map data");}
  }
}
void lmk_astar::HybridAStar::get_position() {
  ros::NodeHandle hybrid_astar_nh("astar_planner");
  //ros::Subscriber point_desti = hybrid_astar.subscribe("/", 100, );
}
void lmk_astar::HybridAStar::set_destination_initial_test() {
  std::vector<double> temp_config(3, 0.0);
  VehiclePose temp_pose;
  // set destination pos randomly
  temp_config = random_point(map_data_.map_height, map_data_.map_length);
  temp_pose = {temp_config[1], temp_config[0], temp_config[2]};
  while(!collision_detection(temp_pose)){
    temp_config = random_point(map_data_.map_height, map_data_.map_length);
    temp_pose = {temp_config[1], temp_config[0], temp_config[2]};
  }
  destination_ = temp_pose;
  // set initial pose randomly
  temp_config = random_point(map_data_.map_height, map_data_.map_length);
  temp_pose = {temp_config[1], temp_config[0], temp_config[2], 0};
  while (!collision_detection(temp_pose)) {
    temp_config = random_point(map_data_.map_height, map_data_.map_length);
    temp_pose = {temp_config[1], temp_config[0], temp_config[2], 0};
  }
  initial_ = temp_pose;
}
// 
std::vector<std::vector<double>> lmk_astar::HybridAStar::motion_primitive(VehiclePose root_pose) {
  std::vector<std::vector<double>> neighbour_nodes;
  VehiclePose temp_pose;
  double resolution = map_data_.resolution;
  std::vector<double> root_pose_vector = {root_pose.y_pose*resolution, root_pose.x_pose*resolution, root_pose.heading_angle};
  std::vector<double> temp_storage;
  double total_angle = car_parameters_[2];
  double temp_angle(0.0);
  for (int i = 0; i < 5; ++i) {
    temp_angle = -total_angle + (total_angle/2.0)*i;
    temp_storage = tiguan_model_.tiguan_bicycle_module(8.0, temp_angle, 0.08, root_pose_vector);
    temp_storage[0] = temp_storage[0]/resolution;
    temp_storage[1] = temp_storage[1]/resolution;
    temp_storage[3] = temp_storage[3]/resolution;
    neighbour_nodes.push_back(temp_storage);
  }
#ifdef _DEBUG__
  std::cout << "root node x is " << root_pose.x_pose << " y is " << root_pose.y_pose << " angle is " << root_pose.heading_angle << std::endl;
  for (int j = 0; j < 5; ++j) {
    temp_storage = neighbour_nodes[j];
    std::cout << "new motion x is " << temp_storage[1] << " y is " << temp_storage[0] << " angle is " << temp_storage[2] << " path length is " << temp_storage[3] << std::endl;
  }
#endif
  return neighbour_nodes;
}
void lmk_astar::HybridAStar::path_generator() {
  std::shared_ptr<VehiclePose> route_pointer;
  route_pointer = std::make_shared<VehiclePose>(closed_list_[destination_.y_index][destination_.x_index]);
  while (route_pointer->parent != nullptr) {
    path_found_.push_back(*route_pointer);
    route_pointer = route_pointer->parent;
  }
  ROS_INFO("path size is %zu ", path_found_.size());
}
// note: random heading angle range is -pi -> pi
std::vector<double> lmk_astar::HybridAStar::random_point(int pic_height, int pic_length) {
  std::vector<double> random_config(3, 0.0);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution_x_axis(0, pic_length);
  std::uniform_real_distribution<double> distribution_y_axis(0, pic_height);
  std::uniform_real_distribution<double> distribution_angle(-pi_, pi_);
  random_config[0] = distribution_y_axis(generator);
  random_config[1] = distribution_x_axis(generator);
  random_config[2] = distribution_angle(generator);
  return random_config;
}
void lmk_astar::HybridAStar::pre_compute_heuristic_cost() {
}
void lmk_astar::HybridAStar::draw_demo() {
  ROS_INFO("start drawing Hybrid A Star pic");
  cv::Mat show_img = cv::imread("/home/mingkun/lmk_ws/src/a_star/assets/map_base_img.jpg");
  cv::Point temp_point, end_point;
  std::shared_ptr<VehiclePose> route_pointer(std::make_shared<VehiclePose>(closed_list_[destination_.y_index][destination_.x_index]));
  // draw destination point and initial point
  temp_point.x = destination_.x_index;
  temp_point.y = destination_.y_index;
  end_point.x = temp_point.x + 20*std::cos(destination_.heading_angle);
  end_point.y = temp_point.y + 20*std::sin(destination_.heading_angle);
  cv::circle(show_img, temp_point, 8, cv::Scalar(0, 0, 255), 2);
  cv::line(show_img, temp_point, end_point, cv::Scalar(0, 0, 255), 2);
  temp_point.x = initial_.x_index;
  temp_point.y = initial_.y_index;
  end_point.x = temp_point.x + 20*std::cos(initial_.heading_angle);
  end_point.y = temp_point.y + 20*std::sin(initial_.heading_angle);
  cv::circle(show_img, temp_point, 8, cv::Scalar(255, 0, 0), 2);
  cv::line(show_img, temp_point, end_point, cv::Scalar(255, 0, 0), 2);
  for (int i = 0; i < map_data_.map_height; ++i) {
    for (int j = 0; j < map_data_.map_length; ++j) {
      if (status_lookuptable[i][j] == 2) {
        show_img.at<cv::Vec3b>(i,j)[0] = 0;
        show_img.at<cv::Vec3b>(i,j)[1] = 255;
        show_img.at<cv::Vec3b>(i,j)[2] = 0;
      }
    }
  }
  while (route_pointer->parent != nullptr) {
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[0] = 0;
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[1] = 0;
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[2] = 255;
    route_pointer = route_pointer->parent;
  }
  cv::imwrite("/home/mingkun/lmk_ws/src/a_star/assets/test1.jpg", show_img);
}
void lmk_astar::HybridAStar::draw_baseimg() {
  cv::Mat img_yutian = cv::Mat::zeros(map_data_.map_height, map_data_.map_length, CV_8UC3);
  for (int i = 0; i < map_data_.map_height; ++i) {
   uchar* image_pointer = img_yutian.ptr(i);
    for (int j = 0; j < map_data_.map_length; ++j) {
      uchar* pixel_pointer = image_pointer;
      if (map_data_.map_occupancy[i][j] == 1) {
        img_yutian.at<cv::Vec3b>(i, j)[0] = 255;
        img_yutian.at<cv::Vec3b>(i, j)[1] = 255;
        img_yutian.at<cv::Vec3b>(i, j)[2] = 255;
      } else if (map_data_.map_occupancy[i][j] == 0) {
        continue;
      } else {
        ROS_ERROR("error: wrong point got when drawing baseimg");
      }
    }
  }
  cv::imwrite("/home/mingkun/lmk_ws/src/a_star/assets/map_base_img_test.jpg", img_yutian);
}
// class NormalAStar
void lmk_astar::NormalAStar::Init() {}
void lmk_astar::NormalAStar::Init(std::shared_ptr<VehiclePose> desti_input, std::shared_ptr<VehiclePose> initial_input, std::shared_ptr<ROSMapData> mapdata_input) {
  destination_pointer_ = desti_input;
  initial_pointer_ = initial_input;
  mapdata_pointer_ = mapdata_input;
  status_lookuptable.resize(mapdata_pointer_->map_height, std::vector<int>(mapdata_pointer_->map_length, 0));
  closed_list_.resize(mapdata_pointer_->map_height, std::vector<VehiclePose>(mapdata_pointer_->map_length, VehiclePose()));
}
void lmk_astar::NormalAStar::Proc() {
  //normal_astar_search(initial_);
}
void lmk_astar::NormalAStar::normal_astar_search() {
  ROS_INFO("start normal astar planning");
  VehiclePose temp_pose(initial_pointer_->x_pose, initial_pointer_->y_pose, initial_pointer_->heading_angle, 0);
  // temp parameters
  temp_pose.total_cost = temp_pose.cost_g + heuristic_func(temp_pose);
  // main logic
  open_list_.insert(temp_pose);
  while (!open_list_.empty()) {
    temp_pose = open_list_.top();
    open_list_.pop();
    // insert into closed list
    status_lookuptable[temp_pose.y_index][temp_pose.x_index] = 2;
    closed_list_[temp_pose.y_index][temp_pose.x_index] = temp_pose;
    if (reach_destination(temp_pose)) {
      destination_pointer_->parent = std::make_shared<VehiclePose>(closed_list_[temp_pose.y_index][temp_pose.x_index]);
      closed_list_[destination_pointer_->y_index][destination_pointer_->x_index] = (*destination_pointer_);
      status_lookuptable[destination_pointer_->y_index][destination_pointer_->x_index] = 2;
      //path_generator(closed_list_);
      //draw_demo(initial_pointer_, std::make_shared<std::vector<std::vector<VehiclePose>>>(closed_list_));
      return;
    } else {update_neighbour(temp_pose);}
  }
  ROS_INFO("no normal a* path found");
}
// protected member function
double lmk_astar::NormalAStar::heuristic_func(VehiclePose start_grid) {
  double length = std::abs(destination_pointer_->x_index - start_grid.x_index) + std::abs(destination_pointer_->y_index - start_grid.x_index);
  return length;
}
void lmk_astar::NormalAStar::update_neighbour(VehiclePose& cur_grid) {
  std::vector<std::vector<int>> neighbour_nodes = motion_primitive(cur_grid);
  VehiclePose temp_pose;
  int open_index(0);
  for (auto i : neighbour_nodes) {
    temp_pose = {static_cast<double>(i[1]), static_cast<double>(i[0]), static_cast<double>(i[2])};
    // check if the pose is valid: valid index and collision free
    if (i[1] <0 || i[1] > (mapdata_pointer_->map_length-1) || i[0] < 0 || i[0] > (mapdata_pointer_->map_height-1) || !collision_detection(temp_pose)) {continue;}
    // if in closed list, do nothing
    if (status_lookuptable[temp_pose.y_index][temp_pose.x_index] == 2) {continue;}
    // update cost_g
    temp_pose.cost_g = cur_grid.cost_g + i[2];
    // calculate the heuristic distance and update the current total_cost
    temp_pose.total_cost = heuristic_func(temp_pose) + temp_pose.cost_g;
    temp_pose.parent = std::make_shared<VehiclePose>(closed_list_[cur_grid.y_index][cur_grid.x_index]);
    // if in openlist: check if its cost_g nearer
    if (status_lookuptable[temp_pose.y_index][temp_pose.x_index] == 1) {
      open_index = open_list_.find(temp_pose);
      if (open_list_.check_nearer(open_index, temp_pose)) {
        open_list_.decrease(open_index, temp_pose);
      }
    } else {
      open_list_.insert(temp_pose);
      status_lookuptable[temp_pose.y_index][temp_pose.x_index] = 1;
    }
    // end of loop for
  }
}
bool lmk_astar::NormalAStar::reach_destination(VehiclePose temp_grid) {
  if (std::abs(destination_pointer_->x_index - temp_grid.x_index) < 6 && std::abs(destination_pointer_->y_index - temp_grid.y_index) < 6) {
    return true;
  } else {return false;}
}
bool lmk_astar::NormalAStar::collision_detection(VehiclePose check_grid) {
  if (mapdata_pointer_->map_occupancy[check_grid.y_index][check_grid.x_index] == 0) {return true;}
  return false;
}
inline std::vector<std::vector<int>> lmk_astar::NormalAStar::motion_primitive(VehiclePose root_grid) {
  std::vector<std::vector<int>> neighbours;
  neighbours = {{root_grid.y_index + 1, root_grid.x_index, 1}, {root_grid.y_index, root_grid.x_index + 1, 1},
                {root_grid.y_index - 1, root_grid.x_index, 1}, {root_grid.y_index, root_grid.x_index - 1, 1}};
  return neighbours;
}
void lmk_astar::NormalAStar::path_generator(std::vector<std::vector<VehiclePose>>& closedlist) {
  std::shared_ptr<VehiclePose> route_pointer;
  route_pointer = std::make_shared<VehiclePose>(closedlist[destination_pointer_->y_index][destination_pointer_->x_index]);
  while (route_pointer->parent != nullptr) {
    route_pointer = route_pointer->parent;
  }
}
void lmk_astar::NormalAStar::draw_demo(std::shared_ptr<VehiclePose> initial, std::shared_ptr<std::vector<std::vector<VehiclePose>>> closedlist_pointer) {
  ROS_INFO("start drawing Normal A Star pic");
  cv::Mat show_img = cv::imread("/home/mingkun/lmk_ws/src/a_star/assets/map_base_img.jpg");
  cv::Point temp_point, end_point;
  std::shared_ptr<VehiclePose> route_pointer;
  route_pointer = std::make_shared<VehiclePose>((*closedlist_pointer)[destination_pointer_->y_index][destination_pointer_->x_index]);
  // draw destination point and initial point
  temp_point.x = destination_pointer_->x_index;
  temp_point.y = destination_pointer_->y_index;
  end_point.x = temp_point.x + 20*std::cos(destination_pointer_->heading_angle);
  end_point.y = temp_point.y + 20*std::sin(destination_pointer_->heading_angle);
  cv::circle(show_img, temp_point, 8, cv::Scalar(0, 0, 255), 2);
  cv::line(show_img, temp_point, end_point, cv::Scalar(0, 0, 255), 2);
  temp_point.x = initial->x_index;
  temp_point.y = initial->y_index;
  end_point.x = temp_point.x + 20*std::cos(initial->heading_angle);
  end_point.y = temp_point.y + 20*std::sin(initial->heading_angle);
  cv::circle(show_img, temp_point, 8, cv::Scalar(255, 0, 0), 2);
  cv::line(show_img, temp_point, end_point, cv::Scalar(255, 0, 0), 2);
  for (int i = 0; i < mapdata_pointer_->map_height; ++i) {
    for (int j = 0; j < mapdata_pointer_->map_length; ++j) {
      if (status_lookuptable[i][j] == 2) {
        show_img.at<cv::Vec3b>(i,j)[0] = 0;
        show_img.at<cv::Vec3b>(i,j)[1] = 255;
        show_img.at<cv::Vec3b>(i,j)[2] = 0;
      }
    }
  }
  while (route_pointer->parent != nullptr) {
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[0] = 0;
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[1] = 0;
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[2] = 255;
    route_pointer = route_pointer->parent;
  }
  cv::imwrite("/home/mingkun/lmk_ws/src/a_star/assets/test1.jpg", show_img);
}
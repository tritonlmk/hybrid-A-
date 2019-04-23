//#define _DEBUG__

#ifndef LMK_WS_SRC_A_STAR_INCLUDE_A_STAR_H_
#define LMK_WS_SRC_A_STAR_INCLUDE_A_STAR_H_
//if X has not been defined yet, compile the following below
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <cmath>
#include <chrono>
#include <random>
#include <map>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "nh_curve.h"

// standard ros msgs
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PointStamped.h"
// self_defined ros msgs
#include "pos320/Pos320Nav.h"
#include "a_star/Trajectory.h"
#include "map_proc/MapMetaP.h"
// self_defined libs
#include "tiguan_movebase.h"

namespace lmk_astar {
struct VehiclePose {
  double x_pose;
  double y_pose;
  double heading_angle;
  int x_index;
  int y_index;
  double cost_g;
  double total_cost;
  std::shared_ptr<VehiclePose> parent;
  VehiclePose();
  //VehiclePose(const VehiclePose& copy_pose);
  VehiclePose(double x, double y, double angle, double temp_cost_g, double temp_total_cost);
  VehiclePose(double x, double y, double angle, double temp_cost_g);
  VehiclePose(double x, double y, double angle);
  //VehiclePose(const VehiclePose&);
  bool operator < (const VehiclePose& temp_node) const;
  bool operator > (const VehiclePose& temp_node) const;
  bool operator == (const VehiclePose& temp_node) const;
  bool operator != (const VehiclePose& temp_node) const;
  void operator = (const VehiclePose& temp_node);
};
struct ROSMapData {
  ROSMapData();
  int map_height;
  int map_length;
  float resolution;
  std::vector<std::vector<int>> map_occupancy;
  bool metadata_flag;
  bool mapdata_flag;
};
class OpenList {
 public:
  OpenList();
  int find(VehiclePose search_pose) const;
  void pop();
  void insert(VehiclePose insert_pose);
  bool check_nearer(int index, VehiclePose check_pose);
  void decrease(int index, VehiclePose change_pose);
  bool empty();
  VehiclePose top();
 private:
  const double cost_infinity_;
  int heap_size_;
  void min_heapify(int index);
  std::vector<VehiclePose> openlist_data_;
};
class AStarInterface {
 public:
  static const double pi_;
  virtual void Init() = 0;
  virtual void Proc() = 0;
 protected:
  std::vector<std::vector<int>> status_lookuptable;
  OpenList open_list_;
  std::vector<std::vector<VehiclePose>> closed_list_;
  virtual double heuristic_func(VehiclePose cal_pose) = 0;
  virtual void update_neighbour(VehiclePose& cur_pose) = 0;
  virtual bool reach_destination(VehiclePose temp_pose) = 0;
  virtual bool collision_detection(VehiclePose check_pose) = 0;
};
class NormalAStar : public AStarInterface {
 public:
  void Init() override;
  void Init(std::shared_ptr<VehiclePose> desti_input, std::shared_ptr<VehiclePose> initial_input, std::shared_ptr<ROSMapData> mapdata_input);
  void Proc() override;
  void normal_astar_search();
 protected:
  double heuristic_func(VehiclePose start_gird) override;
  void update_neighbour(VehiclePose& cur_grid) override;
  bool reach_destination(VehiclePose temp_grid) override;
  bool collision_detection(VehiclePose check_grid) override;
 private:
  std::shared_ptr<VehiclePose> destination_pointer_;
  std::shared_ptr<VehiclePose> initial_pointer_;
  std::shared_ptr<ROSMapData> mapdata_pointer_;
  std::vector<std::vector<int>> motion_primitive(VehiclePose root_grid);
  void path_generator(std::vector<std::vector<VehiclePose>>& closedlist);
  void draw_demo(std::shared_ptr<VehiclePose> initial, std::shared_ptr<std::vector<std::vector<VehiclePose>>> closedlist_pointer);
};
class HybridAStar : virtual public AStarInterface {
 public:
  ~HybridAStar();
  void Init() override;
  void Proc() override;
  void publish_trajectory();
  void hybrid_astar_search();
 protected:
  double heuristic_func(VehiclePose cal_pose) override;
  void update_neighbour(VehiclePose& cur_pose) override;
  bool reach_destination(VehiclePose temp_pose) override;
  bool collision_detection(VehiclePose check_pose) override;
 private:
  void acquire_mapdata();
  void get_position();
  void set_destination_initial_test();
  std::vector<std::vector<double>> motion_primitive(VehiclePose root_pose);
  void path_generator();
  std::vector<double> random_point(int pic_height, int pic_lenght);
  // funcitons that need to be precomputed, before the start of the on time planner
  void pre_compute_heuristic_cost();
  // functions that are used to draw demos
  void draw_demo();
  void draw_baseimg();
  // member parameters
  ROSMapData map_data_;
  VehiclePose destination_;
  VehiclePose initial_;
  NormalAStar DP_Search_;
  // 0 for not met yet, 1 for openlist, 2 for closedlist
  std::vector<VehiclePose> path_found_;
  std::vector<double> car_parameters_;
  tiguan_movebase::VehicleMoveBase tiguan_model_;
  std::vector<std::vector<int>> heuristic_lookup_talbe_;
  //std::map<std::vector<int>, VehiclePose> closed_list_;
};
} // namespace lmk_astar
// if X has already been defined, goto #else directly
#else
// however, nothing should be stated here
#endif //CATKIN_WS_SRC_A_STAR_INCLUDE_A_STAR_H_
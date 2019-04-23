#ifndef LMK_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_
#define LMK_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_

#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>
#include <cmath>
#include <chrono>
#include <random>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "tiguan_movebase.h"

namespace lmk_libraries {
class NHCurve {
 public:
  const static double pi_;
  void DCurve(double steering_radius);
  double DCurvelength(std::vector<double> initial_pose, std::vector<double> desti_pose, double steering_radius);
 private:
  void showtest(std::vector<double> initial_pose, std::vector<double> desti_pose, double steering_radius);
  double LSLcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend);
  double RSRcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend);
  double LSRcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend);
  double RSLcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend);
  std::vector<double> random_point(int pic_height, int pic_length);
};
}
#endif //CATKIN_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_
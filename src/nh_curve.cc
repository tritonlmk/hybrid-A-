#include "nh_curve.h"

//static member
const double lmk_libraries::NHCurve::pi_ = 3.1416;
//public member function
void lmk_libraries::NHCurve::DCurve(double steering_radius) {
  std::vector<double> initial = random_point(150, 150);
  std::vector<double> desti = random_point(150, 150);
  showtest(initial, desti, steering_radius*2);
}
double lmk_libraries::NHCurve::DCurvelength(std::vector<double> initial_pose, std::vector<double> desti_pose, double steering_radius) {}
//private member function
void lmk_libraries::NHCurve::showtest(std::vector<double> initial_pose, std::vector<double> desti_pose, double steering_radius) {
  std::cout << "start testing" << std::endl;
  // output the length
  int length = LSLcurve(initial_pose, steering_radius, desti_pose, steering_radius);
  std::cout << "LSL length is " << length << std::endl;
  // drawing start/end points and their destination
  cv::Mat test_tanline_img = cv::Mat::zeros(150, 150, CV_8UC3);
  cv::Point startpoint, endpoint;
  startpoint.x = initial_pose[0];
  startpoint.y = initial_pose[1];
  endpoint.x = startpoint.x + 10*std::cos(initial_pose[2]);
  endpoint.y = startpoint.y + 10*std::sin(initial_pose[2]);
  cv::line(test_tanline_img, startpoint, endpoint, cv::Scalar(0, 0, 255), 1);
  endpoint.x = desti_pose[0];
  endpoint.y = desti_pose[1];
  startpoint.x = endpoint.x + 10*std::cos(desti_pose[2]);
  startpoint.y = endpoint.y + 10*std::sin(desti_pose[2]);
  cv::line(test_tanline_img, startpoint, endpoint, cv::Scalar(0, 255, 0), 1);
  // draw start and end circles according to the heading angle and turing direction
  // this is the LSL curve
  initial_pose[2] = initial_pose[2] - pi_/2;
  desti_pose[2] = desti_pose[2] - pi_/2;
  std::vector<double> cstart = {initial_pose[0]+steering_radius*std::cos(initial_pose[2]), initial_pose[1]+steering_radius*std::sin(initial_pose[2]), initial_pose[2]};
  std::vector<double> cend = {desti_pose[0]+steering_radius*std::cos(desti_pose[2]), desti_pose[1]+steering_radius*std::sin(desti_pose[2]), desti_pose[2]};
  startpoint.x = cstart[0];
  startpoint.y = cstart[1];
  cv::circle(test_tanline_img, startpoint, steering_radius, cv::Scalar(0, 0, 255), 1);
  endpoint.x = cend[0];
  endpoint.y = cend[1];
  cv::circle(test_tanline_img, endpoint, steering_radius, cv::Scalar(0, 255, 0), 1);
  // drawing the tan line
  double delta_x = cend[0] - cstart[0];
  double delta_y = cend[1] - cstart[1];
  double angle = std::atan2(delta_y, delta_x) - pi_/2;
  startpoint.x = startpoint.x + steering_radius*std::cos(angle);
  startpoint.y = startpoint.y + steering_radius*std::sin(angle);
  endpoint.x = endpoint.x + steering_radius*std::cos(angle);
  endpoint.y = endpoint.y + steering_radius*std::sin(angle);
  cv::line(test_tanline_img, startpoint, endpoint, cv::Scalar(0, 0, 255), 1);
  cv::imwrite("//home/mingkun/lmk_ws/src/a_star/assets/testtanline.jpg", test_tanline_img);
}
// this is the LSL curve, note that, this is the so called "geometry method"
// the vector based method based on eigen library should be better
double lmk_libraries::NHCurve::LSLcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend) {
  cstart[0] += rstart*std::cos(cstart[2] - pi_/2);
  cstart[1] += rstart*std::sin(cstart[2] - pi_/2);
  cend[0] += rend*std::cos(cend[2] - pi_/2);
  cend[1] += rend*std::sin(cend[2] - pi_/2);
  double delta_x = cend[0] - cstart[0];
  double delta_y = cend[1] - cstart[1];
  // length of the upper outer tan line
  double length = std::sqrt(std::pow(delta_y, 2) + std::pow(delta_x, 2) + std::pow((rend-rstart), 2));
  double angle = std::atan2(delta_y, delta_x) - std::atan2((rend - rstart), std::sqrt(std::pow(delta_y, 2) + std::pow(delta_x, 2)));
  if (cend[2] < 0) {
    cend[2] += 2*pi_;
  } else if (cstart[2] < 0) {
    cstart[2] += 2*pi_;
  } else if (angle < 0) {
    angle += 2*pi_;
  }
  // length of the arc length
  if (angle < cstart[2]) {
    length += rstart*(pi_*2 + angle - cstart[2]);
  } else { length += rstart*(angle - cstart[2]); };
  if (angle < cend[2]) {
    length += rend*(cend[2] - angle);
  } else { length += rend*(cend[2] - angle + pi_*2); }
  return length;
};
// this the RSR curve
double lmk_libraries::NHCurve::RSRcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend) {
  double delta_x = cend[0] - cstart[0];
  double delta_y = cend[1] - cstart[1];
  double angle = std::atan2(delta_y, delta_x) + pi_/2;
  double length = std::sqrt(std::pow(delta_y, 2) + std::pow(delta_x, 2));
  if (angle < cstart[2]) {
    length += rstart*(cstart[2] - angle);
  } else { length += rstart*(cstart[2] - angle); }
};
// this is the LSR curve
double lmk_libraries::NHCurve::LSRcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend) {};
// this is the RSL curve
double lmk_libraries::NHCurve::RSLcurve(std::vector<double> cstart, double rstart, std::vector<double> cend, double rend) {};
// note: random heading angle range is -pi -> pi
std::vector<double> lmk_libraries::NHCurve::random_point(int pic_height, int pic_length) {
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
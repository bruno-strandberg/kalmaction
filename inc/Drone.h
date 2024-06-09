#ifndef DRONE_H
#define DRONE_H
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include "KalmanState.h"

class Drone {

public:
  Drone();
  void setDest(double X_dest, double Y_dest);
  void flyToDest(double destTolerance);
  void setAccStd(double acc_std);
  void setPosStd(double pos_std);
  const cv::Mat& getMap() const { return m_map; }
  
  
 private:
  // constants used to simulate the control rool
  static constexpr double m_dt = 0.1;                       // control-loop time interval
  static constexpr double m_acc_max = 1.0;                  // drone max acceleration in m/s2
  static constexpr double m_approach_zone = 30;             // target approach speed if within this meters of the destination
  static constexpr double m_speed_tolerance = 1.0;          // don't accelerate if speed is within speed_tolerance of target speed
  static constexpr double m_dir_tolerance = 0.9848;         // don't accelerate if cos(direction) within dir_tolerance
  static constexpr double m_cruise_speed = 10;              // drone cruise speed in m/s
  static constexpr double m_approach_speed = 2;             // drone destination approach speed in m/s

  double m_expected_acc_std;               // std of acceleration reading from IMU as expected by the Kalman process
  double m_expected_pos_std;               // std of pos reading from GPS as expected by the Kalman process
  double m_current_time;
  double m_max_flight_time;
  double m_total_distance;
  Eigen::Matrix<double, 2, 1> m_true_a;    // vector for storing true acceleration values based on which IMU can simulate a reading
  Eigen::Vector2d m_dest;                  // destination XY

  KalmanState m_ModelState;
  KalmanState m_TrueState;
  KalmanState m_AccOnly;
  KalmanState m_GpsOnly;
  
  std::normal_distribution<double> m_acc_gaus;  // gaussian to simulate random noise in IMU
  std::normal_distribution<double> m_pos_gaus;  // gaussian to simulate random noise in GPS
  std::default_random_engine m_generator;       // random number generator
  
  cv::Mat m_map;                           // illustration of the flight
  
  void UpdateAcceleration();
  void PredictNextState();
  void EstimateThisState();
  void UpdateMap();  
  
  
};

#endif

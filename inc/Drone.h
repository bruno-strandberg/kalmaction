#ifndef DRONE_H
#define DRONE_H
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>

class Drone {

public:
  Drone();
  void setDest(double X_dest, double Y_dest);
  void flyToDest(double destTolerance);
  void setAccStd(double acc_std);
  void setPosStd(double pos_std);
  
  
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
  Eigen::MatrixXd m_true_next_state;       // matrix for storing the true state for exploring true vs model
  Eigen::Vector2d m_true_a;                // vector for storing true acceleration values based on which IMU can simulate a reading
  Eigen::Vector2d m_dest;                  // destination XY

  Eigen::MatrixXd m_model_state;           // matrix for storing the current model state with [x, y, vx, vy]
  Eigen::MatrixXd m_model_state_cov;       // covariance matrix with model uncertainties
  Eigen::MatrixXd m_model_next_state;      // matrix for storing the predicted next model state with [x, y, vx, vy]
  Eigen::MatrixXd m_model_next_state_cov;  // covariance matrix with model uncertainties

  Eigen::Matrix4d m_A;                     // State transition matrix
  Eigen::Matrix<double, 4, 2> m_B;         // Control input matrix
  Eigen::Matrix4d m_Q;                     // control input covariance
  Eigen::Matrix<double, 2, 4> m_H;         // state-to-measurement transition
  Eigen::Matrix2d m_R;                     // position measurement covariance

  std::normal_distribution<double> m_acc_gaus;  // gaussian to simulate random noise in IMU
  std::normal_distribution<double> m_pos_gaus;  // gaussian to simulate random noise in GPS
  std::default_random_engine m_generator;       // random number generator
  
  cv::Mat m_map;                           // illustration of the flight
  
  void UpdateAcceleration();
  void PredictNextState();
  void EstimateThisState();
  void UpdateMap();
  void setQvals(double acc_std);
  void setRvals(double pos_std);
  
  
  
};

#endif

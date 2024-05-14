#ifndef DRONE_H
#define DRONE_H
#include <Eigen/Dense>

class Drone {

public:
  Drone();
  void setDest(double X_dest, double Y_dest) { m_dest << X_dest, Y_dest; }
  void flyToDest(double destTolerance);
  
  
 private:
  // constants used to simulate the control rool
  static constexpr double m_dt = 0.1;                       // control-loop time interval
  static constexpr double m_acc_max = 1.0;                  // drone max acceleration in m/s2
  static constexpr double m_acc_var = 0.01;                 // variance of acceleration reading from IMU
  static constexpr double m_approach_zone = 30;             // target approach speed if within this meters of the destination
  static constexpr double m_speed_tolerance = 1.0;          // don't accelerate if speed is within speed_tolerance of target speed
  static constexpr double m_dir_tolerance = 0.9848;         // don't accelerate if cos(direction) within dir_tolerance
  static constexpr double m_cruise_speed = 10;              // drone cruise speed in m/s
  static constexpr double m_approach_speed = 2;             // drone destination approach speed in m/s
  
  double m_current_time;
  Eigen::MatrixXd m_model_state;           // matrix for storing the current model state with [x, y, vx, vy]
  Eigen::MatrixXd m_model_state_cov;       // covariance matrix with model uncertainties
  Eigen::MatrixXd m_model_next_state;      // matrix for storing the predicted next model state with [x, y, vx, vy]
  Eigen::MatrixXd m_model_next_state_cov;  // covariance matrix with model uncertainties
  Eigen::MatrixXd m_true_state;            // matrix for storing the true state for exploring true vs model
  Eigen::Vector2d m_true_a;                // vector for storing true acceleration values based on which IMU can simulate a reading
  Eigen::Vector2d m_dest;                  // destination XY

  Eigen::Matrix4d m_A;                     // State transition matrix
  Eigen::Matrix<double, 4, 2> m_B;         // Control input matrix
  Eigen::Matrix4d m_Q;                     // control input covariance
  
  void UpdateAcceleration();
  void PredictNextState();
  void EstimateThisState();
  
  
};

#endif

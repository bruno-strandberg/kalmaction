#ifndef KALMANSTATE_H
#define KALMANSTATE_H

#include <Eigen/Dense>

class KalmanState {

 public:
  KalmanState(double dt, double expct_acc_std, double expct_gps_std);
  KalmanState() = delete;
  void PredictNextState(Eigen::Vector2d acc_imu);
  void EstimateThisState(Eigen::Matrix<double, 2, 1> true_pos);
  const Eigen::MatrixXd& getState(bool next=false) { return next ? m_model_next_state : m_model_state; }
  void setVerbose(bool verbose) { m_verbose = verbose; }
  std::string getStateString(bool next=false);
  
 private:
  bool m_verbose;
  double m_dt;
  double m_expected_acc_std;
  double m_expected_pos_std;
  Eigen::MatrixXd m_model_state;           // matrix for storing the current model state with [x, y, vx, vy]
  Eigen::MatrixXd m_model_state_cov;       // covariance matrix with model uncertainties
  Eigen::MatrixXd m_model_next_state;      // matrix for storing the predicted next model state with [x, y, vx, vy]
  Eigen::MatrixXd m_model_next_state_cov;  // covariance matrix with model uncertainties

  Eigen::Matrix4d m_A;                     // State transition matrix
  Eigen::Matrix<double, 4, 2> m_B;         // Control input matrix
  Eigen::Matrix4d m_Q;                     // control input covariance
  Eigen::Matrix<double, 2, 4> m_H;         // state-to-measurement transition
  Eigen::Matrix2d m_R;                     // position measurement covariance

  void setQvals(double acc_std);
  void setRvals(double pos_std);
  
};

#endif

#include "KalmanState.h"
#include <iostream>

KalmanState::KalmanState(double dt, double expct_acc_std, double expct_gps_std) :
  m_verbose(false),
  m_dt(dt),
  m_expected_acc_std(expct_acc_std),
  m_expected_pos_std(expct_gps_std),
  m_model_state(4, 1),
  m_model_state_cov(4, 4),
  m_model_next_state(4, 1),
  m_model_next_state_cov(4, 4) {

  m_model_state << 0.0, 0.0, 0.0, 0.0;
  m_model_state_cov.setZero();
  m_model_next_state << 0.0, 0.0, 0.0, 0.0;
  m_model_next_state_cov.setZero();

  // init the transition matrices predict the next state
  m_A = Eigen::Matrix4d::Identity();
  m_A(0, 2) = m_dt;
  m_A(1, 3) = m_dt;

  m_B.setZero();
  m_B(0, 0) = 0.5*m_dt*m_dt;   m_B(0, 1) =           0.0;
  m_B(1, 0) =           0.0;   m_B(1, 1) = 0.5*m_dt*m_dt;
  m_B(2, 0) =          m_dt;   m_B(2, 1) =           0.0;
  m_B(3, 0) =             0;   m_B(3, 1) =          m_dt;


  // covariance matrices related to measurement from IMU, GPS
  setQvals(m_expected_acc_std);
  setRvals(m_expected_pos_std);

  m_H.setZero();
  m_H(0, 0) = 1.0;
  m_H(1, 1) = 1.0;
}

// -----------------------------------------------------------------------------

/** 
    @brief Function to set control input covariance matrix parameters depending on the expected accelerometer std
*/
void KalmanState::setQvals(double acc_std) {
  m_Q.setZero();
  double _t1 = pow(m_dt, 4)*acc_std*acc_std/4.0;
  double _t2 = pow(m_dt, 3) * acc_std*acc_std/2.0;
  double _t3 = m_dt*m_dt * acc_std*acc_std;
  m_Q(0, 0) = _t1; m_Q(0, 1) = 0.0; m_Q(0, 2) = _t2; m_Q(0, 3) = 0.0;
  m_Q(1, 0) = 0.0; m_Q(1, 1) = _t1; m_Q(1, 2) = 0.0; m_Q(1, 3) = _t2;
  m_Q(2, 0) = _t2; m_Q(2, 1) = 0.0; m_Q(2, 2) = _t3; m_Q(2, 3) = 0.0;
  m_Q(3, 0) = 0.0; m_Q(3, 1) = _t2; m_Q(3, 2) = 0.0; m_Q(3, 3) = _t3;
}

// -----------------------------------------------------------------------------

/** 
    @brief Function to set position measurement covariance matrix parameters depending on the expected position variance
*/
void KalmanState::setRvals(double pos_std) {
  m_R.setZero();
  m_R(0, 0) = pos_std*pos_std;
  m_R(1, 1) = pos_std*pos_std;
}

/**
   @ brief Function to predict the next state       
 */
void KalmanState::PredictNextState(Eigen::Vector2d acc_imu) {
  m_model_next_state = m_A * m_model_state + m_B * acc_imu;
  m_model_next_state_cov = m_A * m_model_state_cov * m_A.transpose() + m_Q;
  if (m_verbose) {
    std::cout << "KalmanState::PredictNextState model next state: " << m_model_next_state << std::endl;
    std::cout << "KalmanState::PredictNextState model next state cov: " << m_model_next_state_cov << std::endl;
  }
}

// -----------------------------------------------------------------------------

/** 
    @brief Function that estimates the current state
*/
void KalmanState::EstimateThisState(Eigen::Matrix<double, 2, 1> true_pos) {
  Eigen::MatrixXd kalman_gain = m_model_next_state_cov *
    m_H.transpose()*(m_H * m_model_next_state_cov * m_H.transpose() + m_R).inverse();
  if (m_model_next_state_cov.isZero()) {
    kalman_gain.setZero();
  }
  m_model_state = m_model_next_state + kalman_gain * (true_pos - m_H * m_model_next_state);
  m_model_state_cov = m_model_next_state_cov - kalman_gain * m_H * m_model_next_state_cov;
  if (m_verbose) {
    std::cout << "KalmanState::EstimateThisState model state: " << m_model_state << std::endl;
    std::cout << "KalmanState::EstimateThisState model state cov: " << m_model_state_cov << std::endl;
  }

}

// -----------------------------------------------------------------------------

std::string KalmanState::getStateString(bool next) {
  std::ostringstream oss;
  const auto& state = next ? m_model_next_state : m_model_state;
  std::string stateStr = next ? "next" : "this";
  oss << stateStr << " pos (" << state(0) << ", " << state(1) << ") v (" << state(2) << ", " << state(3) << ") abs " << Eigen::Vector2d(state(2), state(3)).norm();
  return oss.str();
}

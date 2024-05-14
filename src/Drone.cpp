#include "Drone.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << "DEBUG: " << str << std::endl; } while( false )
#else
    #define DEBUG_MSG(str) do { } while ( false )
#endif

Drone::Drone() : m_model_state(4, 1), m_model_state_cov(4, 4), m_model_next_state(4, 1), m_model_next_state_cov(4, 4),
                 m_true_state(4, 1), m_true_a(0.0, 0.0), m_dest(0.0, 0.0) {

  // set state's to 0s
  m_model_state << 0.0, 0.0, 0.0, 0.0;
  m_model_state_cov.setZero();
  m_model_next_state << 0.0, 0.0, 0.0, 0.0;
  m_model_next_state_cov.setZero();
  m_true_state << 0.0, 0.0, 0.0, 0.0;

  // init the transition matrices predict the next state
  m_A = Eigen::Matrix4d::Identity();
  m_A(0, 2) = m_dt;
  m_A(1, 3) = m_dt;

  m_B.setZero();
  m_B(0, 0) = 0.5*m_dt*m_dt;   m_B(1, 0) =           0.0;
  m_B(1, 0) =           0.0;   m_B(1, 1) = 0.5*m_dt*m_dt;
  m_B(2, 0) =          m_dt;   m_B(2, 1) =           0.0;
  m_B(3, 0) =             0;   m_B(2, 1) =          m_dt;

  m_Q.setZero();
  // assuming same variance of a along x and y
  double _t1 = pow(m_dt, 4)*m_acc_var/4.0;
  double _t2 = pow(m_dt, 3) * m_acc_var/2.0;
  double _t3 = m_dt*m_dt * m_acc_var;
  m_Q(0, 0) = _t1; m_Q(0, 1) = 0.0; m_Q(0, 2) = _t2; m_Q(0, 3) = 0.0;
  m_Q(1, 0) = 0.0; m_Q(1, 1) = _t1; m_Q(1, 2) = 0.0; m_Q(1, 3) = _t2;
  m_Q(2, 0) = _t2; m_Q(2, 1) = 0.0; m_Q(2, 2) = _t3; m_Q(2, 3) = 0.0;
  m_Q(3, 0) = 0.0; m_Q(3, 1) = _t2; m_Q(3, 2) = 0.0; m_Q(3, 3) = _t3;  
}

/** 
    @brief                Function that performs drone control logic to command acceleration
*/

void Drone::UpdateAcceleration() {
  Eigen::Vector2d pos_est(m_model_state(0, 0), m_model_state(1, 0));
  Eigen::Vector2d speed_est(m_model_state(2, 0), m_model_state(3, 0));
  auto vec_to_dest = m_dest - pos_est;

  if (vec_to_dest.norm() > m_approach_zone) {

    // CRUISE LOGIC
    
    auto cur_dir_delta = speed_est.normalized().dot(vec_to_dest.normalized());
    
    // if close to cruise speed and the direction is OK, set acceleration to 0
    if ((abs(speed_est.norm() - m_cruise_speed) < m_speed_tolerance) &&
        (abs(cur_dir_delta) < m_dir_tolerance)) {
      DEBUG_MSG("Speed is: " << speed_est.norm() << " dir_delta is: " << cur_dir_delta << " setting acc to 0" );
      m_true_a << 0.0, 0.0;
    }
    else {
      Eigen::Vector2d target_v = vec_to_dest.normalized() * m_cruise_speed; // calculate what the speed vector should be
      Eigen::Vector2d acc = (target_v - speed_est)/m_dt; // calculate what the acceleration should be to be at target_v in m_dt
      if (acc.norm() > m_acc_max) {
        acc = acc.normalized() * m_acc_max; // limit the acceleration
      }
      m_true_a = acc;
      DEBUG_MSG("Current speed: " << speed_est(0) << ", " << speed_est(1) << " (abs) " << speed_est.norm() << 
                "\nTarget speed: " << target_v(0) << ", " << target_v(1) << " (abs) " << target_v.norm() <<
                "\nAcceleration: " << m_true_a(0) << ", " << m_true_a(1));

    }
  }
  else {
    // APPROACH LOGIC
  }
  
}

void Drone::PredictNextState() {
  auto acc_imu = m_true_a;  // introduce noise here
  m_model_next_state = m_A * m_model_state + m_B * acc_imu;
  m_model_next_state_cov = m_A * m_model_state_cov * m_A.transpose() + m_Q;
}

/** 
    @brief                Function that updates the drone state with the Kalman tecnique
*/
void Drone::EstimateThisState() {

  // this requires predicted state from previous iteration
  
  
}

/** 
    @brief                Function that runs a loop that simulates drone flight from current position to destination
    @param destTolerance  Finish the loop if the drone is within destTolerance meters of the destintion
 */
void Drone::flyToDest(double destTolerance) {
  if (destTolerance < 1.0) {
    throw std::invalid_argument("Drone::flyToDest Argument destTolerance must be greater or equal than 1 meter");
  }
  
  Eigen::Vector2d pos_est(m_model_state(0, 0), m_model_state(1, 0));
  auto distance = (pos_est - m_dest).norm();

  if (distance < destTolerance) {
    std::cout << "The drone postion (" << pos_est(0) << ", " << pos_est(1) << ") is within "
              << destTolerance << " meters of the destination (" << m_dest(0) << ", " << m_dest(1) << "), exiting" << std::endl;
    return;
  }

  UpdateAcceleration();

  
  // code that executes a while loop that ends when the drone thinks it has reached it's destination
}
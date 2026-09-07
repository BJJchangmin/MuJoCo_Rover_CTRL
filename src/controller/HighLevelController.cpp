#include "HighLevelController.hpp"

#include <cmath>
#include <iostream>

#include "Useful.hpp"

template <typename T>
HighLevelController<T>::HighLevelController(RobotLeg<T>& robot)
    : robot_(robot),
      Highlevelctrl_yawrate_output(0),
      Highlevelctrl_vx_output(0),
      wheel_base_(1.09),
      wheel_track_(1.126),
      wheel_radius_(0.15),
      chassis_height_neutral_(0),
      sus_link_length_(0.27),
      sus_pos_limit_(60*M_PI/180),
      chassis_traj_ptr_(nullptr),
      joint_traj_ptr_(nullptr)
{
  sus_pos_neutral_.setZero();
}

template <typename T>
void HighLevelController<T>::High_level_ctrl()
{
  Kin_High_level_ctrl();
  ICR_Kinematic_mapping();
  Ori_Kinematic_mapping();
}


template <typename T>
void HighLevelController<T>::Kin_High_level_ctrl()
{
  if (!chassis_traj_ptr_ || !joint_traj_ptr_)
  {
    return;
  }

  // T kp_vx = fst_order_model_gain(1, 0.6);
  // T kp_gamma = fst_order_model_gain(1, 0.12);

  T kp_vx = 0;
  T kp_gamma = 0;
  T kp_roll = 5;
  T kp_pitch = 5;


  //! Algorithm for yaw rate , Longitudinal Control
  Highlevelctrl_yawrate_output
      = chassis_traj_ptr_->chassis_angvel_des_[2]
      + kp_gamma * (chassis_traj_ptr_->chassis_angvel_des_[2]
                    - robot_.body_omega_world_[2]);

  Highlevelctrl_vx_output
      = chassis_traj_ptr_->chassis_vel_des_[0]
      + kp_vx * (chassis_traj_ptr_->chassis_vel_des_[0]
                 - robot_.body_vel_world_[0]);

  Highlevelctrl_roll_output = chassis_traj_ptr_->chassis_angpos_des_[0]
      + kp_roll * (chassis_traj_ptr_->chassis_angpos_des_[0]
                    - robot_.body_euler_world_[0]);

  Highlevelctrl_pitch_output = chassis_traj_ptr_->chassis_angpos_des_[1]
      + kp_pitch * (chassis_traj_ptr_->chassis_angpos_des_[1]
                    - robot_.body_euler_world_[1]);

}

template <typename T>
void HighLevelController<T>::ICR_Kinematic_mapping()
{
  if (!chassis_traj_ptr_ || !joint_traj_ptr_)
  {
    return;
  }

  std::cout << "Highlevelctrl_vx_output: " << Highlevelctrl_vx_output << std::endl;
  std::cout << "Highlevelctrl_yawrate_output: " << Highlevelctrl_yawrate_output << std::endl;

  T rov_ICR = Max(Highlevelctrl_vx_output, 0.0001)
      / std::copysign(
          Max(abs(Highlevelctrl_yawrate_output), 0.0001),
          Highlevelctrl_yawrate_output);

  T wheel_ICR[4];

  wheel_ICR[0] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR-wheel_track_/2, 2));
  wheel_ICR[1] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR+wheel_track_/2, 2));
  wheel_ICR[2] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR-wheel_track_/2, 2));
  wheel_ICR[3] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR+wheel_track_/2, 2));

  //! Algorithm for yaw rate , Longitudinal Control
  for (size_t i = 0; i < 4; i++)
  {
    joint_traj_ptr_->joint_vel_des_[i][2] =
        (Highlevelctrl_vx_output
         * abs(wheel_ICR[i] / abs(rov_ICR))) / wheel_radius_;

  }

  joint_traj_ptr_->joint_pos_des_[0][1]
      = saturation_block(60*M_PI/180, -60*M_PI/180,
                         atan((wheel_base_/2)/(rov_ICR-wheel_track_/2)));  // FL
  joint_traj_ptr_->joint_pos_des_[1][1]
      = saturation_block(60*M_PI/180, -60*M_PI/180,
                         atan((wheel_base_/2)/(rov_ICR+wheel_track_/2)));  // FR
  joint_traj_ptr_->joint_pos_des_[2][1]
      = saturation_block(60*M_PI/180, -60*M_PI/180,
                         atan((-wheel_base_/2)/(rov_ICR-wheel_track_/2)));  // RL
  joint_traj_ptr_->joint_pos_des_[3][1]
      = saturation_block(60*M_PI/180, -60*M_PI/180,
                         atan((-wheel_base_/2)/(rov_ICR+wheel_track_/2)));  // RR


}

template <typename T>
void HighLevelController<T>::Ori_Kinematic_mapping()
{
  if (!chassis_traj_ptr_ || !joint_traj_ptr_)
  {
    return;
  }

  // Input: chassis height [m], roll/pitch [rad].
  // Output order: FL, FR, RL, RR suspension load-side position [rad].
  // const T height_error =
  //     chassis_traj_ptr_->chassis_pos_des_[2] - chassis_height_neutral_;
  T height_error = 0;
  const T roll_ref = Highlevelctrl_roll_output;
  const T pitch_ref = Highlevelctrl_pitch_output;

  const T roll_sign[4] = {1.0, -1.0, 1.0, -1.0};
  const T pitch_sign[4] = {-1.0, -1.0, 1.0, 1.0};

  for (size_t i = 0; i < 4; i++)
  {
    // Desired vertical displacement of the chassis corner [m].
    const T h_body = height_error
        + roll_sign[i] * (wheel_track_ / 2.0) * std::sin(roll_ref)
        + pitch_sign[i] * (wheel_base_ / 2.0) * std::sin(pitch_ref);

    T asin_input = std::sin(sus_pos_neutral_[i]) - h_body / sus_link_length_;
    if (asin_input > 1.0)
    {
      asin_input = 1.0;
    }
    else if (asin_input < -1.0)
    {
      asin_input = -1.0;
    }

    // Load-side suspension joint position reference [rad].
    T sus_pos_ref = std::asin(asin_input) - roll_sign[i] * roll_ref;

    sus_pos_ref = saturation_block(sus_pos_limit_, -sus_pos_limit_, sus_pos_ref);

    joint_traj_ptr_->joint_pos_des_[i][0] = sus_pos_ref;
  }


}



template <typename T>
void HighLevelController<T>::get_traj_pointer(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredChassisTrajectory>
        chassis_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory>
        joint_traj_ptr)
{
  chassis_traj_ptr_ = chassis_traj_ptr;
  joint_traj_ptr_ = joint_traj_ptr;
}

template class HighLevelController<float>;
template class HighLevelController<double>;

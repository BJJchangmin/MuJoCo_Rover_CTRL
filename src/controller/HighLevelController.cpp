#include "HighLevelController.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "Useful.hpp"
#include "orientation_tools.hpp"

using namespace Eigen;

namespace
{
enum class Mode { Kinematic, Optimization };

// Select each axis here, then rebuild. Optimization is not implemented yet.
constexpr Mode vx_mode      = Mode::Kinematic;
constexpr Mode yawrate_mode = Mode::Kinematic;
constexpr Mode roll_mode    = Mode::Optimization;
constexpr Mode pitch_mode   = Mode::Optimization;
constexpr Mode height_mode  = Mode::Optimization;
}

template <typename T>
HighLevelController<T>::HighLevelController(RobotLeg<T>& robot)
    : robot_(robot),
      Highlevelctrl_yawrate_output(0),
      Highlevelctrl_vx_output(0),
      Highlevelctrl_roll_output(0),
      Highlevelctrl_pitch_output(0),
      wheel_base_(1.09),
      wheel_track_(1.126),
      wheel_radius_(0.15),
      chassis_height_neutral_(0),
      sus_link_length_(0.27),
      sus_pos_limit_(60*M_PI/180),
      sus_pos_rate_limit_(0.5),
      chassis_traj_ptr_(nullptr),
      joint_traj_ptr_(nullptr)
{

  sus_pos_neutral_.setZero();
  sus_pos_ref_limited_.setZero();
  Opt_w_des_.setZero();
  Opt_f_des_.setZero();
  Opt_A_des_.setZero();




}

template <typename T>
void HighLevelController<T>::High_level_ctrl()
{
  Kin_High_level_ctrl();
  ICR_Kinematic_mapping();
  // Ori_Kinematic_mapping();
  High_level_optimization_ctrl();
  OptForce_2_Torque();


}

template <typename T>
void HighLevelController<T>::High_level_optimization_ctrl()
{
  if (!chassis_traj_ptr_ || !joint_traj_ptr_)
  {
    return;
  }

  Optimization_Dynamics_mapping();

  // World -> Chassis 회전 행렬
  const Mat3<T> C_BW = ori::quaternionToRotationMatrix(robot_.body_ang_quat_world_);

  // 월드 +z 방향을 chassis 좌표계로 표현한 단위벡터
  // 노면 법선이 아니라 "월드 위쪽 방향" 이다
  const Vec3<T> vertical_chassis = C_BW.col(2);

  const T nx = vertical_chassis[0];
  const T ny = vertical_chassis[1];
  const T nz = vertical_chassis[2];


  for (int i = 0; i < 4; ++i)
  {
    const T x = wheel_pos_body[i][0];
    const T y = wheel_pos_body[i][1];
    const T z = wheel_pos_body[i][2];

    Opt_A_des_(0, i) = T(1);
    Opt_A_des_(1, i) = y*nz - z*ny;
    Opt_A_des_(2, i) = z*nx - x*nz;
  }

  Opt_w_des_ << Highlevelctrl_height_output, Highlevelctrl_roll_output, Highlevelctrl_pitch_output;

  MatrixXd A = Opt_A_des_.template cast<double>();
  VectorXd w = Opt_w_des_.template cast<double>();

  MatrixXd Q = MatrixXd::Zero(3,3);
  // [Fz, Mx, My]: favor equal force commands over level-body tracking.
  // Suspension travel protection is applied separately in OptForce_2_Torque().
  Q.diagonal() << 1, 1, 1;

  const double lambda_balance = 0.1;
  const double lambda_rate = 0.001;

  //! ProxQP Cost Function Formulation
  //! Must be keep the ProxQP formulation Rule

  MatrixXd H_tracking = A.transpose() * Q * A;
  VectorXd g_tracking = -A.transpose() * Q * w;

  MatrixXd I = MatrixXd::Identity(4, 4);
  MatrixXd D = I - MatrixXd::Constant(4, 4, 0.25);
  MatrixXd H_balance = lambda_balance * D.transpose() * D;

  MatrixXd H_rate = lambda_rate * I;
  VectorXd g_rate = -lambda_rate * Opt_f_prev;

  MatrixXd H = H_tracking + H_balance + H_rate;
  // Preserve the same quadratic form while removing round-off asymmetry.
  H = (0.5 * (H + H.transpose())).eval();
  VectorXd g = g_tracking + g_rate;

  //! ProxQP Constraint Formulation

  MatrixXd C = MatrixXd::Identity(4,4);
  VectorXd l = VectorXd::Constant(4, 0);
  VectorXd u = VectorXd::Constant(4, 200);

  // 1. 입력에 NaN/Inf가 있으면 solver를 호출하지 않는다.
  if (!H.allFinite() || !g.allFinite()
      || !C.allFinite() || !l.allFinite() || !u.allFinite())
  {
    return;
  }

  if (!Opt_qp_initialized_)
  {
    Opt_qp_.settings.eps_abs = 1e-6;
    Opt_qp_.settings.eps_rel = 0.0;

    Opt_qp_.init(
        H, g,
        proxsuite::nullopt, proxsuite::nullopt, // hard equality 없음
        C, l, u);

    Opt_qp_initialized_ = true;
    // Opt_qp_.settings.verbose = true;
  }
  else
  {
    Opt_qp_.update(
        H, g,
        proxsuite::nullopt, proxsuite::nullopt,
        C, l, u);
  }

  Opt_qp_.solve();

  // 4. 정상적으로 풀렸는지 확인한다.
  if (Opt_qp_.results.info.status !=
      proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
  {
    return;
  }

  const auto& f_solution = Opt_qp_.results.x;

  if (!f_solution.allFinite())
  {
    return;
    std::cout<< "Nan/Inf 발생" << std::endl;
  }


  // 6. 정상 결과만 저장한다.
  // 순서: FL, FR, RL, RR / 단위: N
  Opt_f_des_ = f_solution.template cast<T>();

  // std::cout << "Fz [N] | "
  //         << "FL: " << Opt_f_des_[0] << "  "
  //         << "FR: " << Opt_f_des_[1] << "  "
  //         << "RL: " << Opt_f_des_[2] << "  "
  //         << "RR: " << Opt_f_des_[3] << "  "
  //         << "Total: " << Opt_f_des_.sum()
  //         << '\n';

  // 다음 제어 주기의 변화량 cost에 사용한다.
  Opt_f_prev = f_solution;

  Opt_solution_valid_ = true;



}



template <typename T>
void HighLevelController<T>::Kin_High_level_ctrl()
{
  if (!chassis_traj_ptr_ || !joint_traj_ptr_)
  {
    return;
  }

  T kp_vx_kin = 0;
  T kp_gamma_kin = 0;
  T kp_roll_kin = 5;
  T kp_pitch_kin = 5;

  T kp_vx_opt = 0;
  T kp_gamma_opt = 0;
  T kp_roll_opt = 10000;
  T kp_pitch_opt = 10000;
  T kp_height_opt = 10000;

  T kd_roll_opt = 1000;
  T kd_pitch_opt = 3000;
  T kd_height_opt = 3000;

  //! Algorithm for yaw rate , Longitudinal Control
  if (yawrate_mode == Mode::Kinematic)
  {
    //* Output: Yaw rate
    Highlevelctrl_yawrate_output
      = chassis_traj_ptr_->chassis_angvel_des_[2]
      + kp_gamma_kin * (chassis_traj_ptr_->chassis_angvel_des_[2]
                    - robot_.body_omega_world_[2]);

  }
  else if (yawrate_mode == Mode::Optimization)
  {
    //* Output: Mz
    Highlevelctrl_yawrate_output =
      kp_gamma_opt * (chassis_traj_ptr_->chassis_angvel_des_[2]
                    - robot_.body_omega_world_[2]);
  }


  if (vx_mode == Mode::Kinematic)
  {
    //* Output: Vx
    Highlevelctrl_vx_output
      = chassis_traj_ptr_->chassis_vel_des_[0]
      + kp_vx_kin * (chassis_traj_ptr_->chassis_vel_des_[0]
                 - robot_.body_vel_world_[0]);

  }
  else if (vx_mode == Mode::Optimization)
  {
    //* Output: Fx
    Highlevelctrl_vx_output
      = kp_vx_opt * (chassis_traj_ptr_->chassis_vel_des_[0]
                 - robot_.body_vel_world_[0]);
  }


  if (roll_mode == Mode::Kinematic)
  {
    //* Output: Roll
    Highlevelctrl_roll_output = chassis_traj_ptr_->chassis_angpos_des_[0]
      + kp_roll_kin * (chassis_traj_ptr_->chassis_angpos_des_[0]
                    - robot_.body_euler_world_[0]);
  }
  else if (roll_mode == Mode::Optimization)
  {
    //* Output: Mx
    Highlevelctrl_roll_output = kp_roll_opt * (chassis_traj_ptr_->chassis_angpos_des_[0]
                    - robot_.body_euler_world_[0]) + kd_roll_opt *(0 - robot_.body_omega_chassis_[0]);

    // std::cout << "Trunk Mx : " << Highlevelctrl_roll_output << std::endl;

  }


  if (pitch_mode == Mode::Kinematic)
  {
    //* Output: Pitch
    Highlevelctrl_pitch_output = chassis_traj_ptr_->chassis_angpos_des_[1]
      + kp_pitch_kin * (chassis_traj_ptr_->chassis_angpos_des_[1]
                    - robot_.body_euler_world_[1]);
  }
  else if (pitch_mode == Mode::Optimization)
  {
    //* Output: My
    Highlevelctrl_pitch_output = kp_pitch_opt * (chassis_traj_ptr_->chassis_angpos_des_[1]
                    - robot_.body_euler_world_[1]) + kd_pitch_opt * (0 - robot_.body_omega_chassis_[1]);
    // std::cout << "Trunk My : " << Highlevelctrl_pitch_output << std::endl;
  }


  if (height_mode == Mode::Optimization)
  {
    //* Output: Fz
    Highlevelctrl_height_output = -robot_.Total_mass_ * robot_.gravity_
      + kp_height_opt * (0.2 - robot_.body_pos_world_[2])
      + kd_height_opt * (0 - robot_.body_vel_world_[2]);


    // std::cout << "Trunk Fz : " << Highlevelctrl_height_output << std::endl;
  }


}

template <typename T>
void HighLevelController<T>::ICR_Kinematic_mapping()
{
  if (!chassis_traj_ptr_ || !joint_traj_ptr_)
  {
    return;
  }


  // 정지 명령: 조향 위치 [rad], 구동 속도 [rad/s]를 모두 0으로 설정.
  if (std::abs(Highlevelctrl_vx_output) < T(1e-6) &&
      std::abs(Highlevelctrl_yawrate_output) < T(1e-6))
  {
    for (int i = 0; i < 4; ++i)
    {
      joint_traj_ptr_->joint_pos_des_[i][1] = T(0);
      joint_traj_ptr_->joint_vel_des_[i][2] = T(0);
    }
    return;
  }



  T rov_ICR = Max(Highlevelctrl_vx_output, 0.000000000001)
      / std::copysign(
          Max(abs(Highlevelctrl_yawrate_output), 0.00000000001),
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

    sus_pos_ref_limited_[i] =
      Rate_Limit(sus_pos_ref, sus_pos_ref_limited_[i], sus_pos_rate_limit_);

    joint_traj_ptr_->joint_pos_des_[i][0] = sus_pos_ref_limited_[i];
  }


}

template <typename T>
void HighLevelController<T>::Optimization_Dynamics_mapping()
{
  // Wheel center positions:
  // chassis origin 기준, body 좌표계 [m]
  // Wheel order: FL, FR, RL, RR


  const T side_sign[4]  = {1, -1, 1, -1};
  const T front_sign[4] = {1,  1, -1, -1};

  // 현재 v1 XML에서 유도한 치수.
  // 기존 Ori mapping의 sus_link_length_ = 0.27과 구분한다.
  const T suspension_length = T(0.16);
  const T steering_offset   = T(0.163);
  const T wheel_z_neutral   = T(-0.01);

  for (int i = 0; i < 4; ++i)
  {
    // 실제 load-side 관절각 [rad], reference가 아님.
    const T q_sus   = robot_.joint_pos_act_[i][0];
    const T q_steer = robot_.joint_pos_act_[i][2];

    wheel_pos_body[i][0] =
        front_sign[i] * wheel_base_ / T(2)
        - side_sign[i] * steering_offset * std::sin(q_steer);

    wheel_pos_body[i][1] =
        side_sign[i] * (
            wheel_track_ / T(2)
            + suspension_length * (std::cos(q_sus) - T(1))
            + steering_offset * (std::cos(q_steer) - T(1)));

    wheel_pos_body[i][2] =
        wheel_z_neutral + suspension_length * std::sin(q_sus);
  }

}

template <typename T>
void HighLevelController<T>::OptForce_2_Torque()
{

  // World -> Chassis 회전 행렬
  const Mat3<T> C_BW = ori::quaternionToRotationMatrix(robot_.body_ang_quat_world_);

  // 월드 +z 방향을 chassis 좌표계로 표현한 단위벡터
  // 노면 법선이 아니라 "월드 위쪽 방향" 이다
  const Vec3<T> vertical_chassis = C_BW.col(2);

  const T side_sign[4] = {T(1), T(-1), T(1), T(-1)};





  // 수평 차체, 수직 중력, 이상적인 평행링크 가정.
  const T g = -robot_.gravity_;  // 양수 중력 크기 [m/s²]

  const T suspension_length = T(0.16);  // [m]
  const T bar_mass = T(5.56431);         // [kg]
  const T distal_mass =
      T(1.7057) + T(2.73012) + T(5.00004);  // [kg]

  // Load-side travel protection, not another body-leveling controller.
  // No position-restoring torque inside +/-10 deg; retain the existing
  // joint damping in main.cc. The MJCF +/-60 deg limits are a backup.
  const T sus_travel_start = T(10.0 * M_PI / 180.0);  // [rad]
  const T kp_sus_travel = T(400);                   // [N*m/rad]

  for (int i = 0; i < 4; ++i)
  {
    const T q = robot_.joint_pos_act_[i][0];  // [rad]
    const T cos_q = std::cos(q);
    const T sin_q = std::sin(q);

    Vec3<T> J_sus_chassis;

    J_sus_chassis << T(0), -side_sign[i] * suspension_length * sin_q, suspension_length * cos_q;

    // 여기서 dot은 내적하는거임
    const T J_vertical = J_sus_chassis.dot(vertical_chassis);

    // 다리 자체의 중력보상 토크 [N*m]. 위의 수평 차체/평행링크 가정.
    // const T gravity_torque =
    //     g * (
    //         bar_mass * (T(0.08) * cos_q - T(0.025) * sin_q)
    //         + distal_mass * suspension_length * cos_q
    //     );

    const T gravity_torque = 0;

    const T force_torque = -J_vertical * Opt_f_des_[i];
    const T total_torque = force_torque + gravity_torque;

    robot_.joint_torque_des_[i][0] = total_torque;

    // std::cout << "Leg " << i
    //           << " | q [rad]: " << q
    //           << " | Jz [m/rad]: " << Jz
    //           << " | Fz [N]: " << Opt_f_des_[i]
    //           << " | force_tau [Nm]: " << force_torque
    //           << " | gravity_tau [Nm]: " << gravity_torque
    //           << " | total_tau [Nm]: " << total_torque
    //           << '\n';
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

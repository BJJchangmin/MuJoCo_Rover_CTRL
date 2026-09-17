#include "HighLevelController.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>

#include "Useful.hpp"
#include "orientation_tools.hpp"

using namespace Eigen;

namespace
{
enum class Mode { Kinematic, Optimization, Both };

// Select each axis here, then rebuild. Mapping/motor calls below and in main.cc must match.
constexpr Mode vx_mode      = Mode::Both;
constexpr Mode yawrate_mode = Mode::Both;
constexpr Mode roll_mode    = Mode::Optimization;
constexpr Mode pitch_mode   = Mode::Optimization;
constexpr Mode height_mode  = Mode::Optimization;
}

template <typename T>
HighLevelController<T>::HighLevelController(RobotLeg<T>& robot)
    : robot_(robot),
      Highlevelctrl_yawrate_output_kin(0),
      Highlevelctrl_yawrate_output_opt(0),
      Highlevelctrl_vx_output_kin(0),
      Highlevelctrl_vx_output_opt(0),
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

  for (int i = 0; i < 4; ++i) {
    contact_normal_body_[i].setZero();
    contact_tangent_body_[i].setZero();
    contact_force_body_des_[i].setZero();
  }

  higlctrl_opt_ptr_ = std::make_shared<HighCtrl_Optimization>();


}

template <typename T>
void HighLevelController<T>::High_level_ctrl(double time_s)
{
  Opt_fxfz_solution_valid_ = false;

  if (!chassis_traj_ptr_ || !joint_traj_ptr_) {
    return;
  }
  if (height_mode == Mode::Optimization && !est_chassis_height(time_s)) {
    Opt_solution_valid_ = false;
    std::cerr << "Invalid four-point chassis height: skip this control update; "
                 "previous joint commands are retained.\n";
    return;
  }

  Kin_High_level_ctrl();
  ICR_Kinematic_mapping();
  // Ori_Kinematic_mapping();
  // High_level_optimization_ctrl();
  High_level_optimization_ctrl_FxFz();
  OptForce_2_Torque_sus();
  OptForce_2_Torque_dri();

}

template <typename T>
bool HighLevelController<T>::est_chassis_height(double time_s)
{
  // Reuse the same v1 suspension/steering kinematics as the force allocator.
  Optimization_Dynamics_mapping();
  Eigen::Matrix<double, 4, 3> wheel_centers_B;
  for (int i = 0; i < 4; ++i) {
    wheel_centers_B.row(i) = wheel_pos_body[i].template cast<double>().transpose();
  }

  Eigen::Vector4d quat = robot_.body_ang_quat_world_.template cast<double>();
  const double quat_norm = quat.norm();
  if (!quat.allFinite() || !std::isfinite(quat_norm) || quat_norm < 1e-12) {
    chassis_height_estimator_.reset();
    return false;
  }
  quat /= quat_norm;
  const Eigen::Matrix3d C_BW = ori::quaternionToRotationMatrix(quat);
  const Eigen::Vector3d imu_position_B(0.0, 0.0, 0.05);  // v1 IMU offset [m]
  return chassis_height_estimator_.update(
      wheel_centers_B, C_BW.transpose(), imu_position_B,
      static_cast<double>(wheel_radius_), time_s);
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

  // const double lambda_balance = 0.1;
  const double lambda_rate = 0.001;

  //! ProxQP Cost Function Formulation
  //! Must be keep the ProxQP formulation Rule

  // MatrixXd H_tracking = A.transpose() * Q * A;
  // VectorXd g_tracking = -A.transpose() * Q * w;

  MatrixXd I = MatrixXd::Identity(4, 4);
  // MatrixXd D = I - MatrixXd::Constant(4, 4, 0.25);
  // MatrixXd H_balance = lambda_balance * D.transpose() * D;

  MatrixXd H_rate = lambda_rate * I;
  VectorXd g_rate = -lambda_rate * Opt_f_prev;

  MatrixXd H = I + H_rate;
  // Preserve the same quadratic form while removing round-off asymmetry.
  H = (0.5 * (H + H.transpose())).eval();

  VectorXd g = g_rate;

  //! ProxQP Constraint Formulation

  MatrixXd C = MatrixXd::Identity(4,4);
  VectorXd l = VectorXd::Constant(4, 0);
  VectorXd u = VectorXd::Constant(4, 200);


  // 1. 입력에 NaN/Inf가 있으면 solver를 호출하지 않는다.
  if (!H.allFinite() || !g.allFinite()
      || !A.allFinite() || !w.allFinite()
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
        A, w, // hard equality 없음 // inequality, equality가 없으면 proxsuite::nullopt 로 자리를 대신 채워 줘야 한다
        C, l, u);

    Opt_qp_initialized_ = true;
    // Opt_qp_.settings.verbose = true;
  }
  else
  {
    Opt_qp_.update(
        H, g,
        A, w,
        C, l, u);
  }

  Opt_qp_.solve();

  // 4. 정상적으로 풀렸는지 확인한다.
  if (Opt_qp_.results.info.status !=
      proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
  {
    std::cout<<"문제 발생 정상적으로 안풀림" << std::endl;
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

  std::cout << "Fz [N] | "
          << "FL: " << Opt_f_des_[0] << "  "
          << "FR: " << Opt_f_des_[1] << "  "
          << "RL: " << Opt_f_des_[2] << "  "
          << "RR: " << Opt_f_des_[3] << "  "
          << "Total: " << Opt_f_des_.sum()
          << '\n';

  // solver status가 PROXQP_SOLVED인지 확인한 뒤
  // std::cout << "desired wrench: " << w.transpose() << '\n';
  // std::cout << "QP wrench: "
  //           << (A * f_solution).transpose() << '\n';
  // std::cout << "equality residual: "
  //           << (A * f_solution - w).transpose() << '\n';

  // 다음 제어 주기의 변화량 cost에 사용한다.
  Opt_f_prev = f_solution;

  Opt_solution_valid_ = true;

  publish_opt_result();



}

template <typename T>
void HighLevelController<T>::High_level_optimization_ctrl_FxFz()
{

  //이번 호출의 결과 유효 여부
  Opt_fxfz_solution_valid_ = false;

  if (!chassis_traj_ptr_ || !joint_traj_ptr_ || !estimate_param_ptr_)
  {
    std::cerr << "FxFz QP: missing trajectory/Estimate pointer; retain previous torques.\n";
    return;
  }

  // World -> Body. Reject invalid attitude before constructing contact frames.
  Eigen::Vector4d quat = robot_.body_ang_quat_world_.template cast<double>();
  const double quat_norm = quat.norm();
  if (!quat.allFinite() || !std::isfinite(quat_norm) || quat_norm < 1e-12) {
    std::cerr << "FxFz QP: invalid quaternion; retain previous torques.\n";
    return;
  }
  quat /= quat_norm;
  const Eigen::Matrix3d C_BW = ori::quaternionToRotationMatrix(quat);

  // Keep the user's existing Body-origin -> contact-point approximation.
  // It is NOT the measured contact position, nor a lever arm about total CoM.
  Optimization_Dynamics_mapping();

  Opt_A_fxfz_des_.setZero();

  // =============
  // 2. A 구성: 접촉력 -> 차체 힘/모멘트
  // =============

  for (int i=0; i<4;i++)
  {
    // This allocator still assumes four supporting wheels. No guessed normal
    // and no variable-contact reallocation when a supporting contact is missing.
    if (!estimate_param_ptr_->contact_normal_valid_[i]) {
      std::cerr << "FxFz QP: no valid support normal for wheel " << i
                << "; retain previous torques.\n";
      return;
    }

    Eigen::Vector3d normal_world;
    for (int j = 0; j < 3; ++j) {
      normal_world[j] = estimate_param_ptr_->contact_normal_world_[i][j];
    }
    const double normal_norm = normal_world.norm();
    if (!normal_world.allFinite() || !std::isfinite(normal_norm) || normal_norm < 1e-12) {
      std::cerr << "FxFz QP: invalid contact normal for wheel " << i << '\n';
      return;
    }
    // 동일 게인 비교: 접촉 유효성 검사는 유지하되 법선 방향은 평지로 가정.
    // Uneven terrain 법선 적용 코드 보존 (복원 시 아래 평지 선언과 교체):
    const Eigen::Vector3d n_B = C_BW * (normal_world / normal_norm);

    // n_W = [0, 0, 1] -> Body. 차체 roll/pitch/yaw 변환은 유지한다.
    // const Eigen::Vector3d n_B = C_BW.col(2);
    const Eigen::Vector3d r_B = wheel_contact_pos_body[i].template cast<double>();

    const double delta = robot_.joint_pos_act_[i][2];

    //* 바퀴의 회전방향은 y방향, 그리고 법선벡터 를 서로 외적하면 바퀴의 전진방향이 나오는데 그게 t_B에 해당됨
    //* 바퀴의 회전방향 방향은 조향각에 따라 회전방향 축이 달라짐 그걸 표현하기 위해서 axle_B로 표현한거임
    const Eigen::Vector3d axle_B(-std::sin(delta), std::cos(delta), 0.0);

    //* t_b 바퀴의 전진방향을 의미. Uneven terrain 고려를 위해 cross로 표현함
    Eigen::Vector3d t_B = axle_B.cross(n_B); //

    //* 전빈방향을 의미하게 되고 단위벡터여야 하기 때문에 norm으로 처리해준다. 실제 계산은 밑에 있음
    const double t_norm = t_B.norm();

    if (!t_B.allFinite() ||
        !std::isfinite(t_norm) ||
        t_norm < 1e-12)
    {
      std::cerr << "FxFz QP: invalid rolling direction\n";
      return;
    }

    // 힘의 크기와 방향을 분리하기 위해 단위벡터로 만든다.
    t_B /= t_norm;
    contact_normal_body_[i] = n_B.template cast<T>();
    contact_tangent_body_[i] = t_B.template cast<T>();

    //*
    const Eigen::Vector3d moment_fx = r_B.cross(t_B);
    const Eigen::Vector3d moment_fz = r_B.cross(n_B);

    Opt_A_fxfz_des_.col(i)
      << t_B.x(),          // chassis Fx
         t_B.z(),          // chassis Fz: 접선력도 Body z 성분을 가질 수 있다
         moment_fx.x(),    // chassis Mx
         moment_fx.y(),    // chassis My
         moment_fx.z();    // chassis Mz

    // 뒤 4열: 가정한 법선력 fn의 Body wrench 기여.
    // 현재 평지 가정에서는 fn이 월드 수직력이다.
    Opt_A_fxfz_des_.col(4+i)
      << n_B.x(),          // chassis Fx
         n_B.z(),          // chassis Fz
         moment_fz.x(),    // chassis Mx
         moment_fz.y(),    // chassis My
         moment_fz.z();    // chassis Mz

  }

  // ============================================================
  // 3. 요구 wrench 구성
  // 순서: Fx_body, Fz_body, Mx_body, My_body, Mz_body
  // 단위: N, N, N*m, N*m, N*m
  // ============================================================
  // Keep the existing velocity loop along Body x and height loop along World z.
  // Rotate the WHOLE vertical force (including mg) into Body before summing.
  // This also supplies its Body-x gravity/height contribution on a tilted chassis.
  // Body Fy is not constrained by this five-row allocator.
  const Eigen::Vector3d desired_force_body =
      Eigen::Vector3d(static_cast<double>(Highlevelctrl_vx_output_opt), 0.0, 0.0)
      + C_BW.col(2) * static_cast<double>(Highlevelctrl_height_output);
  Opt_w_fxfz_des_
      << T(desired_force_body.x()),
         T(desired_force_body.z()),
         Highlevelctrl_roll_output,
         Highlevelctrl_pitch_output,
         Highlevelctrl_yawrate_output_opt;

  // ProxQP는 double로 계산. //*바로 Equality로 들어가게 된다.
  Eigen::MatrixXd A =
      Opt_A_fxfz_des_.template cast<double>();

  Eigen::VectorXd w =
      Opt_w_fxfz_des_.template cast<double>();

  // ============================================================
  // 4. Cost function
  //
  // 0.5*f^T*W_force*f
  // + 0.5*lambda_rate*||f - f_prev||^2
  //
  // H = W_force + lambda_rate*I
  // g = -lambda_rate*f_prev
  // ============================================================

  const double weight_fx = 1.0;
  const double weight_fz = 1.0;
  const double lambda_rate = 1;

  // Weighting 고려 해주는 부분
  Eigen::MatrixXd W_force = Eigen::MatrixXd::Zero(8, 8);
  W_force.diagonal().head(4).setConstant(weight_fx);
  W_force.diagonal().tail(4).setConstant(weight_fz);

  Eigen::MatrixXd H =
      W_force
      + lambda_rate * Eigen::MatrixXd::Identity(8, 8);

  Eigen::VectorXd g =
      -lambda_rate * Opt_fx_fz_prev_.template cast<double>();

  // ============================================================
  // 5. Inequality: l <= C*f <= u
  //
  //  Fx=contact rolling-tangent force ft, Fz=contact-normal force fn [N].
  //  0~3행: Fx 범위
  //  4~7행: Fz 범위
  //  8~11행:  Fx - mu*Fz <= 0
  // 12~15행: -Fx - mu*Fz <= 0
  // ============================================================

  // 초기 시험용 설정. 튜닝 완료값은 아니다.
  const double mu = 0.8;
  const double fz_max = 200.0;  // 기존 Fz 상한 [N].

  // 마찰 제한과 Fz 상한으로 정해지는 최대 접선력.
  const double fx_max = mu * fz_max;

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(16, 8);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(16);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(16);

  // 앞 8행: 각 힘의 상하한.
  C.topRows(8).setIdentity();

  l.head(4).setConstant(-fx_max);
  u.head(4).setConstant( fx_max);

  l.segment(4, 4).setZero();
  u.segment(4, 4).setConstant(fz_max);

  for (int i = 0; i < 4; ++i)
  {
    // Fx_i - mu*Fz_i <= 0
    C(8 + i, i) = 1.0;
    C(8 + i, 4 + i) = -mu;

    // -Fx_i - mu*Fz_i <= 0
    C(12 + i, i) = -1.0;
    C(12 + i, 4 + i) = -mu;

    // 위 box bound에서 이미 보장되는 중복 하한.
    // 유한값을 사용하므로 아래 allFinite 검사와 함께 쓸 수 있다.
    l[8 + i] = -fx_max - mu * fz_max;
    l[12 + i] = -fx_max - mu * fz_max;

    u[8 + i] = 0.0;
    u[12 + i] = 0.0;
  }

  // ============================================================
  // 6. 입력 검사
  // ============================================================
  if (!H.allFinite() || !g.allFinite() ||
      !A.allFinite() || !w.allFinite() ||
      !C.allFinite() || !l.allFinite() || !u.allFinite())
  {
    std::cerr << "FxFz QP: NaN/Inf input\n";
    return;
  }

  // ============================================================
  // 7. Solver 초기화 / 갱신
  // Equality: A*f = w
  // Inequality: l <= C*f <= u
  // ============================================================
  if (!Opt_qp_fxfz_initialized_)
  {
    Opt_qp_fxfz_.settings.eps_abs = 1e-6;
    Opt_qp_fxfz_.settings.eps_rel = 0.0;

    Opt_qp_fxfz_.init(H, g, A, w, C, l, u);

    Opt_qp_fxfz_initialized_ = true;
  }
  else
  {
    Opt_qp_fxfz_.update(H, g, A, w, C, l, u);
  }

  Opt_qp_fxfz_.solve();

  // ============================================================
  // 8. Solver 결과 확인
  // ============================================================
  if (Opt_qp_fxfz_.results.info.status !=
      proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
  {
    std::cerr
        << "FxFz QP failed. status = "
        << static_cast<int>(Opt_qp_fxfz_.results.info.status)
        << '\n';
    std::cout
    << "FAILED desired wrench [Fx_B Fz_B Mx_B My_B Mz_B]: "
    << w.transpose() << '\n'
    << "A:\n" << A << '\n';

    return;
  }

  const auto& f_solution = Opt_qp_fxfz_.results.x;

  if (f_solution.size() != 8 || !f_solution.allFinite())
  {
    std::cerr << "FxFz QP: invalid solution\n";
    return;
  }

  // Check the load-side T representation and reconstructed forces before publishing.
  const Eigen::Matrix<T, 8, 1> force_solution = f_solution.template cast<T>();
  Vec3<T> force_body[4];
  if (!force_solution.allFinite()) {
    std::cerr << "FxFz QP: force conversion overflow\n";
    return;
  }
  for (int i = 0; i < 4; ++i) {
    force_body[i] = contact_tangent_body_[i] * force_solution[i]
                  + contact_normal_body_[i] * force_solution[4 + i];
    if (!force_body[i].allFinite()) {
      std::cerr << "FxFz QP: invalid Body contact force\n";
      return;
    }
  }

  // ============================================================
  // 9. 정상 결과 저장
  // ============================================================
  Opt_fxfz_des_ = force_solution;
  Opt_fx_fz_prev_ = Opt_fxfz_des_;
  for (int i = 0; i < 4; ++i) {
    contact_force_body_des_[i] = force_body[i];
  }

  Opt_fxfz_solution_valid_ = true;

  publish_opt_result();

  // ============================================================
  // 10. 출력 확인
  // ============================================================
  // std::cout
  //     << "Fx [N] FL FR RL RR: "
  //     << f_solution.head(4).transpose() << '\n'
  //     << "Fz [N] FL FR RL RR: "
  //     << f_solution.tail(4).transpose() << '\n'
  //     << "desired wrench: "
  //     << w.transpose() << '\n'
  //     << "QP wrench: "
  //     << (A * f_solution).transpose() << '\n'
  //     << "equality residual: "
  //     << (A * f_solution - w).transpose() << '\n';

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

  T kp_vx_opt = 483.938809;
  T kd_vx_opt = 0;
  T ki_vx_opt = 217.657403;

  T kp_gamma_opt = 1718.52239;
  T kp_roll_opt = 59.773875;
  T kp_pitch_opt = 250.437149;
  T kp_height_opt = 70526.5952;

  T kd_roll_opt = 150.552102;
  T kd_pitch_opt = 57.8921754;
  T kd_height_opt = 1574.2728;

  // T kp_vx_opt = 535.265304;
  // T kd_vx_opt = 0;
  // T ki_vx_opt = 184.656062;

  // T kp_gamma_opt = 331.870333;

  // T kp_roll_opt   = 116.271306;
  // T kp_pitch_opt  = 318.470148;
  // T kp_height_opt = 77107.084;

  // T kd_roll_opt   = 64.7244418;
  // T kd_pitch_opt  = 17.2376335;
  // T kd_height_opt = 518.876582;

  //! Algorithm for yaw rate , Longitudinal Control
  if (yawrate_mode == Mode::Kinematic)
  {
    //* Output: Yaw rate
    Highlevelctrl_yawrate_output_kin
      = chassis_traj_ptr_->chassis_angvel_des_[2]
      + kp_gamma_kin * (chassis_traj_ptr_->chassis_angvel_des_[2]
                    - robot_.body_omega_chassis_[2]);

  }
  else if (yawrate_mode == Mode::Optimization)
  {
    //* Output: Mz
    Highlevelctrl_yawrate_output_opt =
      kp_gamma_opt * (chassis_traj_ptr_->chassis_angvel_des_[2]
                    - robot_.body_omega_chassis_[2]);
  }
  else if (yawrate_mode == Mode::Both)
  {
    //* Output: Yaw rate
    Highlevelctrl_yawrate_output_kin = chassis_traj_ptr_->chassis_angvel_des_[2];

    //* Output: Mz
    Highlevelctrl_yawrate_output_opt =
      kp_gamma_opt * (chassis_traj_ptr_->chassis_angvel_des_[2]
                    - robot_.body_omega_chassis_[2]);

  }


  if (vx_mode == Mode::Kinematic)
  {
    //* Output: Vx
    Highlevelctrl_vx_output_kin
      = chassis_traj_ptr_->chassis_vel_des_[0]
      + kp_vx_kin * (chassis_traj_ptr_->chassis_vel_des_[0]
                 - robot_.body_vel_chassis_[0]);

  }
  else if (vx_mode == Mode::Optimization)
  {
    //* Output: Fx
    Highlevelctrl_vx_output_opt
      = kp_vx_opt * (chassis_traj_ptr_->chassis_vel_des_[0]
                 - robot_.body_vel_chassis_[0]);
  }
  else if (vx_mode == Mode::Both)
  {
    Highlevelctrl_vx_output_kin = chassis_traj_ptr_->chassis_vel_des_[0];

    const T Fx_FF = Highlevelctrl_vx_FF_opt.control(robot_.Total_mass_, 0,chassis_traj_ptr_->chassis_vel_des_[0], 10);

    const T Fx_limit = 100000;
    const T err_cut_off = T(20);

    const T Fx_raw = Highlevelctrl_vx_pid_opt.control(kp_vx_opt,ki_vx_opt,kd_vx_opt,chassis_traj_ptr_->chassis_vel_des_[0] , robot_.body_vel_chassis_[0],
      vx_aw_error_opt_, 20) + Fx_FF;

    const T Fx_sat = saturation_block(Fx_limit, -Fx_limit, Fx_raw);

    vx_aw_error_opt_ = vx_aw_opt.cal_saturation_error(Fx_raw-Fx_sat);

    Highlevelctrl_vx_output_opt = Fx_sat;

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
                    - robot_.body_euler_world_[0]) + kd_roll_opt *(chassis_traj_ptr_->chassis_angvel_des_[0] - robot_.body_omega_chassis_[0]);

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
                    - robot_.body_euler_world_[1]) + kd_pitch_opt * (chassis_traj_ptr_->chassis_angvel_des_[1] - robot_.body_omega_chassis_[1]);
    // std::cout << "Trunk My : " << Highlevelctrl_pitch_output << std::endl;
  }


  if (height_mode == Mode::Optimization)
  {
    // Four-point support-plane -> IMU vertical clearance [m], not world z.
    // Preserve the existing numerical reference and gains.
    const T height_des = T(0.2);
    const T height_rate_des = T(0);
    const auto& height = chassis_height_estimator_.result();
    //* Output: world vertical force Fz [N]
    Highlevelctrl_height_output = -robot_.Total_mass_ * robot_.gravity_
      + kp_height_opt * (height_des - T(height.height_m))
      + kd_height_opt * (height_rate_des - T(height.height_rate_mps));


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
  if (std::abs(Highlevelctrl_vx_output_kin) < T(1e-6) &&
      std::abs(Highlevelctrl_yawrate_output_kin) < T(1e-6))
  {
    for (int i = 0; i < 4; ++i)
    {
      joint_traj_ptr_->joint_pos_des_[i][1] = T(0);
      joint_traj_ptr_->joint_vel_des_[i][2] = T(0);
    }
    return;
  }


  T rov_ICR = Max(Highlevelctrl_vx_output_kin, 0.000000000001)
      / std::copysign(
          Max(abs(Highlevelctrl_yawrate_output_kin), 0.00000000001),
          Highlevelctrl_yawrate_output_kin);





  T wheel_ICR[4];

  wheel_ICR[0] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR-wheel_track_/2, 2));
  wheel_ICR[1] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR+wheel_track_/2, 2));
  wheel_ICR[2] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR-wheel_track_/2, 2));
  wheel_ICR[3] = sqrt(pow(wheel_base_/2, 2) + pow(rov_ICR+wheel_track_/2, 2));

  //! Algorithm for yaw rate , Longitudinal Control
  for (size_t i = 0; i < 4; i++)
  {
    joint_traj_ptr_->joint_vel_des_[i][2] =
        (Highlevelctrl_vx_output_kin
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

  const Mat3<T> C_BW =
    ori::quaternionToRotationMatrix(robot_.body_ang_quat_world_);

  const Vec3<T> vertical_chassis = C_BW.col(2);

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

    wheel_contact_pos_body[i] =
        wheel_pos_body[i] - wheel_radius_ * vertical_chassis;
  }

}

// 이전 구현 보존: 아래 새 구현과 중복 실행하지 않는다.

template <typename T>
void HighLevelController<T>::OptForce_2_Torque_sus()
{
  // Do not reinterpret old contact-coordinate forces using a new contact frame.
  if (!Opt_fxfz_solution_valid_) return;  // retain previous joint torque commands

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

    // 다리 자체의 중력보상 토크 [N*m]. 위의 수평 차체/평행링크 가정.
    // const T gravity_torque =
    //     g * (
    //         bar_mass * (T(0.08) * cos_q - T(0.025) * sin_q)
    //         + distal_mass * suspension_length * cos_q
    //     );

    const T gravity_torque = 0;

    // Same Body force vector as used by A: F_B = t_B*ft + n_B*fn.
    // Existing load-side sign convention; gravity compensation remains unchanged.
    const T force_torque = -J_sus_chassis.dot(contact_force_body_des_[i]);
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


// template <typename T>
// void HighLevelController<T>::OptForce_2_Torque_sus()
// {
//   // 이번 QP가 실패하면 이전 관절 토크를 유지한다.
//   // 이전 힘을 새 Contact frame과 섞어서 다시 변환하지 않는다.
//   if (!Opt_fxfz_solution_valid_) return;

//   const T side_sign[4] = {T(1), T(-1), T(1), T(-1)};  // FL, FR, RL, RR
//   const T suspension_length = T(0.16);               // [m], 기존 v1 기구학

//   for (int i = 0; i < 4; ++i)
//   {
//     // 1. 현재 load-side suspension 각도 [rad].
//     const T q_sus = robot_.joint_pos_act_[i][0];

//     // 2. 기존 접촉 위치 기구학을 q_sus로 미분한 Body Jacobian [m/rad].
//     //    조향각과 차체 자세는 고정하여 편미분한다.
//     Vec3<T> J_sus_body;
//     J_sus_body << T(0),
//         -side_sign[i] * suspension_length * std::sin(q_sus),
//          suspension_length * std::cos(q_sus);

//     // 3. QP 출력: A에서 가정한 프레임의 구름 접선력 ft, 법선력 fn [N].
//     //    현재 평지 가정에서는 fn이 World z 힘이며 Body z 힘과는 다르다.
//     const T force_tangent = Opt_fxfz_des_[i];
//     const T force_normal  = Opt_fxfz_des_[4 + i];

//     // 4. A 구성에 사용한 동일한 방향벡터로 Body 힘을 복원한다 [N].
//     //    현재는 평지 법선 기반 방향이 저장되어 있으므로 토크 변환도 평지 가정.
//     const Vec3<T> force_body =
//         contact_tangent_body_[i] * force_tangent
//         + contact_normal_body_[i] * force_normal;

//     // 5. 지면이 바퀴에 가하는 힘에 대응하는 actuator 토크 [N*m].
//     //    기존 부호 관례: tau_sus = -J_sus_body^T * force_body.
//     //    다리 자체 중력 보상은 기존처럼 비활성, damping은 main.cc에서 적용.
//     robot_.joint_torque_des_[i][0] = -J_sus_body.dot(force_body);
//   }
// }

template <typename T>
void HighLevelController<T>::OptForce_2_Torque_dri()
{
  if (!Opt_fxfz_solution_valid_) return;  // retain previous joint torque commands
  // 구동축 회전 방향이 좌우 반대이므로 부호 확인 필요 (실측으로 검증할 것)
  const T drive_sign[4] = {T(1), T(1), T(1), T(1)};

  for (int i = 0; i < 4; ++i)
  {
    // Ideal circular-wheel approximation: radius * local rolling-tangent force.
    // Wheel inertia/rolling resistance compensation is not added here.
    // Opt_fxfz_des_[i]: 구름 방향 접선력 [N] (조향각 및 A의 평지 법선 가정 반영)
    robot_.joint_torque_des_[i][2] =
        drive_sign[i] * Opt_fxfz_des_[i] * wheel_radius_;
  }

}

template <typename T>
void HighLevelController<T>::publish_opt_result()
{
  //! 순서: FL, FR, RL, RR / 단위: N
  //! Contact frame 성분: Fx=구름 접선력, Fz=법선력. Fy는 미최적화로 0.

  if (!higlctrl_opt_ptr_)
  {
    return;
  }

  for (int i = 0; i < 4; ++i)
  {
    higlctrl_opt_ptr_->Opt_Wheel_Fx_[i] = Opt_fxfz_des_(i);
    higlctrl_opt_ptr_->Opt_Wheel_Fy_[i] = T(0);
    higlctrl_opt_ptr_->Opt_Wheel_Fz_[i] = Opt_fxfz_des_(i+4);
  }
}

template <typename T>
std::shared_ptr<typename HighLevelController<T>::HighCtrl_Optimization> HighLevelController<T>::set_higlctrl_opt_ptr()
{
  return higlctrl_opt_ptr_;
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

template <typename T>
void HighLevelController<T>::get_estimate_ptr(
    std::shared_ptr<typename Estimate<T>::EstimateParam>
      estimate_param_ptr
)
{
  estimate_param_ptr_ = estimate_param_ptr;
}

template class HighLevelController<float>;
template class HighLevelController<double>;

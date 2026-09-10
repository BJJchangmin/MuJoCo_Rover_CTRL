#include "estimate.hpp"
#include "orientation_tools.hpp"
#include "Useful.hpp"

#include <iostream>

using namespace ori;
using namespace std;

template <typename T>
Estimate<T>::Estimate(RobotLeg<T> & robot) : robot_(robot)
{
  estimate_param_ptr_ = std::make_shared<EstimateParam>();
}

template <typename T>
void Estimate<T>::slip_ratio_estimate()
{
  T radius = 0.15; // 바퀴 반지름
  T Rover_vel_x;
  double wv[4];

  // Rover_vel_x = Max(robot_.body_vel_world_[0],0.001); // 초기 속도가 없을 때 0으로 나누는 것을 방지하기 위함
  Rover_vel_x = robot_.body_vel_world_[0];
  for(size_t i = 0; i < 4; i++)
  {
    wv[i] = robot_.joint_vel_act_[i][3];
    wheel_vel_[i] = max(radius*wv[i],0.00001); // reference에 반영이 되어 있어서 기어비 고려안해줘도 됨

    estimate_param_ptr_->slip_ratio_[i]
      = abs((wheel_vel_[i] - Rover_vel_x)/wheel_vel_[i]);

    // cout <<"slip_ratio : " << estimate_param_ptr_->slip_ratio_[0] << endl;
    estimate_param_ptr_->slip_ratio_[i] = saturation_block(1, -1, estimate_param_ptr_->slip_ratio_[i]); // 일단 전진만 한다는 가정
  }
  // cout << "Rover_vel_x : " << Rover_vel_x << endl;
  // cout << "wheel_vel : " << wheel_vel_[0] << endl;
}

template <typename T>
void Estimate<T>::cal_mu()
{
  /**
   * @brief : Lambda-mu 곡선 완벽히 구현 X 일단 stribeck model로 최대한 비슷하게 구현
   * @brief : Graph가 어떻게 그려지는 지는 Data 폴더 내의 Mu_Lambda.m 파일 참고
   */
  T e,T_brk,T_c,w_brk,w_st,w_coul,f,bias;
  e = 2.71828;
  T_brk = 0.8;
  T_c = 0.5;
  w_brk = 0.4;
  w_st = sqrt(2)*w_brk;
  w_coul = w_brk/10;
  f = -0.01;
  bias = 0;
  T S[4];

  slip_ratio_estimate();
  for(size_t i = 0; i < 4; i++)
  {
    S[i] = estimate_param_ptr_->slip_ratio_[i];

    estimate_param_ptr_->mu_[i] = sqrt(2)*e*(T_brk - T_c)*exp(-pow(S[i]/w_st,2))*(S[i]/ w_st) +
      T_c* tanh(S[i]/ w_coul) + f*S[i] + bias;

    estimate_param_ptr_->mu_[i] = max(0.001, estimate_param_ptr_->mu_[i]);

  }


}

template <typename T>
void Estimate<T>::est_GRF(const mjModel* m, const mjData * d)
{
  // for (size_t i = 0; i < 4; i++)
  // {
  //   // estimate_param_ptr_->grf_x_[i] = d->sensordata[49+3*i]/0.15;
  //   estimate_param_ptr_->grf_x_[i] = abs(robot_.foot_grf_world_[i][0]*sin(robot_.joint_pos_act_[i][3]) + robot_.foot_grf_world_[i][1]*cos(robot_.joint_pos_act_[i][3]));
  //   estimate_param_ptr_->grf_z_[i] = abs(robot_.foot_grf_world_[i][0]*cos(robot_.joint_pos_act_[i][3]) - robot_.foot_grf_world_[i][1]*sin(robot_.joint_pos_act_[i][3]));
  // }

  // FL, FR, RL, RR: 매 제어 주기마다 접촉력을 계산하고 저장한다.
  const char* wheel_body_names[4] = {
      "FL_wheel", "FR_wheel", "RL_wheel", "RR_wheel"};
  const char* wheel_labels[4] = {"FL", "FR", "RL", "RR"};

  // 현재 v1 framequat [w, x, y, z]. IMU 축은 chassis 축과 정렬되어 있다.
  // 접촉력과 같은 mjData에서 자세를 읽는다.
  Vec4<T> chassis_quat;
  for (int j = 0; j < 4; ++j) {
    chassis_quat[j] = T(d->sensordata[15 + j]);
  }
  // 이 프로젝트의 함수는 World -> Chassis 회전행렬을 반환한다.
  const Mat3<T> C_BW = ori::quaternionToRotationMatrix(chassis_quat);

  for (size_t i = 0; i < 4; ++i) {
    estimate_param_ptr_->grf_x_[i] = T(0);
    estimate_param_ptr_->grf_y_[i] = T(0);
    estimate_param_ptr_->grf_z_[i] = T(0);

    const int wheel_body_id =
        mj_name2id(m, mjOBJ_BODY, wheel_body_names[i]);
    if (wheel_body_id < 0) {
      std::cout << "[" << wheel_labels[i] << "] wheel body not found\n";
      continue;
    }

    int contact_count = 0;
    mjtNum force_world_sum[3] = {};

    for (int c = 0; c < d->ncon; ++c) {
      const mjContact& contact = d->contact[c];
      if (contact.efc_address < 0) continue;
      if (contact.geom[0] < 0 || contact.geom[1] < 0) continue;

      const int body0 = m->geom_bodyid[contact.geom[0]];
      const int body1 = m->geom_bodyid[contact.geom[1]];

      // 현재 지형은 world body(ID=0)에 속한다.
      // 지면이 해당 바퀴에 가하는 힘으로 부호를 통일한다.
      mjtNum sign = 0;
      if (body0 == 0 && body1 == wheel_body_id) {
        sign = 1;
      } else if (body0 == wheel_body_id && body1 == 0) {
        sign = -1;
      } else {
        continue;
      }
      ++contact_count;

      mjtNum contact_wrench[6] = {};
      mj_contactForce(m, d, c, contact_wrench);

      // 접촉 좌표계 -> World, 앞의 힘 성분 3개만 사용한다.
      mjtNum force_world[3] = {};
      mju_mulMatTVec(force_world, contact.frame, contact_wrench, 3, 3);
      for (int j = 0; j < 3; ++j) {
        force_world[j] *= sign;
        force_world_sum[j] += force_world[j];
      }

      const mjtNum normal_force =
          sign * contact.frame[0] * force_world[0] +
          sign * contact.frame[1] * force_world[1] +
          sign * contact.frame[2] * force_world[2];
    }

    Vec3<T> force_chassis = Vec3<T>::Zero();
    if (contact_count > 0) {
      Vec3<T> force_world;
      force_world << T(force_world_sum[0]),
                     T(force_world_sum[1]),
                     T(force_world_sum[2]);
      force_chassis = C_BW * force_world;
    }

    // 최종 저장: 부호 있는 chassis 기준 접촉력 [N].
    // 접촉이 없으면 0. abs()와 바퀴 구름각 보정은 적용하지 않는다.
    estimate_param_ptr_->grf_x_[i] = force_chassis[0];
    estimate_param_ptr_->grf_y_[i] = force_chassis[1];
    estimate_param_ptr_->grf_z_[i] = force_chassis[2];
  }
}

template <typename T>
std::shared_ptr<typename Estimate<T>::EstimateParam> Estimate<T>::set_estimate_param_ptr()
{
  return estimate_param_ptr_;
}




template class Estimate<float>;
template class Estimate<double>;

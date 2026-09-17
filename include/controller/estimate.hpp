#ifndef ESTIMATE_HPP_
#define ESTIMATE_HPP_

#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"
#include <mujoco/mujoco.h>

template <typename T>
class Estimate
{
  private:
    RobotLeg<T> & robot_;
    T wheel_vel_[4]; // [FL FR RL RR]




  public:
    struct EstimateParam
    {
      T slip_ratio_[4]; // [FL FR RL RR]
      double mu_[4]; // [FL FR RL RR]
      T grf_x_[4]; // [FL FR RL RR]
      T grf_y_[4]; // [FL FR RL RR]
      T grf_z_[4]; // [FL FR RL RR]

      // world 기준 접촉 법선 단위벡터: [nx, ny, nz]
      // 방향: 지면 -> 바퀴
      T contact_normal_world_[4][3] = {}; // 법선 벡터의 world 기준 나중에 chassis frame으로 돌려야 할듯

      T contact_pos_world_[4][3] = {}; //! 이거는 body frame에서 찾는게 더 적합하지 않나? 생각

      // 이번 주기에 유효한 대표 접촉을 찾았는가?
      bool contact_normal_valid_[4] = {};

    };

    std::shared_ptr<EstimateParam> estimate_param_ptr_;
    std::shared_ptr<EstimateParam> set_estimate_param_ptr();

  public:
    Estimate(RobotLeg<T> & robot);
    void slip_ratio_estimate();
    void cal_mu();
    void est_GRF(const mjModel* m, const mjData * d);
};

#endif  // ESTIMATE_HPP_

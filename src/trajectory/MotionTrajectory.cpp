#include "MotionTrajectory.hpp"

#include <cmath>
#include <iostream>
#include <random>

using namespace std;

template <typename T>
MotionTrajectory<T>::MotionTrajectory()
{
  NUM_SEGMENTS = 20;
  SEGMENT_DURATION = 20;
  signal_idx = 0;
  // loadDataFromFile("../data/ID_torque_input/input33_colvec.txt");
  // loadDataFromFile("../data/ID_torque_input/torque_input.txt");
  // loadDataFromFile("../data/ID_torque_input/torque_input2.txt");
  loadDataFromFile("../data/ID_torque_input/torque_100.txt");

  foot_traj_ptr_ = std::make_shared<DesiredFootTrajectory>();
  chassis_traj_ptr_ = std::make_shared<DesiredChassisTrajectory>();
  joint_traj_ptr_ = std::make_shared<DesiredJointTrajectory>();
}

template <typename T>
void MotionTrajectory<T>::driving_test(T time)
{
  T t = time;
  Vec4<T> Driving_M_Command;
  for(size_t i = 0; i < 4; i++)
  {
    //** Driving Test */
    if(t <= 5)
    {
      Driving_M_Command[i] = 0;
    }
    else if(5 < t <= 15)
    {
      Driving_M_Command[i] = (time-5)/40;
    }
    else if (15 < t <= 50)
    {
      Driving_M_Command[i] = 10.0/40;
    }


    joint_traj_ptr_->joint_pos_des_[i][0] = 0.0;//suspension Joint position reference

    joint_traj_ptr_->joint_pos_des_[i][1] = 0.0;//steer Joint position reference

    joint_traj_ptr_->joint_vel_des_[i][2] = Driving_M_Command[i]; // drive joint velocity reference [rad/s]



  }

}

template <typename T>
void MotionTrajectory<T>::suspension_test(T time)
{
  T f= 0.1;
  for(size_t i = 0; i < 4; i++)
  {
    //** Steering Test */
    // if(time <= 20)
    // {
    //   joint_traj_ptr_->joint_pos_des_[i][1] = 0.5*sin(2*M_PI*f*time); //steer Joint position reference
    // }
    // else if( 3 <= time <= 6)
    // {
    //   joint_traj_ptr_->joint_pos_des_[i][0] = 0.5*sin(2*M_PI*f*(time-3));
    // }

    //** Suspension Test */
    joint_traj_ptr_->joint_pos_des_[i][0] = 0.5*sin(2*M_PI*f*time);

  }
}


template <typename T>
void MotionTrajectory<T>::Slalom_traj(T time)
{
    // =============================================================================
    // ## [사용자 입력] 시뮬레이션 할 때마다 이 숫자만 변경하세요 (1 ~ 6) ##
    // =============================================================================

    int case_number = 1; //! Case 정해주자

    // =============================================================================
    // ## Case 별 파라미터 자동 할당 (Dynamic Cycles) ##
    // =============================================================================

    T target_vel  = 0.0;
    T target_freq = 0.0;
    T target_cycles  = 0;

    switch(case_number) {
        // [Low Speed: 0.25 m/s]
        case 1: //! 37 seconds
            target_vel = 0.3; target_freq = 0.1;
            target_cycles = 3;  // 30초 소요
            break;
        case 2: //! 19 seconds
            target_vel = 0.25*3; target_freq = 0.5;
            target_cycles = 6;  // 12초 소요
            break;
        case 3: //! 17 seconds
            target_vel = 0.25*4; target_freq = 1.0;
            target_cycles = 10; // 10초 소요
            break;

        // [High Speed: 0.50 m/s]
        case 4:
            target_vel = 0.50; target_freq = 0.1;
            target_cycles = 3;
            break;
        case 5:
            target_vel = 0.50; target_freq = 0.5;
            target_cycles = 6;
            break;
        case 6:
            target_vel = 0.50; target_freq = 1.0;
            target_cycles = 10;
            break;

        default: break;
    }

    // =============================================================================
    // ## 시간 및 궤적 자동 계산 ##
    // =============================================================================

    T settle_time = 5.0;              // 초기 직진 안정화
    T steer_amp   = 10.0 * (M_PI / 180.0); // 조향 진폭 15도

    // 주파수에 맞춰 필요한 시간만큼만 수행
    T slalom_duration = (T)target_cycles / target_freq;

    T t_start_slalom = settle_time;
    T t_end_slalom   = t_start_slalom + slalom_duration;

    // 로버 제원
    T L = 1.09; // Wheelbase (m)
    T current_Vx = 0.0;
    T current_Steer_Angle = 0.0;

    // --- 1. 초기 안정화 (Ramp Up & Straight) ---
    if (time < t_start_slalom)
    {
        T ramp_time = 1.0;
        if (time < ramp_time) current_Vx = (target_vel / ramp_time) * time;
        else current_Vx = target_vel;

        current_Steer_Angle = 0.0;
    }
    // --- 2. 슬라롬 주행 (Sine Wave) ---
    else if (time < t_end_slalom)
    {
        current_Vx = target_vel;

        // 로컬 시간 기준 사인파 생성
        T t_local = time - t_start_slalom;
        current_Steer_Angle = steer_amp * sin(2.0 * M_PI * target_freq * t_local);
    }
    // --- 3. 종료 ---
    else
    {
        current_Vx = 0.0;
        current_Steer_Angle = 0.0;
    }

    // --- Yaw Rate 변환 및 할당 ---
    T current_Omega = 0.0;
    if (abs(current_Vx) > 0.001) {
        current_Omega = (current_Vx * tan(current_Steer_Angle)) / L;
    }
    if (current_Vx < 0.0) current_Vx = 0.0;

    chassis_traj_ptr_->chassis_vel_des_[0]    = current_Vx; // Vy,Vz는 제어 안함
    chassis_traj_ptr_->chassis_angvel_des_[2] = current_Omega;

    chassis_traj_ptr_->chassis_pos_des_[2]    = 0; // IMU height [m]
    chassis_traj_ptr_->chassis_angpos_des_[0] = 0; // Roll Angle
    chassis_traj_ptr_->chassis_angpos_des_[1] = 0; // Pitch Angle

}

template <typename T>
void MotionTrajectory<T>::Orientation_traj(T time, T f , T roll_amp, T pitch_amp)
{


  //** Roll Ptch 각도 sin reference만 생성하고 나머지 값은 전부 0 */
  // Roll_amp, Pitch_amp는 deg라서 단위 변환 필요, f는 Hz로 들어옴

  chassis_traj_ptr_->chassis_vel_des_[0]    = 0; // Vy,Vz는 제어 안함
  chassis_traj_ptr_->chassis_angvel_des_[2] = 0;
  chassis_traj_ptr_->chassis_pos_des_[2]    = 0; // IMU height [m]

  if (time < 2)
  {
    chassis_traj_ptr_->chassis_angpos_des_[0] = 0.0; // Roll Angle
    chassis_traj_ptr_->chassis_angpos_des_[1] = 0.0; // Pitch Angle
  }
  else
  {
    chassis_traj_ptr_->chassis_angpos_des_[0] = roll_amp*M_PI/180.0*sin(2*M_PI*f*(time-2)); // Roll Angle
    chassis_traj_ptr_->chassis_angpos_des_[1] = pitch_amp*M_PI/180.0*sin(2*M_PI*f*(time-2)); // Pitch Angle
  }


}

template <typename T>
T MotionTrajectory<T>::motor_ID(T time)
{
  start_time = 3;


  if (time < 0.003) // first time step
  {
    RandomFreqs();
  }
  // RandomFreqs();

  double ex_time = time - start_time;
  int current_segment = static_cast<int>(std::floor(ex_time / SEGMENT_DURATION));

  double seg_freq = freqs[current_segment];
  double seg_amp = amps[current_segment];
  double seg_phase = phases[current_segment];

  if (time < 3)
  {
    return 0;
  }
  else
  {
    double torque = seg_amp * sin(2 * M_PI * seg_freq * ex_time + seg_phase);
    return torque;
  }


}

template <typename T>
T MotionTrajectory<T>::system_ID(T time)
{
  double output;
  cout << "time: " << time << endl;
  cout << "signal_idx: " << signal_idx << endl;
  if (time < 3 || signal_idx >= 30000)
  {
    return 0;
  }
  else
  {
    output = shreder_ID_data_[signal_idx];
    cout << "signal_idx: " << output << endl;
    signal_idx = signal_idx + 1;
    return output;

  }



}

template <typename T>
void MotionTrajectory<T>::RandomFreqs()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> freqDist(0.1, 100.0);
  std::uniform_real_distribution<double> ampDist(0.1, 0.5);
  std::uniform_real_distribution<double> phaseDist(0.0, 2.0 * M_PI);

  for (int i = 0; i < NUM_SEGMENTS; i++)
  {
    freqs[i] = freqDist(gen);
    amps[i] = ampDist(gen);
    phases[i] = phaseDist(gen);
  }

}

template <typename T>
void MotionTrajectory<T>::loadDataFromFile(const char* filename)
{
  double DATA_SIZE = 30000;

  FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Error opening file\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_SIZE; i++) {
        if (fscanf(file, "%f", &shreder_ID_data_[i]) != 1)
        {
          cout <<"hello" << endl;
          perror("Error opening file\n");
          fclose(file);
          exit(EXIT_FAILURE);
        }
    }

    fclose(file);

}


template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> MotionTrajectory<T>::set_foot_traj_ptr()
{
  return foot_traj_ptr_;
}

template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredChassisTrajectory> MotionTrajectory<T>::set_chassis_traj_ptr()
{
  return chassis_traj_ptr_;
}

template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> MotionTrajectory<T>::set_joint_traj_ptr()
{
  return joint_traj_ptr_;
}






template class MotionTrajectory<float>;
template class MotionTrajectory<double>;

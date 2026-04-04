#ifndef FWIS_ROBOT_CONTROLLER_HPP_
#define FWIS_ROBOT_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "fwis_robot/kinematics.hpp"

struct Command {
    double steering_ang;    // [deg]
    double wheel_vel;       // [m/s]
};

class Controller
{
public:
    struct Params 
    {
        // geometry
        double wheel_radius;
        double wheel_x_offset;
        double wheel_y_offset;
        double wheel_max_rpm;           // [rpm] 인휠 모터 최대 rpm

        // [cmd_vel] step 스무딩 파라미터
        bool use_rate_limit;
        double bound_cmd_speed;
        double add_cmd_speed;
        double bound_cmd_ang_speed;
        double add_cmd_ang_speed;
        
        // [cmd_vel] 가속도 제한 파라미터
        double max_linear_accel;
        double max_angular_accel;

        // LPF/limit
        double lpf_linear;
        double lpf_angular;
        double deadzone_linear;
        double deadzone_angular;
        double limit_linear_abs;
        double limit_angular_abs;

        // 조향 사다리꼴 프로파일 (모터 프로파일)
        double steer_max_velocity;
        double steer_max_accel;
        double steer_position_tolerance;

        // 인휠 가속/감속 프로파일 (모터 프로파일)
        double inwheel_max_accel;
        double inwheel_max_decel;
        double inwheel_min_rpm;

        // 조향-구동 싱크
        bool wait_for_steer;
        bool drive_scale_by_steer_err;
        double steer_err_full_drive;
        double steer_err_zero_drive;
    };

    Controller();
    explicit Controller(const Params& params);

    void SetParams(const Params& params);
    const Params& GetParams() const { return params_; }

    // cmd_vel 콜백에서 호출 (목표값 저장)
    void SetReference(double vx, double vy, double wz);

    // 현재 바퀴 상태 및 모터 명령
    std::vector<Command> Update(const std::vector<WheelState>& current_states, double dt);
    
    void Reset();

private:
    // [cmd_vel] step 간격만큼 이동
    static void StepSmoothing(double ref, double& cmd, double bound, double add);
    // [cmd_vel] max_accel * dt 만큼 변화
    static void RateLimitSmoothing(double dt, double ref, double& cmd, double max_accel);
    // 조향 사다리꼴 프로파일
    double SteerProfile(int i, double target_ang, double current_ang, double dt);
    // 인휠 가속/감속 프로파일
    double InwheelProfile(int i, double target_vel, double dt);
    // 조향-구동 싱크
    double ComputeSteerSyncScale(double steer_error_deg) const;

    // 유틸
    static double Clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }
    static double ApplyDeadzone(double x, double dz) {
        return (std::fabs(x) < dz) ? 0.0 : x;
    }
    static double Sign(double x) {
        return (x > 1e-9) ? 1.0 : (x < -1e-9 ? -1.0 : 0.0);
    }
    static double MsToRpm(double v, double r) {
        return v * 60.0 / (2.0 * M_PI * r);
    }
    static double RpmToMs(double rpm, double r) {
        return rpm * 2.0 * M_PI * r / 60.0;
    }

private:
    Params params_;
    Kinematics kinematics_;

    // reference
    double ref_vx_ = 0.0;
    double ref_vy_ = 0.0;
    double ref_wz_ = 0.0;

    // LPF 출력
    double filt_vx_ = 0.0;
    double filt_vy_ = 0.0;
    double filt_wz_ = 0.0;

    // [cmd_vel] 스무딩 결과
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_wz_ = 0.0;

    // 바퀴별 조향 프로파일 속도 상태 [deg/s]
    std::array<double, 4> steer_profile_vel_{};

    // 바퀴별 조향 명령 누적 위치 [deg]
    // 피드백 없이도 SteerProfile이 위치를 추적할 수 있도록 내부에서 관리
    std::array<double, 4> steer_cmd_pos_{};

    // 바퀴별 인휠 명령 속도 상태 [m/s]
    std::array<double, 4> inwheel_cmd_vel_{};
};

#endif // FWIS_ROBOT_CONTROLLER_HPP_
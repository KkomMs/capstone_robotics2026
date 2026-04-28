#include "fwis_robot/controller.hpp"

Controller::Controller()
: Controller(Params{})
{}

Controller::Controller(const Params& params)
{
    SetParams(params);
    Reset();
}

void Controller::SetParams(const Params& params)
{
    params_ = params;

    // 안전 sanity
    if (params_.limit_linear_abs < 0.0) params_.limit_linear_abs = 0.0;
    if (params_.limit_angular_abs < 0.0) params_.limit_angular_abs = 0.0;
    if (params_.steer_err_zero_drive <= params_.steer_err_full_drive)
        params_.steer_err_zero_drive = params_.steer_err_full_drive + 1.0;

    // Kinematics - geometry 파라미터 동기화
    ::Params kin;
    kin.wheel_radius = params_.wheel_radius;
    kin.wheel_x_offset = params_.wheel_x_offset;
    kin.wheel_y_offset = params_.wheel_y_offset;
    kin.deadzone_linear = params_.deadzone_linear;
    kin.deadzone_angular = params_.deadzone_angular;
    kinematics_.SetParams(kin);
}

void Controller::Reset()
{
    ref_vx_ = ref_vy_ = ref_wz_ = 0.0;
    filt_vx_ = filt_vy_ = filt_wz_ = 0.0;
    cmd_vx_ = cmd_vy_ = cmd_wz_ = 0.0;
    steer_profile_vel_.fill(0.0);
    steer_cmd_pos_.fill(0.0);
    inwheel_cmd_vel_.fill(0.0);
    kinematics_.Reset();
}

void Controller::SetReference(double vx, double vy, double wz)
{
    ref_vx_ = Clamp(vx, -params_.limit_linear_abs, params_.limit_linear_abs);
    ref_vy_ = Clamp(vy, -params_.limit_linear_abs, params_.limit_linear_abs);
    ref_wz_ = Clamp(wz, -params_.limit_angular_abs, params_.limit_angular_abs);
}

void Controller::StepSmoothing(double ref, double& cmd, double bound, double add)
{
    if (cmd + bound < ref)
        cmd += add;
    else if (cmd - bound > ref)
        cmd -= add;
    else
        cmd = 0.8 * cmd + 0.2 * ref;
}

void Controller::RateLimitSmoothing(double dt, double ref, double& cmd, double max_accel)
{
    const double delta = ref - cmd;
    const double max_step = max_accel * dt;
    cmd += Clamp(delta, -max_step, max_step);
}

// 메인 제어 함수
std::vector<Command> Controller::Update(const std::vector<WheelState>& current_states, double dt)
{
    std::vector<Command> cmds(4);
    if (current_states.size() != 4 || dt <= 0.0) return cmds;

    // [1] deadzone
    double rx = ApplyDeadzone(ref_vx_, params_.deadzone_linear);
    double ry = ApplyDeadzone(ref_vy_, params_.deadzone_linear);
    double rw = ApplyDeadzone(ref_wz_, params_.deadzone_angular);

    // [2] LPF
    if (params_.lpf_linear > 0.0) {
        filt_vx_ += params_.lpf_linear * (rx - filt_vx_);
        filt_vy_ += params_.lpf_linear * (ry - filt_vy_);
    } else {
        filt_vx_ = rx;
        filt_vy_ = ry;
    }
    if (params_.lpf_angular > 0.0)
        filt_wz_ += params_.lpf_angular * (rw - filt_wz_);
    else
        filt_wz_ = rw;

    // [3] (cmd_vel) 스무딩
    if (params_.use_rate_limit) {   // 가속도 제한
        RateLimitSmoothing(dt, filt_vx_, cmd_vx_, params_.max_linear_accel);
        RateLimitSmoothing(dt, filt_vy_, cmd_vy_, params_.max_linear_accel);
        RateLimitSmoothing(dt, filt_wz_, cmd_wz_, params_.max_angular_accel);
    } else {    // step 스무딩
        StepSmoothing(filt_vx_, cmd_vx_, params_.bound_cmd_speed, params_.add_cmd_speed);
        StepSmoothing(filt_vy_, cmd_vy_, params_.bound_cmd_speed, params_.add_cmd_speed);
        StepSmoothing(filt_wz_, cmd_wz_, params_.bound_cmd_ang_speed, params_.add_cmd_ang_speed);
    }

    // [4] 속도 클램프
    cmd_vx_ = Clamp(cmd_vx_, -params_.limit_linear_abs, params_.limit_linear_abs);
    cmd_vy_ = Clamp(cmd_vy_, -params_.limit_linear_abs, params_.limit_linear_abs);
    cmd_wz_ = Clamp(cmd_wz_, -params_.limit_angular_abs, params_.limit_angular_abs);

    // [5] IK
    std::vector<double> current_steer(4);
    for (int i = 0; i < 4; i++) current_steer[i] = steer_cmd_pos_[i];       // 실제 모터값 대신 cmd 값 사용

    // 제자리 회전은 속도 높이기
    double rf_vx = cmd_vx_;
    double rf_vy = cmd_vy_;
    double rf_wz = cmd_wz_;
    if (std::fabs(cmd_vx_) < params_.deadzone_linear && std::fabs(cmd_vy_) < params_.deadzone_linear && std::fabs(cmd_wz_) >= params_.deadzone_angular) {
        rf_wz = cmd_wz_ * 1.5;
        rf_wz = Clamp(rf_wz, -params_.limit_angular_abs, params_.limit_angular_abs);
    }

    std::vector<WheelState> ik = kinematics_.InverseKinematics(rf_vx, rf_vy, rf_wz, current_steer);

    // [6-8] 프로파일 & 모터 동기화
    for (int i = 0; i < 4; i++) {
        const double target_steer = ik[i].steering_ang;
        const double current_ang = steer_cmd_pos_[i];               // [조향 프로파일] 실제 모터값 대신 cmd 값 사용
        const double actual_ang = current_states[i].steering_ang;   // [구동 동기화] 실제 피드백 사용
        
        // option 1. cmd 기준 오차
        const double steer_err_cmd = std::fabs(target_steer - steer_cmd_pos_[i]);
        // option 2. 모터 피드백 기준 오차
        const double steer_err_fb = std::fabs(target_steer - actual_ang);

        // [6] 조향 사다리꼴 프로파일
        double cmd_steer = SteerProfile(i, target_steer, current_ang, dt);

        // [7] 조향 - 구동 동기화 (구동 모터는 조향 오차에 비례하여 서서히 출력)
        double target_drive = ik[i].wheel_vel;

        if (params_.wait_for_steer) {   // 조향이 완료된 후 모터 구동
            if (steer_err_fb > params_.steer_position_tolerance) {
                target_drive = 0.0;
            } else if (params_.drive_scale_by_steer_err) {
                target_drive *= ComputeSteerSyncScale(steer_err_fb);
            }
        } else if (params_.drive_scale_by_steer_err) {  // 조향과 상관없이 모터 구동
            target_drive *= ComputeSteerSyncScale(steer_err_fb);
        }

        // [8] 인휠 가속/감속 프로파일
        double cmd_drive = InwheelProfile(i, target_drive, dt);

        cmds[i].steering_ang = cmd_steer;
        cmds[i].wheel_vel = cmd_drive;
    }

    return cmds;
}

// 조향 모터 trapezoidal 속도 프로파일
// current_ang : 모터 명령 각도 [deg]
double Controller::SteerProfile(int i, double target_ang, double current_ang, double dt)
{
    const double max_vel   = params_.steer_max_velocity;
    const double max_accel = params_.steer_max_accel;
    const double tol       = params_.steer_position_tolerance;

    double& pos      = steer_cmd_pos_[i];
    double& prof_vel = steer_profile_vel_[i];

    double error = target_ang - pos;
    double dist  = std::fabs(error);

    if (dist < tol) {
        prof_vel = 0.0;
        pos      = target_ang;
        return pos;
    }

    double sign = Sign(error);

    // 제동 거리 v^2 / (2a)
    double braking_dist = (prof_vel * prof_vel) / (2.0 * max_accel);

    double new_vel;
    if (dist > braking_dist) {
        new_vel = std::min(prof_vel + max_accel * dt, max_vel);
    } else {
        new_vel = std::max(prof_vel - max_accel * dt, 0.0);
    }

    new_vel  = std::min(new_vel, std::sqrt(2.0 * max_accel * dist));
    new_vel  = std::min(new_vel, max_vel);
    prof_vel = new_vel;

    double step = sign * new_vel * dt;
    if (std::fabs(step) >= dist) {  // 오버슈트 방지
        prof_vel = 0.0;
        pos      = target_ang;
        return pos;
    }

    pos += step;
    return pos;
}

// 인휠 모터 가속/감속 프로파일
double Controller::InwheelProfile(int i, double target_vel, double dt)
{
    const double r = params_.wheel_radius;
    const double max_rpm = params_.wheel_max_rpm;

    double target_rpm = MsToRpm(target_vel, r);
    double current_rpm = MsToRpm(inwheel_cmd_vel_[i], r);

    target_rpm = Clamp(target_rpm, -max_rpm, max_rpm);
    if (std::fabs(target_rpm) < params_.inwheel_min_rpm) target_rpm = 0.0;

    const double diff = target_rpm - current_rpm;
    double max_step;

    // 방향 전환 or 속도 크기 감소 -> 감속
    // 속도 증가 -> 가속
    const bool reversing = (Sign(target_rpm) != Sign(current_rpm) && (current_rpm != 0.0));
    const bool speeding_up = !reversing && (std::fabs(target_rpm) > std::fabs(current_rpm));

    max_step = (speeding_up ? params_.inwheel_max_accel : params_.inwheel_max_decel) * dt;

    const double new_rpm = current_rpm + Clamp(diff, -max_step, max_step);
    inwheel_cmd_vel_[i] = RpmToMs(new_rpm, r);
    return inwheel_cmd_vel_[i];
}

// 조향 - 구동 모터 동기화
double Controller::ComputeSteerSyncScale(double steer_error_deg) const
{
    const double full = params_.steer_err_full_drive;
    const double zero = params_.steer_err_zero_drive;

    if (steer_error_deg <= full) return 1.0;
    if (steer_error_deg >= zero) return 0.0;

    return 1.0 - (steer_error_deg - full) / (zero - full);
}
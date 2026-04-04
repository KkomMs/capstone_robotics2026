#include "fwis_robot/kinematics.hpp"

// 구동 모터: 바퀴 선속도 명령 [m/s]
// 조향 모터: 바퀴 각도 명령 [deg]
Kinematics::Kinematics()
: Kinematics(Params{})
{}

Kinematics::Kinematics(const Params& params)
{
    SetParams(params);
    Reset();
}

void Kinematics::SetParams(const Params& params)
{
    params_ = params;
}

void Kinematics::Reset()
{
    pose_ = Pose{};
}

std::pair<double, double> Kinematics::GetWheelPos(int i) const
{
    const double lx = params_.wheel_x_offset;
    const double ly = params_.wheel_y_offset;
    switch (i)
    {
    case 0: return {lx, ly};       // [FL]
    case 1: return {lx, -ly};        // [FR]
    case 2: return {-lx, ly};      // [RL]
    case 3: return {-lx, -ly};       // [RR]
    default: return {0.0, 0.0};
    }
}

RobotVelocity Kinematics::ForwardKinematics(const std::vector<WheelState>& wheel_states, double dt)
{
    RobotVelocity vel{0.0, 0.0, 0.0};

    // 값이 모두 넘어왔는지 검사
    if (wheel_states.size() != 4) return vel;

    double sum_vx = 0.0;
    double sum_vy = 0.0;
    double sum_wz = 0.0;

    double r2 = params_.wheel_x_offset * params_.wheel_x_offset + params_.wheel_y_offset * params_.wheel_y_offset;
    // FK 계산
    for (int i = 0; i < 4; i++) {
        auto [rx, ry] = GetWheelPos(i);
        double v_linear = wheel_states[i].wheel_vel;        // [todo] 모터에서 받아오는 값이 선속도인지 각속도인지 확인하기
        double theta_rad = wheel_states[i].steering_ang * M_PI / 180.0;

        double v_xi = v_linear * std::cos(theta_rad);
        double v_yi = v_linear * std::sin(theta_rad);

        sum_vx += v_xi;
        sum_vy += v_yi;
        sum_wz += (v_yi * rx - v_xi * ry) / r2;
    }

    vel.vx = sum_vx / 4.0;
    vel.vy = sum_vy / 4.0;
    vel.wz = sum_wz / 4.0;

    pose_.vx = vel.vx;
    pose_.vy = vel.vy;
    pose_.wz = vel.wz;

    // Pose 계산
    if (dt > 0.0001) {
        double delta_x = (vel.vx * std::cos(pose_.theta) - vel.vy * std::sin(pose_.theta)) * dt;
        double delta_y = (vel.vx * std::sin(pose_.theta) + vel.vy * std::cos(pose_.theta)) * dt;
        double delta_theta = vel.wz * dt;

        pose_.x += delta_x;
        pose_.y += delta_y;
        pose_.theta += delta_theta;

        pose_.theta = NormalizeAngle(pose_.theta);      // [rad]
    }

    return vel;
}

std::vector<WheelState> Kinematics::InverseKinematics(double linear_x, double linear_y, double angular, const std::vector<double>& current_steering_angles)
{
    std::vector<WheelState> commands(4);

    // deadzone 처리
    if (std::abs(linear_x) < params_.deadzone_linear && std::abs(linear_y) < params_.deadzone_linear && std::abs(angular) < params_.deadzone_angular) {
        for (int i = 0; i < 4; i++) {
            // 조향값이 전부 다 넘어왔는지 검사
            commands[i].steering_ang = (current_steering_angles.size() == 4) ? current_steering_angles[i] : 0.0;
            commands[i].wheel_vel = 0.0;
        }
        return commands;
    }

    // IK 계산
    for (int i = 0; i < 4; i++) {
        auto [rx, ry] = GetWheelPos(i);

        // V_wheel = V_robot + Omega x R_vector
        double wheel_vx = linear_x - angular * ry;
        double wheel_vy = linear_y + angular * rx;

        double target_speed = std::sqrt(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
        double target_angle = std::atan2(wheel_vy, wheel_vx);

        double current_angle_deg = (current_steering_angles.size() == 4) ? current_steering_angles[i] : 0.0;
        double current_angle_rad = current_angle_deg * M_PI / 180.0;

        // 조향각 최적화
        double ang_diff = NormalizeAngle(target_angle - current_angle_rad);

        if (std::abs(ang_diff) > M_PI / 2.0) {
            if (ang_diff > 0.0) {
                ang_diff -= M_PI;
            } else {
                ang_diff += M_PI;
            }
            target_speed *= -1.0;   // 구동 바퀴 방향 뒤집기
        }

        double final_angle = current_angle_rad + ang_diff;

        // 조향각을 -pi ~ pi 사이로 clamping
        if (final_angle > M_PI || final_angle < -M_PI) {
            if (final_angle > M_PI) {
                final_angle -= M_PI;
            } else {
                final_angle += M_PI;
            }
            target_speed *= -1.0;
        }
        
        final_angle = std::max(-M_PI, std::min(M_PI, final_angle));     // 최종 clamp

        commands[i].wheel_vel = target_speed;   // [m/s]
        commands[i].steering_ang = (current_angle_rad + ang_diff) * 180.0 / M_PI;   // [deg]
    }

    return commands;
}
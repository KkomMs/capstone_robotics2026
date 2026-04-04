#include "fwis_robot/camera.hpp"

#include <cmath>
#include <algorithm>

// [todo] 추후에 수정하기.

// ==========================================================
//  생성자
// ==========================================================
CameraController::CameraController()
    : CameraController(CameraParams{})
{}

CameraController::CameraController(const CameraParams& params)
{
    SetParams(params);
    Reset();
}

// ==========================================================
//  파라미터
// ==========================================================
void CameraController::SetParams(const CameraParams& params)
{
    params_ = params;

    // min < max 보장
    if (params_.min_angle_deg > params_.max_angle_deg)
        std::swap(params_.min_angle_deg, params_.max_angle_deg);

    // 속도 음수 방지
    if (params_.sweep_velocity_dps < 0.0)
        params_.sweep_velocity_dps = std::fabs(params_.sweep_velocity_dps);
}

void CameraController::Reset()
{
    mode_ = CameraMode::IDLE;
    goto_target_.fill(0.0);

    for (auto& ax : axes_) {
        ax.current_angle_deg = 0.0;
        ax.command_angle_deg = 0.0;
        ax.sweep_direction   = 1;
    }
}

// ==========================================================
//  외부 인터페이스
// ==========================================================
void CameraController::SetMode(CameraMode mode)
{
    mode_ = mode;
}

void CameraController::SetGoalAngle(int axis, double angle_deg)
{
    if (axis < 0 || axis >= CAMERA_AXIS_COUNT) return;
    goto_target_[axis] = Clamp(angle_deg, params_.min_angle_deg, params_.max_angle_deg);
}

void CameraController::SetSweepVelocity(double vel_dps)
{
    params_.sweep_velocity_dps = std::fabs(vel_dps);
}

void CameraController::UpdateFeedback(int axis, double angle_deg)
{
    if (axis < 0 || axis >= CAMERA_AXIS_COUNT) return;
    axes_[axis].current_angle_deg = angle_deg;
}

// ==========================================================
//  메인 Update
// ==========================================================
std::array<CameraCommand, CAMERA_AXIS_COUNT> CameraController::Update(double dt)
{
    std::array<CameraCommand, CAMERA_AXIS_COUNT> cmds{};

    if (dt <= 0.0 || dt > 1.0) dt = 0.02;  // 기본 50 Hz 가정

    for (int i = 0; i < CAMERA_AXIS_COUNT; i++) {
        cmds[i].motor_id = params_.motor_ids[i];

        switch (mode_) {
        case CameraMode::SWEEP:
            ProcessSweep(i, dt, cmds[i]);
            break;

        case CameraMode::GOTO:
            ProcessGoto(i, dt, cmds[i]);
            break;

        case CameraMode::IDLE:
        default:
            // 현재 명령 각도를 유지 (모터 홀드)
            cmds[i].target_angle_deg = axes_[i].command_angle_deg;
            cmds[i].velocity_dps     = 0.0;
            break;
        }
    }

    return cmds;
}

// ==========================================================
//  균일 속도 한 스텝 이동
//  매 tick마다 velocity * dt [deg] 만큼 target 방향으로
//  일정하게 이동한다. 오버슈트 시 target에 스냅.
// ==========================================================
double CameraController::MoveConstantVelocity(int axis, double target_deg, double dt)
{
    auto& ax = axes_[axis];

    const double error = target_deg - ax.command_angle_deg;
    const double dist  = std::fabs(error);

    // 이미 도달
    if (dist < params_.position_tolerance_deg) {
        return target_deg;
    }

    // 이동 방향: error의 부호
    const double dir = (error > 0.0) ? 1.0 : -1.0;

    // 이번 tick 이동량 (균일 속도)
    const double step = params_.sweep_velocity_dps * dt;

    // 오버슈트 방지: 남은 거리보다 step이 크면 target에 스냅
    if (step >= dist) {
        return target_deg;
    }

    // 범위 클램핑 (하드 리밋)
    return Clamp(ax.command_angle_deg + dir * step,
                 params_.min_angle_deg,
                 params_.max_angle_deg);
}

// ==========================================================
//  Sweep 모드: min ↔ max 균일 속도 왕복
// ==========================================================
void CameraController::ProcessSweep(int axis, double dt, CameraCommand& out)
{
    auto& ax = axes_[axis];

    // 현재 sweep 방향에 따른 끝단
    const double target = (ax.sweep_direction > 0)
                              ? params_.max_angle_deg
                              : params_.min_angle_deg;

    double next_angle = MoveConstantVelocity(axis, target, dt);

    // 끝단 도달 → 방향 반전
    const double remain = std::fabs(target - next_angle);
    if (remain < params_.position_tolerance_deg) {
        ax.sweep_direction *= -1;
        next_angle = Clamp(next_angle, params_.min_angle_deg, params_.max_angle_deg);
    }

    ax.command_angle_deg = next_angle;

    out.target_angle_deg = next_angle;
    out.velocity_dps     = params_.sweep_velocity_dps;
}

// ==========================================================
//  Goto 모드: 목표 각도에 균일 속도로 이동, 도달 시 IDLE
// ==========================================================
void CameraController::ProcessGoto(int axis, double dt, CameraCommand& out)
{
    auto& ax = axes_[axis];
    const double target = goto_target_[axis];

    double next_angle = MoveConstantVelocity(axis, target, dt);

    ax.command_angle_deg = next_angle;

    // 도달 확인
    const double remain = std::fabs(target - next_angle);
    if (remain < params_.position_tolerance_deg) {
        // 모든 축이 도달했는지 확인 후 IDLE 전환
        bool all_done = true;
        for (int i = 0; i < CAMERA_AXIS_COUNT; i++) {
            if (std::fabs(goto_target_[i] - axes_[i].command_angle_deg) >= params_.position_tolerance_deg) {
                all_done = false;
                break;
            }
        }
        if (all_done) {
            mode_ = CameraMode::IDLE;
        }
    }

    out.target_angle_deg = next_angle;
    out.velocity_dps     = params_.sweep_velocity_dps;
}
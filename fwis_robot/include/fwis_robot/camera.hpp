#ifndef FWIS_ROBOT_CAMERA_HPP_
#define FWIS_ROBOT_CAMERA_HPP_

#include <array>
#include <cmath>
#include <cstdint>

// ==========================================================
//  카메라 모터 (Dynamixel) 제어
//  - 전 구간 균일 속도 왕복 (프로파일 없음)
//  - 추후: goto, tracking 등 모드 확장
// ==========================================================

// [todo] 대강 러프한 코드. 추후 수정

// ─────────── 모터 ID 매핑 ───────────
enum CameraAxis : int {
    CAMERA_LEFT  = 0,
    CAMERA_RIGHT = 1,
    CAMERA_AXIS_COUNT = 2
};

// ─────────── 동작 모드 ───────────
enum class CameraMode {
    IDLE,       // 정지 (현재 위치 유지)
    SWEEP,      // min_angle ↔ max_angle 균일 속도 왕복
    GOTO,       // 목표 각도로 1회 이동 후 IDLE 전환
};

// ─────────── 단일 축 명령 출력 ───────────
struct CameraCommand {
    uint8_t  motor_id;          // Dynamixel ID
    double   target_angle_deg;  // [deg] 목표 위치
    double   velocity_dps;      // [deg/s] 이동 속도 (양수)
};

// ─────────── 파라미터 ───────────
struct CameraParams {
    // 모터 ID
    std::array<uint8_t, CAMERA_AXIS_COUNT> motor_ids = {0, 1};

    // 동작 범위 [deg]
    double min_angle_deg = -40.0;
    double max_angle_deg = 90.0;

    // 균일 이동 속도 [deg/s]
    double sweep_velocity_dps = 60.0;

    // 도달 판정 임계 [deg]
    double position_tolerance_deg = 1.0;
};

// ─────────── 단일 축 내부 상태 ───────────
struct CameraAxisState {
    double current_angle_deg = 0.0;     // 현재 각도 (피드백)
    double command_angle_deg = 0.0;     // 마지막 출력 각도
    int    sweep_direction   = 1;       // +1: max 방향, -1: min 방향
};

// ─────────── 카메라 컨트롤러 ───────────
class CameraController {
public:
    CameraController();
    explicit CameraController(const CameraParams& params);

    void SetParams(const CameraParams& params);
    const CameraParams& GetParams() const { return params_; }

    // --- 외부 인터페이스 ---
    void SetMode(CameraMode mode);
    CameraMode GetMode() const { return mode_; }

    // GOTO 모드용: 목표 각도 설정
    void SetGoalAngle(int axis, double angle_deg);

    // sweep 속도 런타임 변경
    void SetSweepVelocity(double vel_dps);

    // 모터 피드백 갱신 (통신 콜백에서 호출)
    void UpdateFeedback(int axis, double angle_deg);

    // 메인 제어 루프에서 매 tick 호출
    std::array<CameraCommand, CAMERA_AXIS_COUNT> Update(double dt);

    void Reset();

private:
    // 균일 속도로 target까지 한 스텝 이동. 새 명령 각도를 반환.
    double MoveConstantVelocity(int axis, double target_deg, double dt);

    void ProcessSweep(int axis, double dt, CameraCommand& out);
    void ProcessGoto(int axis, double dt, CameraCommand& out);

    static double Clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

private:
    CameraParams params_;
    CameraMode   mode_ = CameraMode::IDLE;

    std::array<CameraAxisState, CAMERA_AXIS_COUNT> axes_;
    std::array<double, CAMERA_AXIS_COUNT> goto_target_ = {0.0, 0.0};
};

#endif // FWIS_ROBOT_CAMERA_HPP_
#ifndef FWIS_ROBOT_KINEMATICS_HPP_
#define FWIS_ROBOT_KINEMATICS_HPP_

#include <vector>
#include <cmath>
#include <utility>

struct Params
{
    double wheel_radius;
    double wheel_x_offset;
    double wheel_y_offset;
    double deadzone_linear;
    double deadzone_angular;
};

struct Pose
{   
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct RobotVelocity {
    double vx;
    double vy;
    double wz;
};

// 각 바퀴의 상태 및 명령
struct WheelState {
    double steering_ang;    // [deg]
    double wheel_vel;       // [m/s]    [todo] 선속도인지 각속도인지 확인하기
};

class Kinematics {
public:
    Kinematics();
    explicit Kinematics(const Params& params);

    void SetParams(const Params& params);
    const Params& GetParams() const { return params_; }

    // FK (모터 상태 -> 로봇 pose)
    RobotVelocity ForwardKinematics(const std::vector<WheelState>& wheel_states, double dt);

    // IK (cmd_vel -> 모터 명령)
    std::vector<WheelState> InverseKinematics(double linear_x, double linear_y, double angular, const std::vector<double>& current_steering_angles);

    const Pose& GetPose() const { return pose_; }
    void Reset();

private:
    static inline double NormalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    std::pair<double, double> GetWheelPos(int i) const;

private:
    Params params_;
    Pose   pose_;
};

#endif // FWIS_ROBOT_KINEMATICS_HPP_
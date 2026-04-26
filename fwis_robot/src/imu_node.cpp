// ─────────────────────────────────────────────────────────────
//  iAHRS IMU 센서 데이터 수신 및 ROS2 토픽 발행 노드
//
//  기능:
//    - iAHRS 센서 Sync Data 모드 자동 설정
//    - 가속도, 각속도, Euler angle 수신 → /imu/data 토픽 발행
//    - sensor_msgs/Imu 메시지 형식 (robot_localization 호환)
//
//  토픽:
//    Published:
//      /imu/data  (sensor_msgs/Imu)  50Hz
// ─────────────────────────────────────────────────────────────
#include <memory>
#include <string>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "fwis_robot/imu_driver.hpp"

namespace {
template<typename T>
T GetParam(const rclcpp::Node& node, const std::string& name)
{
    T value{};
    if (!node.get_parameter(name, value)) {
        throw std::runtime_error(
            "Required parameter '" + name + "' is not set. "
            "Set it via launch/YAML (parameters=[...]).");
    }
    return value;
}
}  // namespace


class ImuNode : public rclcpp::Node
{
public:
    explicit ImuNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("imu_node", options)
    {
        LoadParams();
        SetupSensor();
        SetupRos();

        RCLCPP_INFO(get_logger(),
            "[ImuNode] Started. port=%s baud=%d frame=%s sync=%dms",
            config_.port.c_str(), config_.baudrate,
            imu_frame_id_.c_str(), config_.sync_period_ms);
    }

    ~ImuNode() override
    {
        // sync 전송 중지 후 닫기
        if (driver_.IsOpen()) {
            driver_.SendCommand("sd=0");
            driver_.Close();
        }
    }

private:
    // ==========================================================
    //  파라미터 로드
    // ==========================================================
    void LoadParams()
    {
        config_.port           = GetParam<std::string>(*this, "imu_port");
        config_.baudrate       = GetParam<int>(*this, "imu_baudrate");
        imu_frame_id_          = GetParam<std::string>(*this, "imu_frame_id");
        config_.sync_port      = GetParam<int>(*this, "sync_port");
        config_.sync_period_ms = GetParam<int>(*this, "sync_period_ms");
        publish_rate_          = GetParam<double>(*this, "imu_publish_rate");

        // sync data 마스크: 가속도(0x004) + 각속도(0x008) + Euler(0x040) = 0x04C
        config_.sync_data_mask = imu::SD_ACCEL | imu::SD_GYRO | imu::SD_EULER;

        RCLCPP_INFO(get_logger(),
            "IMU params: port=%s baud=%d sync_port=%d period=%dms rate=%.1fHz mask=0x%03X",
            config_.port.c_str(), config_.baudrate,
            config_.sync_port, config_.sync_period_ms,
            publish_rate_, config_.sync_data_mask);
    }

    // ==========================================================
    //  센서 설정
    // ==========================================================
    void SetupSensor()
    {
        if (!driver_.Open(config_.port, config_.baudrate)) {
            RCLCPP_ERROR(get_logger(),
                "[IMU] Failed to open serial port: %s", config_.port.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[IMU] Serial port opened: %s", config_.port.c_str());

        // 센서 버전 확인
        std::string ver = driver_.SendAndReceive("vr", 500);
        if (!ver.empty()) {
            RCLCPP_INFO(get_logger(), "[IMU] Sensor info: %s", ver.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "[IMU] No response to version query. Sensor may need reset.");
        }

        // Sync Data 모드 설정
        if (!driver_.ConfigureSensor(config_)) {
            RCLCPP_ERROR(get_logger(), "[IMU] Failed to configure sensor");
            return;
        }
        RCLCPP_INFO(get_logger(), "[IMU] Sync Data mode configured (period=%dms, mask=0x%03X)",
            config_.sync_period_ms, config_.sync_data_mask);
    }

    // ==========================================================
    //  ROS 인터페이스 설정
    // ==========================================================
    void SetupRos()
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 50);

        // 수신 타이머: sync 주기보다 빠르게 폴링 (5ms)
        rx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&ImuNode::PollAndPublish, this));
    }

    // ==========================================================
    //  주기적 수신 및 발행
    // ==========================================================
    void PollAndPublish()
    {
        if (!driver_.IsOpen()) {
            // 재연결 시도 (5초마다)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[IMU] Port closed. Attempting reconnect...");
            if (driver_.Open(config_.port, config_.baudrate)) {
                driver_.ConfigureSensor(config_);
                RCLCPP_INFO(get_logger(), "[IMU] Reconnected.");
            }
            return;
        }

        // 한 번의 폴링에서 가능한 모든 줄 처리
        std::string line;
        while (driver_.ReadLine(line))
        {
            imu::ImuData data;
            if (!driver_.ParseSyncData(line, config_.sync_data_mask, data)) {
                // 설정 응답 등 sync data가 아닌 줄은 무시
                continue;
            }

            PublishImu(data);
        }
    }

    // ==========================================================
    //  IMU 메시지 생성 및 발행
    // ==========================================================
    void PublishImu(const imu::ImuData& data)
    {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp    = this->get_clock()->now();
        msg.header.frame_id = imu_frame_id_;

        // --- Orientation (Euler -> Quaternion) ---
        if (data.euler_valid) {
            tf2::Quaternion q;
            q.setRPY(data.roll, data.pitch, data.yaw);
            msg.orientation.x = q.x();
            msg.orientation.y = q.y();
            msg.orientation.z = q.z();
            msg.orientation.w = q.w();

            // 공분산 (iAHRS 정지 상태 정밀도 기반 추정)
            msg.orientation_covariance[0] = 0.0025;   // roll  variance
            msg.orientation_covariance[4] = 0.0025;   // pitch variance
            msg.orientation_covariance[8] = 0.01;     // yaw   variance
        } else {
            // orientation 데이터 없음
            msg.orientation_covariance[0] = -1.0;
        }

        // --- Angular velocity [rad/s] ---
        if (data.gyro_valid) {
            msg.angular_velocity.x = data.gx;
            msg.angular_velocity.y = data.gy;
            msg.angular_velocity.z = data.gz;

            msg.angular_velocity_covariance[0] = 0.001;
            msg.angular_velocity_covariance[4] = 0.001;
            msg.angular_velocity_covariance[8] = 0.001;
        } else {
            msg.angular_velocity_covariance[0] = -1.0;
        }

        // --- Linear acceleration [m/s²] ---
        if (data.accel_valid) {
            msg.linear_acceleration.x = data.ax;
            msg.linear_acceleration.y = data.ay;
            msg.linear_acceleration.z = data.az;

            msg.linear_acceleration_covariance[0] = 0.01;
            msg.linear_acceleration_covariance[4] = 0.01;
            msg.linear_acceleration_covariance[8] = 0.01;
        } else {
            msg.linear_acceleration_covariance[0] = -1.0;
        }

        imu_pub_->publish(msg);

        // [Debug]
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "[IMU] euler(deg): r=%.2f p=%.2f y=%.2f | gyro(dps): %.2f %.2f %.2f | acc(m/s²): %.2f %.2f %.2f",
            data.roll * 180.0 / M_PI, data.pitch * 180.0 / M_PI, data.yaw * 180.0 / M_PI,
            data.gx * 180.0 / M_PI, data.gy * 180.0 / M_PI, data.gz * 180.0 / M_PI,
            data.ax, data.ay, data.az);
    }

    // ==========================================================
    //  멤버 변수
    // ==========================================================
    imu::ImuDriver  driver_;
    imu::SensorConfig config_;

    std::string imu_frame_id_;
    double      publish_rate_ = 50.0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr rx_timer_;
};


// ==========================================================
//  main
// ==========================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ImuNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
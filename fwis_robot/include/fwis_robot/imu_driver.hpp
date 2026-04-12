#ifndef FWIS_ROBOT_IMU_DRIVER_HPP_
#define FWIS_ROBOT_IMU_DRIVER_HPP_

#include <string>
#include <vector>
#include <functional>
#include <cstdint>

// ─────────────────────────────────────────────────────────────
//  iAHRS Driver
//  ROBOR iAHRS 센서 시리얼 통신 드라이버
//
//  지원 모드:
//    - Sync data 수신 (센서가 주기적으로 전송하는 CSV 데이터 파싱)
//    - 커맨드 송수신  (초기 설정, 상태 조회 등)
//
//  통신 프로토콜:
//    TX: "오브젝트명=값\n" 또는 "오브젝트명\n"
//    RX: "오브젝트명=값1,값2,...\n"
//    Sync data: "값1,값2,값3,...\n"  (sd 설정에 따라 CSV 형식)
// ─────────────────────────────────────────────────────────────

namespace imu {

// Sync data 비트 마스크 (sd 오브젝트)
enum SyncDataBit : uint16_t {
    SD_TIME     = 0x001,  // 1ms count [ms]
    SD_TEMP     = 0x002,  // 온도 [°C]
    SD_ACCEL    = 0x004,  // 센서 좌표계 가속도 [g]  (ax, ay, az)
    SD_GYRO     = 0x008,  // 센서 좌표계 각속도 [deg/s] (gx, gy, gz)
    SD_MAGN     = 0x010,  // 센서 좌표계 지자기 [μT]
    SD_ACCEL_G  = 0x020,  // 전역 좌표계 중력 제거 가속도 [m/s²]
    SD_EULER    = 0x040,  // Euler angle [deg] (roll, pitch, yaw)
    SD_QUAT     = 0x080,  // Quaternion (r, v0, v1, v2)
    SD_VEL      = 0x100,  // 전역 좌표계 속도 [m/s]
    SD_POS      = 0x200,  // 전역 좌표계 위치 [m]
    SD_FREQ     = 0x400,  // 진동 주파수/크기
};

// Sync data 전송 포트 선택
enum SyncPort : int {
    SYNC_PORT_RS232    = 0,
    SYNC_PORT_USB_UART = 1,
};

// 파싱된 IMU 데이터
struct ImuData
{
    // 가속도 [m/s²] (센서 좌표계)
    double ax = 0.0, ay = 0.0, az = 0.0;
    bool   accel_valid = false;

    // 각속도 [rad/s] (센서 좌표계)
    double gx = 0.0, gy = 0.0, gz = 0.0;
    bool   gyro_valid = false;

    // Euler angle [rad]
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    bool   euler_valid = false;
};

// 센서 설정 파라미터
struct SensorConfig
{
    std::string port       = "/dev/ttyIMU";
    int         baudrate   = 115200;
    int         sync_port  = SYNC_PORT_USB_UART;  // USB 연결이므로 1
    int         sync_period_ms = 20;              // 50Hz
    uint16_t    sync_data_mask = SD_ACCEL | SD_GYRO | SD_EULER;  // 0x04C
};

class ImuDriver
{
public:
    ImuDriver();
    ~ImuDriver();

    // 시리얼 포트 열기/닫기
    bool Open(const std::string& port, int baudrate);
    void Close();
    bool IsOpen() const;

    // 센서 초기 설정 (sync data 모드 구성)
    bool ConfigureSensor(const SensorConfig& config);

    // 센서에 텍스트 커맨드 전송
    bool SendCommand(const std::string& cmd);

    // 센서에 커맨드 전송 후 응답 수신 (타임아웃 ms)
    std::string SendAndReceive(const std::string& cmd, int timeout_ms = 200);

    // 수신 버퍼에서 줄 단위로 읽기 (non-blocking)
    // 완전한 줄이 있으면 true 반환, line에 결과 저장
    bool ReadLine(std::string& line);

    // Sync data CSV 줄을 파싱
    // sync_data_mask에 따라 필드 순서가 결정됨
    bool ParseSyncData(const std::string& line, uint16_t sync_data_mask, ImuData& out);

    // 센서 yaw 리셋
    bool ResetAngle();

    // 센서 재시작
    bool RestartDevice();

private:
    int fd_ = -1;  // 시리얼 파일 디스크립터
    std::string read_buffer_;

    // sync_data_mask에서 활성화된 필드 개수 계산
    static int CountSyncFields(uint16_t mask);

    // CSV 문자열을 double 벡터로 분리
    static std::vector<double> SplitCsv(const std::string& line);
};

}  // namespace imu

#endif  // FWIS_ROBOT_IMU_DRIVER_HPP_
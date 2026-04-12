#include "fwis_robot/imu_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>

namespace imu {

// ─────────────────────────────────────────────────────────────
//  생성자 / 소멸자
// ─────────────────────────────────────────────────────────────
ImuDriver::ImuDriver() = default;

ImuDriver::~ImuDriver()
{
    Close();
}

// ─────────────────────────────────────────────────────────────
//  시리얼 포트 열기
// ─────────────────────────────────────────────────────────────
bool ImuDriver::Open(const std::string& port, int baudrate)
{
    Close();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    // baudrate 매핑
    speed_t baud;
    switch (baudrate) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default:     baud = B115200; break;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        Close();
        return false;
    }

    // 8N1, no flow control
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~PARENB;        // no parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // enable read, ignore modem control

    // raw 모드
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // non-blocking read
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        Close();
        return false;
    }

    read_buffer_.clear();
    return true;
}

void ImuDriver::Close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    read_buffer_.clear();
}

bool ImuDriver::IsOpen() const
{
    return fd_ >= 0;
}

// ─────────────────────────────────────────────────────────────
//  센서 초기 설정 (Sync Data 모드)
//
//  설정 순서:
//    1. 기존 sync 전송 중지 (sd=0)
//    2. sync 포트 설정 (so=1)
//    3. sync 주기 설정 (sp=20)
//    4. sync 데이터 선택 (sd=0x04C)
//    5. 지자기 보정 끄기 (mv=0)
//    6. 설정 저장 (fw)
// ─────────────────────────────────────────────────────────────
bool ImuDriver::ConfigureSensor(const SensorConfig& config)
{
    if (!IsOpen()) return false;

    // 잠시 대기 (센서 부팅 대기)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 수신 버퍼 비우기
    {
        char buf[512];
        while (::read(fd_, buf, sizeof(buf)) > 0) {}
    }

    // 1) 기존 sync 중지
    SendCommand("sd=0");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 2) sync 포트 설정
    SendCommand("so=" + std::to_string(config.sync_port));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 3) sync 주기 설정
    SendCommand("sp=" + std::to_string(config.sync_period_ms));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 4) 지자기 보정 끄기 (모터 근처 환경)
    SendCommand("mv=0");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 5) Euler angle 리셋
    SendCommand("ra");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 6) sync 데이터 선택 (이 시점부터 데이터 전송 시작)
    {
        std::ostringstream oss;
        oss << "sd=0x" << std::hex << std::uppercase << config.sync_data_mask;
        SendCommand(oss.str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 7) 플래시 저장 (전원 꺼져도 유지)
    SendCommand("fw");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
}

// ─────────────────────────────────────────────────────────────
//  커맨드 송수신
// ─────────────────────────────────────────────────────────────
bool ImuDriver::SendCommand(const std::string& cmd)
{
    if (!IsOpen()) return false;

    std::string msg = cmd + "\r\n";
    ssize_t written = ::write(fd_, msg.c_str(), msg.size());
    return (written == static_cast<ssize_t>(msg.size()));
}

std::string ImuDriver::SendAndReceive(const std::string& cmd, int timeout_ms)
{
    if (!SendCommand(cmd)) return "";

    // 타임아웃 기반 수신
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    std::string response;
    while (std::chrono::steady_clock::now() < deadline)
    {
        std::string line;
        if (ReadLine(line)) {
            // "오브젝트=값" 형태의 응답인지 확인
            if (!line.empty() && line.find('=') != std::string::npos) {
                return line;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return "";
}

// ─────────────────────────────────────────────────────────────
//  줄 단위 수신 (non-blocking)
// ─────────────────────────────────────────────────────────────
bool ImuDriver::ReadLine(std::string& line)
{
    if (!IsOpen()) return false;

    // 시리얼에서 읽기
    char buf[512];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
        read_buffer_.append(buf, static_cast<size_t>(n));
    }

    // 줄바꿈 찾기 (\r\n 또는 \n)
    auto pos = read_buffer_.find('\n');
    if (pos == std::string::npos) {
        // 버퍼 과대 방지
        if (read_buffer_.size() > 4096) {
            read_buffer_.clear();
        }
        return false;
    }

    // 줄 추출
    line = read_buffer_.substr(0, pos);
    read_buffer_.erase(0, pos + 1);

    // \r 제거
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    return !line.empty();
}

// ─────────────────────────────────────────────────────────────
//  Sync Data CSV 파싱
//
//  iAHRS sync data 필드 순서 (sd 비트 순서대로):
//    SD_TIME(0x001)  → 1 필드  [ms]
//    SD_TEMP(0x002)  → 1 필드  [°C]
//    SD_ACCEL(0x004) → 3 필드  [g]   (ax, ay, az)
//    SD_GYRO(0x008)  → 3 필드  [deg/s] (gx, gy, gz)
//    SD_MAGN(0x010)  → 3 필드  [μT]
//    SD_ACCEL_G(0x020) → 3 필드 [m/s²]
//    SD_EULER(0x040) → 3 필드  [deg] (roll, pitch, yaw)
//    SD_QUAT(0x080)  → 4 필드  (r, v0, v1, v2)
//    SD_VEL(0x100)   → 3 필드  [m/s]
//    SD_POS(0x200)   → 3 필드  [m]
//    SD_FREQ(0x400)  → 6 필드  [Hz, magnitude]
//
//  설정 예시: sd=0x04C → ACCEL(3) + GYRO(3) + EULER(3) = 9 필드
// ─────────────────────────────────────────────────────────────
bool ImuDriver::ParseSyncData(const std::string& line, uint16_t mask, ImuData& out)
{
    // "오브젝트=" 형태의 응답이면 sync data가 아님 (초기 설정 응답 등)
    if (line.find('=') != std::string::npos) {
        return false;
    }

    auto values = SplitCsv(line);
    if (values.empty()) {
        return false;
    }

    // 필드 순서대로 읽기
    size_t idx = 0;
    auto consume = [&](int count) -> bool {
        if (idx + count > values.size()) return false;
        idx += count;
        return true;
    };
    auto val = [&](int offset) -> double {
        return values[idx + offset];
    };

    // SD_TIME (0x001)
    if (mask & SD_TIME) {
        if (!consume(1)) return false;
    }

    // SD_TEMP (0x002)
    if (mask & SD_TEMP) {
        if (!consume(1)) return false;
    }

    // SD_ACCEL (0x004) - [g] → [m/s²]
    if (mask & SD_ACCEL) {
        if (idx + 3 > values.size()) return false;
        out.ax = val(0) * 9.80665;  // g → m/s²
        out.ay = val(1) * 9.80665;
        out.az = val(2) * 9.80665;
        out.accel_valid = true;
        idx += 3;
    }

    // SD_GYRO (0x008) - [deg/s] → [rad/s]
    if (mask & SD_GYRO) {
        if (idx + 3 > values.size()) return false;
        constexpr double DEG2RAD = M_PI / 180.0;
        out.gx = val(0) * DEG2RAD;
        out.gy = val(1) * DEG2RAD;
        out.gz = val(2) * DEG2RAD;
        out.gyro_valid = true;
        idx += 3;
    }

    // SD_MAGN (0x010)
    if (mask & SD_MAGN) {
        if (!consume(3)) return false;
    }

    // SD_ACCEL_G (0x020)
    if (mask & SD_ACCEL_G) {
        if (!consume(3)) return false;
    }

    // SD_EULER (0x040) - [deg] → [rad]
    if (mask & SD_EULER) {
        if (idx + 3 > values.size()) return false;
        constexpr double DEG2RAD = M_PI / 180.0;
        out.roll  = val(0) * DEG2RAD;
        out.pitch = val(1) * DEG2RAD;
        out.yaw   = val(2) * DEG2RAD;
        out.euler_valid = true;
        idx += 3;
    }

    // SD_QUAT (0x080)
    if (mask & SD_QUAT) {
        if (!consume(4)) return false;
    }

    // SD_VEL (0x100)
    if (mask & SD_VEL) {
        if (!consume(3)) return false;
    }

    // SD_POS (0x200)
    if (mask & SD_POS) {
        if (!consume(3)) return false;
    }

    // SD_FREQ (0x400)
    if (mask & SD_FREQ) {
        if (!consume(6)) return false;
    }

    return (out.accel_valid || out.gyro_valid || out.euler_valid);
}

bool ImuDriver::ResetAngle()
{
    return SendCommand("ra");
}

bool ImuDriver::RestartDevice()
{
    return SendCommand("rd");
}

// ─────────────────────────────────────────────────────────────
//  유틸리티
// ─────────────────────────────────────────────────────────────
int ImuDriver::CountSyncFields(uint16_t mask)
{
    int count = 0;
    if (mask & SD_TIME)    count += 1;
    if (mask & SD_TEMP)    count += 1;
    if (mask & SD_ACCEL)   count += 3;
    if (mask & SD_GYRO)    count += 3;
    if (mask & SD_MAGN)    count += 3;
    if (mask & SD_ACCEL_G) count += 3;
    if (mask & SD_EULER)   count += 3;
    if (mask & SD_QUAT)    count += 4;
    if (mask & SD_VEL)     count += 3;
    if (mask & SD_POS)     count += 3;
    if (mask & SD_FREQ)    count += 6;
    return count;
}

std::vector<double> ImuDriver::SplitCsv(const std::string& line)
{
    std::vector<double> result;
    std::istringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ',')) {
        // 공백 제거
        token.erase(std::remove(token.begin(), token.end(), ' '), token.end());
        if (token.empty()) continue;

        try {
            result.push_back(std::stod(token));
        } catch (...) {
            // 숫자 변환 실패 → sync data가 아닌 줄
            return {};
        }
    }
    return result;
}

}  // namespace imu
#!/usr/bin/env python3
"""
stm32_bridge.py  (피드포워드 + PID 속도 제어 버전)
─────────────────────────────────────────────────────────────────
피드백 토픽:
  /steer_fb/front   Float32MultiArray  [degL, degR]
  /steer_fb/rear    Float32MultiArray  [degL, degR]
  /encoder/front    Int32MultiArray    [encL, encR]
  /encoder/rear     Int32MultiArray    [encL, encR]
  /wheel_vel/front  Float32MultiArray  [vL, vR]  m/s
  /wheel_vel/rear   Float32MultiArray  [vL, vR]  m/s

명령 토픽:
  /motor_N/inwheel  Float32  m/s 단위로 입력
  /motor_N/steer    Float32  -180.0 ~ 180.0 deg
  /motor/cmd        String   STOP / ZERO / AUTO / MAN ...

속도 제어 구조:
  duty = feedforward + PID보정
  feedforward = 목표속도 / MAX_WHEEL_VEL_MS
  PID보정     = KP*err + KI*int(err) + KD*filtered_derr/dt

수정 사항:
  1) 정지 바이패스: target≈0 → duty=0 즉시 + 버퍼 클리어
  2) 방향 전환 감지 → PID 리셋 + 버퍼 클리어 + 블랭킹 타이머
  3) 블랭킹 타이머: STM32 방향전환 상태머신(400ms) 동안
     PID를 멈추고 피드포워드만 출력 → 적분/미분 오염 방지
  4) 미분 로우패스 필터: 스텝 입력 시 derivative 스파이크 억제
─────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32MultiArray, Float32MultiArray

import serial
import threading
import time
import os


# ──────────────────────────────────────────────────────────────
# 인휠 속도 계산 상수
# ──────────────────────────────────────────────────────────────
FG_PULSES_PER_REV = 60.0
WHEEL_DIAMETER_M  = 0.1397
WHEEL_CIRCUMF_M   = 3.14159265 * WHEEL_DIAMETER_M

MAX_WHEEL_VEL_MS = 1.4


# ──────────────────────────────────────────────────────────────
# PID 게인 (기존 값 유지)
# ──────────────────────────────────────────────────────────────
KP = 0.5
KI = 0.26
KD = 0.1
INTEGRAL_LIMIT     = 0.30
DERIV_FILTER_ALPHA = 0.3

VEL_BUF_SIZE = 4

# STM32 방향전환 상태머신 소요 시간 (300ms 브레이크 + 100ms 대기)
# 여유를 줘서 0.5초로 설정
DIR_CHANGE_BLANKING_SEC = 0.80


# ──────────────────────────────────────────────────────────────
# XOR 체크섬
# ──────────────────────────────────────────────────────────────
def append_checksum(cmd: str) -> str:
    xor = 0
    for b in cmd.encode():
        xor ^= b
    return f'{cmd}*{xor:02X}'


def _sign(x: float) -> int:
    if x > 1e-4:
        return 1
    elif x < -1e-4:
        return -1
    return 0


# ──────────────────────────────────────────────────────────────
# WheelPID
# ──────────────────────────────────────────────────────────────
class WheelPID:
    def __init__(self, kp=KP, ki=KI, kd=KD, max_vel=MAX_WHEEL_VEL_MS):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.max_vel = max_vel

        self.target         = 0.0
        self.integral       = 0.0
        self.prev_err       = 0.0
        self.filtered_deriv = 0.0
        self.duty           = 0.0

        self._prev_sign      = 0
        self.need_buf_clear  = False

        # 블랭킹: 방향 전환 후 일정 시간 PID 비활성
        self._blanking_until = 0.0

    def reset(self):
        self.integral       = 0.0
        self.prev_err       = 0.0
        self.filtered_deriv = 0.0
        self.duty           = 0.0

    def set_target(self, vel_ms: float):
        new_sign = _sign(vel_ms)
        old_sign = self._prev_sign

        self.target = vel_ms

        # 목표 0 → 전부 리셋 + 버퍼 클리어
        if new_sign == 0:
            self.reset()
            self.need_buf_clear = True
            self._prev_sign = 0
            return

        # 방향 전환 → PID 리셋 + 버퍼 클리어 + 블랭킹 시작
        if old_sign != 0 and new_sign != old_sign:
            self.reset()
            self.need_buf_clear = True
            self._blanking_until = time.monotonic() + DIR_CHANGE_BLANKING_SEC

        self._prev_sign = new_sign

    def update(self, actual_vel: float, dt: float) -> float:
        if dt <= 0:
            return self.duty

        # 정지 바이패스
        if abs(self.target) < 1e-4:
            self.duty           = 0.0
            self.integral       = 0.0
            self.prev_err       = 0.0
            self.filtered_deriv = 0.0
            return 0.0

        # 피드포워드
        feedforward = self.target / self.max_vel

        # 블랭킹 중이면 피드포워드만 출력, PID 안 돌림
        # STM32가 방향 전환하는 동안 피드백이 엉뚱하므로
        # PID가 적분/미분을 쌓지 않게 함
        now = time.monotonic()
        if now < self._blanking_until:
            # self.duty = max(-1.0, min(1.0, feedforward))
            # # prev_err를 현재 오차로 세팅해서 블랭킹 끝날 때
            # # derivative가 튀지 않게 준비
            # self.prev_err = self.target - actual_vel
            # return self.duty
            self.duty = 0.0
            self.prev_err = 0.0
            self.integral = 0.0
            self.filtered_deriv = 0.0
            return 0.0

        # PID
        err = self.target - actual_vel

        self.integral += err * dt
        self.integral  = max(-INTEGRAL_LIMIT,
                             min(INTEGRAL_LIMIT, self.integral))

        raw_deriv = (err - self.prev_err) / dt
        self.filtered_deriv = (DERIV_FILTER_ALPHA * raw_deriv
                               + (1.0 - DERIV_FILTER_ALPHA) * self.filtered_deriv)
        self.prev_err = err

        pid_out = (self.kp * err
                   + self.ki * self.integral
                   + self.kd * self.filtered_deriv)

        self.duty = max(-1.0, min(1.0, feedforward + pid_out))

        return self.duty


# ──────────────────────────────────────────────────────────────
# BoardLink
# ──────────────────────────────────────────────────────────────
class BoardLink:
    def __init__(self, port: str, baud: int, label: str, logger):
        self.port   = port
        self.baud   = baud
        self.label  = label
        self.logger = logger

        self._ser       = None
        self._stop      = threading.Event()
        self._connected = False

        self.RECONNECT_INTERVAL = 2.0
        self._init_cmd    = None
        self._fb_callback = None

    def _try_open(self) -> bool:
        if not os.path.exists(self.port):
            return False
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._ser       = ser
            self._connected = True
            self.logger.info(f'[{self.label}] 연결됨 ({self.port})')
            if self._init_cmd:
                time.sleep(0.5)
                ser.write((self._init_cmd + '\n').encode())
            return True
        except serial.SerialException as e:
            self.logger.warn(f'[{self.label}] 열기 실패: {e}')
            return False

    def _force_close(self):
        ser = self._ser
        self._ser       = None
        self._connected = False
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def write_line(self, cmd: str) -> bool:
        ser = self._ser
        if ser is None:
            return False
        try:
            line = append_checksum(cmd) + '\n'
            ser.write(line.encode())
            return True
        except Exception as e:
            self.logger.warn(f'[{self.label}] 전송 오류: {e}')
            self._force_close()
            return False

    def flush(self):
        ser = self._ser
        if ser:
            try:
                ser.flush()
            except Exception:
                pass

    def run_reader(self):
        while not self._stop.is_set() and not self._connected:
            if not self._try_open():
                self.logger.warn(
                    f'[{self.label}] {self.port} 없음, '
                    f'{self.RECONNECT_INTERVAL:.0f}초 후 재시도...')
                time.sleep(self.RECONNECT_INTERVAL)

        buf = b''
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                time.sleep(self.RECONNECT_INTERVAL)
                self.logger.info(f'[{self.label}] 재연결 시도...')
                self._try_open()
                buf = b''
                continue

            try:
                waiting = ser.in_waiting
                data    = ser.read(waiting if waiting > 0 else 1)
                if data:
                    buf += data
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        text = line.decode('utf-8', errors='replace').strip()
                        if text:
                            if text.startswith('FB ') and self._fb_callback:
                                self._fb_callback(text)
                            else:
                                self.logger.info(f'[{self.label}] {text}')
            except serial.SerialException as e:
                self.logger.warn(f'[{self.label}] 연결 끊김: {e}')
                self._force_close()
                buf = b''
                time.sleep(0.5)
            except Exception as e:
                self.logger.warn(f'[{self.label}] 읽기 오류: {e}')
                self._force_close()
                buf = b''

    def stop(self):
        self._stop.set()
        self._force_close()

    @property
    def connected(self) -> bool:
        return self._connected


# ──────────────────────────────────────────────────────────────
# BoardState
# ──────────────────────────────────────────────────────────────
class BoardState:
    def __init__(self):
        self.lock = threading.Lock()

        self.target_vel_L = 0.0
        self.target_vel_R = 0.0

        self.actual_vel_L = 0.0
        self.actual_vel_R = 0.0

        self.pid_L = WheelPID()
        self.pid_R = WheelPID()
        self.last_pid_t = time.monotonic()

        self.steer_L    = 0.0
        self.steer_R    = 0.0
        self.steer_auto = False
        self.active     = False
        self.pending_cmds = []


# ──────────────────────────────────────────────────────────────
# STM32BridgeNode
# ──────────────────────────────────────────────────────────────
HEARTBEAT_PERIOD = 0.100
STAGGER_OFFSET   = HEARTBEAT_PERIOD * 0.4


class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('port_front', '/dev/ttyFRONT')
        self.declare_parameter('port_rear',  '/dev/ttyREAR')
        self.declare_parameter('baud',       115200)

        port_front = self.get_parameter('port_front').value
        port_rear  = self.get_parameter('port_rear').value
        baud       = self.get_parameter('baud').value

        self.get_logger().info(
            f'전륜: {port_front} | 후륜: {port_rear} | baud: {baud}')

        self.front       = BoardLink(port_front, baud, 'FRONT', self.get_logger())
        self.rear        = BoardLink(port_rear,  baud, 'REAR',  self.get_logger())
        self.front_state = BoardState()
        self.rear_state  = BoardState()

        # 퍼블리셔
        self._pub_steer_fb_front = self.create_publisher(
            Float32MultiArray, '/steer_fb/front', 10)
        self._pub_steer_fb_rear  = self.create_publisher(
            Float32MultiArray, '/steer_fb/rear',  10)
        self._pub_enc_front = self.create_publisher(
            Int32MultiArray, '/encoder/front', 10)
        self._pub_enc_rear  = self.create_publisher(
            Int32MultiArray, '/encoder/rear',  10)
        self._pub_vel_front = self.create_publisher(
            Float32MultiArray, '/wheel_vel/front', 10)
        self._pub_vel_rear  = self.create_publisher(
            Float32MultiArray, '/wheel_vel/rear',  10)

        self.front._fb_callback = lambda t: self._parse_fb('front', t)
        self.rear._fb_callback  = lambda t: self._parse_fb('rear',  t)

        self._fg_prev = {
            'front': {'L': 0, 'R': 0, 't': time.monotonic(), 'initialized': False},
            'rear':  {'L': 0, 'R': 0, 't': time.monotonic(), 'initialized': False},
        }
        self._vel_buf = {
            'front': {'L': [0.0]*VEL_BUF_SIZE, 'R': [0.0]*VEL_BUF_SIZE},
            'rear':  {'L': [0.0]*VEL_BUF_SIZE, 'R': [0.0]*VEL_BUF_SIZE},
        }

        threading.Thread(
            target=self.front.run_reader, daemon=True, name='rx_FRONT').start()
        threading.Thread(
            target=self.rear.run_reader,  daemon=True, name='rx_REAR').start()

        self._hb_stop = threading.Event()
        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='heartbeat').start()

        qos = 10

        self.create_subscription(Float32, '/motor_1/inwheel',
            lambda msg: self._update_target_vel(self.front_state, 'L', msg, 'front'), qos)
        self.create_subscription(Float32, '/motor_2/inwheel',
            lambda msg: self._update_target_vel(self.front_state, 'R', msg, 'front'), qos)
        self.create_subscription(Float32, '/motor_3/inwheel',
            lambda msg: self._update_target_vel(self.rear_state,  'L', msg, 'rear'), qos)
        self.create_subscription(Float32, '/motor_4/inwheel',
            lambda msg: self._update_target_vel(self.rear_state,  'R', msg, 'rear'), qos)

        self.create_subscription(Float32, '/motor_1/steer',
            lambda msg: self._update_steer(self.front_state, 'L', msg), qos)
        self.create_subscription(Float32, '/motor_2/steer',
            lambda msg: self._update_steer(self.front_state, 'R', msg), qos)
        self.create_subscription(Float32, '/motor_3/steer',
            lambda msg: self._update_steer(self.rear_state,  'L', msg), qos)
        self.create_subscription(Float32, '/motor_4/steer',
            lambda msg: self._update_steer(self.rear_state,  'R', msg), qos)

        self.create_subscription(String, '/motor/cmd', self._on_cmd, qos)

        self.get_logger().info(
            f'stm32_bridge 시작 (피드포워드+PID | '
            f'KP={KP} KI={KI} KD={KD} | '
            f'INT_LIM={INTEGRAL_LIMIT} DERIV_α={DERIV_FILTER_ALPHA} | '
            f'BLANKING={DIR_CHANGE_BLANKING_SEC}s | '
            f'MAX_VEL={MAX_WHEEL_VEL_MS}m/s)')

    # ── 이동평균 버퍼 클리어 ──────────────────────────────────
    def _clear_vel_buf(self, board: str, side: str = None):
        buf = self._vel_buf[board]
        if side is None or side == 'L':
            buf['L'] = [0.0] * VEL_BUF_SIZE
        if side is None or side == 'R':
            buf['R'] = [0.0] * VEL_BUF_SIZE

    # ── 목표 속도 갱신 ────────────────────────────────────────
    def _update_target_vel(self, state: BoardState, side: str, msg: Float32, board: str):
        v = float(msg.data)
        with state.lock:
            pid = state.pid_L if side == 'L' else state.pid_R
            pid.set_target(v)

            if pid.need_buf_clear:
                self._clear_vel_buf(board, side)
                if side == 'L':
                    state.actual_vel_L = 0.0
                else:
                    state.actual_vel_R = 0.0
                pid.need_buf_clear = False

            if side == 'L':
                state.target_vel_L = v
            else:
                state.target_vel_R = v
            state.active = True

    # ── 실제 속도 갱신 ────────────────────────────────────────
    def _update_actual_vel(self, state: BoardState, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        with state.lock:
            state.actual_vel_L = msg.data[0]
            state.actual_vel_R = msg.data[1]

    # ── 조향 갱신 ─────────────────────────────────────────────
    def _update_steer(self, state: BoardState, side: str, msg: Float32):
        v = max(-180.0, min(180.0, float(msg.data)))
        with state.lock:
            if side == 'L':
                state.steer_L = v
            else:
                state.steer_R = v
            state.steer_auto = True
            state.active     = True

    # ── 직접 명령 ─────────────────────────────────────────────
    def _on_cmd(self, msg: String):
        cmd   = msg.data.strip()
        upper = cmd.upper()

        if upper == 'STOP':
            for board_name, st in [('front', self.front_state),
                                   ('rear',  self.rear_state)]:
                with st.lock:
                    st.target_vel_L = 0.0
                    st.target_vel_R = 0.0
                    st.pid_L.reset()
                    st.pid_R.reset()
                    st.actual_vel_L = 0.0
                    st.actual_vel_R = 0.0
                    st.steer_L    = 0.0
                    st.steer_R    = 0.0
                    st.steer_auto = False
                    st.active     = False
                    st.pending_cmds.append(cmd)
                self._clear_vel_buf(board_name)
        elif upper == 'AUTO':
            for st in (self.front_state, self.rear_state):
                with st.lock:
                    st.steer_auto = True
                    st.pending_cmds.append(cmd)
        elif upper == 'MAN':
            for st in (self.front_state, self.rear_state):
                with st.lock:
                    st.steer_auto = False
                    st.pending_cmds.append(cmd)
        else:
            for st in (self.front_state, self.rear_state):
                with st.lock:
                    st.pending_cmds.append(cmd)

        self.get_logger().info(f'[CMD] {cmd}')

    # ── FB 파싱 ───────────────────────────────────────────────
    def _parse_fb(self, board: str, text: str):
        try:
            parts = text.split()
            if len(parts) < 9:
                return
            degL = int(parts[1]) / 100.0
            degR = int(parts[2]) / 100.0
            encL = int(parts[3])
            encR = int(parts[4])
            fgL  = int(parts[5])
            fgR  = int(parts[6])
            dirL = int(parts[7])
            dirR = int(parts[8])

            steer_msg = Float32MultiArray(data=[degL, degR])
            enc_msg   = Int32MultiArray(data=[encL, encR])

            now  = time.monotonic()
            prev = self._fg_prev[board]

            vel_msg = None
            if not prev['initialized']:
                prev['L']           = fgL
                prev['R']           = fgR
                prev['t']           = now
                prev['initialized'] = True
            else:
                dt = now - prev['t']
                if dt > 0.01:
                    dL = fgL - prev['L']
                    dR = fgR - prev['R']
                    if dL < 0: dL += 0x100000000
                    if dR < 0: dR += 0x100000000

                    rps_L = (dL / FG_PULSES_PER_REV) / dt
                    rps_R = (dR / FG_PULSES_PER_REV) / dt
                    vel_L = rps_L * WHEEL_CIRCUMF_M * (1 if dirL >= 0 else -1)
                    vel_R = rps_R * WHEEL_CIRCUMF_M * (1 if dirR >= 0 else -1)

                    buf = self._vel_buf[board]
                    buf['L'].pop(0); buf['L'].append(vel_L)
                    buf['R'].pop(0); buf['R'].append(vel_R)
                    vel_L = sum(buf['L']) / len(buf['L'])
                    vel_R = sum(buf['R']) / len(buf['R'])

                    prev['L'] = fgL
                    prev['R'] = fgR
                    prev['t'] = now

                    vel_msg = Float32MultiArray(data=[vel_L, vel_R])

            if board == 'front':
                self._pub_steer_fb_front.publish(steer_msg)
                self._pub_enc_front.publish(enc_msg)
                if vel_msg:
                    self._pub_vel_front.publish(vel_msg)
            else:
                self._pub_steer_fb_rear.publish(steer_msg)
                self._pub_enc_rear.publish(enc_msg)
                if vel_msg:
                    self._pub_vel_rear.publish(vel_msg)

            if vel_msg:
                st = self.front_state if board == 'front' else self.rear_state
                with st.lock:
                    st.actual_vel_L = vel_msg.data[0]
                    st.actual_vel_R = vel_msg.data[1]

        except (ValueError, IndexError):
            pass

    # ── Heartbeat + PID 루프 (100ms) ─────────────────────────
    def _heartbeat_loop(self):
        while not self._hb_stop.is_set():
            t0 = time.monotonic()

            self._send_board(self.front, self.front_state, True)

            elapsed = time.monotonic() - t0
            wait    = STAGGER_OFFSET - elapsed
            if wait > 0:
                time.sleep(wait)

            self._send_board(self.rear, self.rear_state, True)

            elapsed = time.monotonic() - t0
            wait    = HEARTBEAT_PERIOD - elapsed
            if wait > 0:
                time.sleep(wait)

    def _send_board(self, board: BoardLink, state: BoardState, send_fb: bool):
        if not board.connected:
            return

        with state.lock:
            cmds   = state.pending_cmds[:]
            state.pending_cmds.clear()
            active = state.active

            actual_L = state.actual_vel_L
            actual_R = state.actual_vel_R

            now = time.monotonic()
            dt  = now - state.last_pid_t
            state.last_pid_t = now
            if dt > 0.5:
                dt = HEARTBEAT_PERIOD

            tL   = state.steer_L
            tR   = state.steer_R
            auto = state.steer_auto

            if active:
                duty_L = state.pid_L.update(actual_L, dt)
                duty_R = state.pid_R.update(actual_R, dt)
            else:
                duty_L = 0.0
                duty_R = 0.0

        for cmd in cmds:
            board.write_line(cmd)

        if not active:
            if send_fb:
                board.write_line('FB')
            board.flush()
            return

        board.write_line(f'SPDL {duty_L:.3f}')
        time.sleep(0.003)
        board.write_line(f'SPDR {duty_R:.3f}')
        time.sleep(0.003)

        if auto:
            board.write_line(f'STERL {tL:.2f}')
            time.sleep(0.003)
            board.write_line(f'STERR {tR:.2f}')
            time.sleep(0.003)

        if send_fb:
            board.write_line('FB')
        board.flush()

    # ── 종료 ──────────────────────────────────────────────────
    def destroy_node(self):
        self._hb_stop.set()
        self.front.stop()
        self.rear.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
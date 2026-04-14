#!/usr/bin/env python3
"""
stm32_bridge.py
─────────────────────────────────────────────────────────────────
ROS2 토픽 → STM32 시리얼 명령 변환 브리지

피드백 토픽:
  /steer_fb/front   Float32MultiArray  [degL, degR]  조향 각도 (도)
  /steer_fb/rear    Float32MultiArray  [degL, degR]
  /encoder/front    Int32MultiArray    [encL, encR]  조향 엔코더 raw
  /encoder/rear     Int32MultiArray    [encL, encR]
  /wheel_vel/front  Float32MultiArray  [vL, vR]     인휠 선속도 (m/s)
  /wheel_vel/rear   Float32MultiArray  [vL, vR]

명령 토픽:
  /motor_N/inwheel  Float32  (-1.0 ~ 1.0)
  /motor_N/steer    Float32  (-180.0 ~ 180.0 deg)
  /motor/cmd        String   직접 명령 (STOP / ZERO / AUTO / MAN ...)
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
FG_PULSES_PER_REV = 60.0              # FG 펄스/회전 (실측 후 보정)
WHEEL_DIAMETER_M  = 0.1397           # 5.5인치
WHEEL_CIRCUMF_M   = 3.14159265 * WHEEL_DIAMETER_M


# ──────────────────────────────────────────────────────────────
# XOR 체크섬 생성
# ──────────────────────────────────────────────────────────────
def append_checksum(cmd: str) -> str:
    xor = 0
    for b in cmd.encode():
        xor ^= b
    return f'{cmd}*{xor:02X}'


# ──────────────────────────────────────────────────────────────
# BoardLink
# ──────────────────────────────────────────────────────────────
class BoardLink:
    def __init__(self, port: str, baud: int, label: str, logger):
        self.port   = port
        self.baud   = baud
        self.label  = label
        self.logger = logger

        self._ser   = None
        self._stop  = threading.Event()
        self._connected = False

        self.RECONNECT_INTERVAL = 2.0
        self._init_cmd = None
        self._fb_callback = None

    def _try_open(self) -> bool:
        if not os.path.exists(self.port):
            return False
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._ser = ser
            self._connected = True
            self.logger.info(f'[{self.label}] 연결됨 ({self.port})')
            if self._init_cmd:
                time.sleep(0.5)
                ser.write((self._init_cmd + '\n').encode())
                self.logger.info(f'[{self.label}] init: {self._init_cmd}')
            return True
        except serial.SerialException as e:
            self.logger.warn(f'[{self.label}] 열기 실패: {e}')
            return False

    def _force_close(self):
        ser = self._ser
        self._ser = None
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
                data = ser.read(waiting if waiting > 0 else 1)
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
        self.speed_L = 0.0
        self.speed_R = 0.0
        self.steer_L = 0.0
        self.steer_R = 0.0
        self.steer_auto = False
        self.active = False
        self.pending_cmds = []


# ──────────────────────────────────────────────────────────────
# STM32BridgeNode
# ──────────────────────────────────────────────────────────────
HEARTBEAT_PERIOD = 0.100  # 100ms
STAGGER_OFFSET   = HEARTBEAT_PERIOD * 0.4  # 40ms


class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # ── 파라미터 ──────────────────────────────────────────
        self.declare_parameter('port_front', '/dev/ttyACM0')
        self.declare_parameter('port_rear',  '/dev/ttyACM1')
        self.declare_parameter('baud',       115200)

        port_front = self.get_parameter('port_front').value
        port_rear  = self.get_parameter('port_rear').value
        baud       = self.get_parameter('baud').value

        self.get_logger().info(
            f'전륜: {port_front} | 후륜: {port_rear} | baud: {baud}')

        # ── BoardLink + State ─────────────────────────────────
        self.front = BoardLink(port_front, baud, 'FRONT', self.get_logger())
        self.rear  = BoardLink(port_rear,  baud, 'REAR',  self.get_logger())
        self.front_state = BoardState()
        self.rear_state  = BoardState()

        # ── 피드백 퍼블리셔 ───────────────────────────────────
        self._pub_steer_fb_front = self.create_publisher(
            Float32MultiArray, '/steer_fb/front', 10)
        self._pub_steer_fb_rear = self.create_publisher(
            Float32MultiArray, '/steer_fb/rear', 10)
        self._pub_enc_front = self.create_publisher(
            Int32MultiArray, '/encoder/front', 10)
        self._pub_enc_rear = self.create_publisher(
            Int32MultiArray, '/encoder/rear', 10)
        self._pub_vel_front = self.create_publisher(
            Float32MultiArray, '/wheel_vel/front', 10)
        self._pub_vel_rear = self.create_publisher(
            Float32MultiArray, '/wheel_vel/rear', 10)

        # ── 피드백 콜백 연결 ──────────────────────────────────
        self.front._fb_callback = lambda t: self._parse_fb('front', t)
        self.rear._fb_callback  = lambda t: self._parse_fb('rear', t)

        # ── FG 펄스 기반 속도 계산용 ─────────────────────────
        self._fg_prev = {
            'front': {'L': 0, 'R': 0, 't': time.monotonic(), 'initialized': False},
            'rear':  {'L': 0, 'R': 0, 't': time.monotonic(), 'initialized': False},
        }
        self._vel_buf = {
            'front': {'L': [0.0]*4, 'R': [0.0]*4},
            'rear':  {'L': [0.0]*4, 'R': [0.0]*4},
        }

        # ── RX 스레드 ─────────────────────────────────────────
        threading.Thread(
            target=self.front.run_reader, daemon=True, name='rx_FRONT').start()
        threading.Thread(
            target=self.rear.run_reader, daemon=True, name='rx_REAR').start()

        # ── Heartbeat 스레드 ──────────────────────────────────
        self._hb_stop = threading.Event()
        self._hb_count = 0
        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='heartbeat').start()

        # ── 구독자 등록 ───────────────────────────────────────
        qos = 10

        # motor_1=전좌(FL), motor_2=전우(FR), motor_3=후좌(RL), motor_4=후우(RR)
        self.create_subscription(Float32, '/motor_1/inwheel',
            lambda msg: self._update_speed(self.front_state, 'L', msg), qos)
        self.create_subscription(Float32, '/motor_2/inwheel',
            lambda msg: self._update_speed(self.front_state, 'R', msg), qos)
        self.create_subscription(Float32, '/motor_3/inwheel',
            lambda msg: self._update_speed(self.rear_state,  'L', msg), qos)
        self.create_subscription(Float32, '/motor_4/inwheel',
            lambda msg: self._update_speed(self.rear_state,  'R', msg), qos)

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
            f'stm32_bridge 시작 (heartbeat {HEARTBEAT_PERIOD*1000:.0f}ms, '
            f'stagger {STAGGER_OFFSET*1000:.0f}ms, checksum ON, FB ON)')

    # ── 상태 갱신 콜백 ────────────────────────────────────────
    def _update_speed(self, state: BoardState, side: str, msg: Float32):
        v = max(-1.0, min(1.0, float(msg.data)))
        with state.lock:
            if side == 'L':
                state.speed_L = v
            else:
                state.speed_R = v
            state.active = True

    def _update_steer(self, state: BoardState, side: str, msg: Float32):
        v = max(-180.0, min(180.0, float(msg.data)))
        with state.lock:
            if side == 'L':
                state.steer_L = v
            else:
                state.steer_R = v
            state.steer_auto = True
            state.active = True

    # ── 직접 명령 콜백 ────────────────────────────────────────
    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        upper = cmd.upper()

        if upper == 'STOP':
            for st in (self.front_state, self.rear_state):
                with st.lock:
                    st.speed_L = 0.0
                    st.speed_R = 0.0
                    st.steer_L = 0.0
                    st.steer_R = 0.0
                    st.steer_auto = False
                    st.active = False
                    st.pending_cmds.append(cmd)
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

        self.get_logger().info(f'[CMD] {cmd} → queued')

    # ── 피드백 파싱 ───────────────────────────────────────────
    def _parse_fb(self, board: str, text: str):
        """'FB <degL_x100> <degR_x100> <encL> <encR> <fgL> <fgR>' 파싱"""
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
            dirL = int(parts[7])   # 1 or -1
            dirR = int(parts[8])

            steer_msg = Float32MultiArray(data=[degL, degR])
            enc_msg   = Int32MultiArray(data=[encL, encR])

            # 선속도 계산
            now = time.monotonic()
            prev = self._fg_prev[board]
            # dt = now - prev['t']

            vel_msg = None
            if not prev['initialized']:
                # 첫 FB: 기준점만 설정, 속도 publish 안함
                prev['L'] = fgL
                prev['R'] = fgR
                prev['t'] = now
                prev['initialized'] = True
            else:
                dt = now - prev['t']
                if dt > 0.01:
                    dL = fgL - prev['L']
                    dR = fgR - prev['R']
                    if dL < 0:
                        dL += 0x100000000
                    if dR < 0:
                        dR += 0x100000000

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
        except (ValueError, IndexError):
            pass

    # ── Heartbeat 전송 루프 ───────────────────────────────────
    def _heartbeat_loop(self):
        while not self._hb_stop.is_set():
            t0 = time.monotonic()
            self._hb_count += 1

            self._send_board(self.front, self.front_state,
                             True)

            elapsed = time.monotonic() - t0
            wait = STAGGER_OFFSET - elapsed
            if wait > 0:
                time.sleep(wait)

            self._send_board(self.rear, self.rear_state,
                             True)

            elapsed = time.monotonic() - t0
            wait = HEARTBEAT_PERIOD - elapsed
            if wait > 0:
                time.sleep(wait)

    def _send_board(self, board: BoardLink, state: BoardState, send_fb: bool):
        if not board.connected:
            return

        with state.lock:
            cmds = state.pending_cmds[:]
            state.pending_cmds.clear()
            active = state.active
            sL = state.speed_L
            sR = state.speed_R
            tL = state.steer_L
            tR = state.steer_R
            auto = state.steer_auto

        for cmd in cmds:
            board.write_line(cmd)

        if not active:
            if send_fb:
                board.write_line('FB')
            board.flush()
            return

        board.write_line(f'SPDL {sL:.3f}')
        time.sleep(0.003)
        board.write_line(f'SPDR {sR:.3f}')
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
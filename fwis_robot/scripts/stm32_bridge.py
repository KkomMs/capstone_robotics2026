#!/usr/bin/env python3
"""
stm32_bridge.py
─────────────────────────────────────────────────────────────────
ROS2 토픽 → STM32 시리얼 명령 변환 브리지

설계 원칙:
  1. 토픽 콜백은 내부 상태만 갱신 (시리얼 전송 없음)
  2. 단일 heartbeat 스레드가 100ms 주기로 상태 전송
  3. front/rear 전송을 50ms 오프셋으로 시간 분리
     → USB 호스트 컨트롤러의 bulk OUT 경합 완전 방지
  4. 모든 명령에 XOR 체크섬 부착 (SPDL 0.300*A3)
     → ORE로 바이트 손상 시 STM32가 해당 줄 폐기
     → 100ms 뒤 다음 heartbeat에서 복구

토픽:
  /motor_N/inwheel  Float32  (-1.0 ~ 1.0)
  /motor_N/steer    Float32  (-35.0 ~ 35.0 deg)
  /motor/cmd        String   직접 명령 (STOP / ZERO / AUTO / MAN ...)
─────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import serial
import threading
import time
import os


# ──────────────────────────────────────────────────────────────
# XOR 체크섬 생성
# ──────────────────────────────────────────────────────────────
def append_checksum(cmd: str) -> str:
    """'SPDL 0.300' → 'SPDL 0.300*A3'"""
    xor = 0
    for b in cmd.encode():
        xor ^= b
    return f'{cmd}*{xor:02X}'


# ──────────────────────────────────────────────────────────────
# BoardLink : 시리얼 포트 관리 + 자동 재연결
# ──────────────────────────────────────────────────────────────
class BoardLink:
    """
    TX queue 없음. 전송은 heartbeat 스레드에서만 수행.
    → 단일 스레드 write 보장, lock 불필요.
    """
    def __init__(self, port: str, baud: int, label: str, logger):
        self.port   = port
        self.baud   = baud
        self.label  = label
        self.logger = logger

        self._ser   = None
        self._stop  = threading.Event()
        self._connected = False

        self.RECONNECT_INTERVAL = 2.0

    def _try_open(self) -> bool:
        if not os.path.exists(self.port):
            return False
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._ser = ser
            self._connected = True
            self.logger.info(f'[{self.label}] 연결됨 ({self.port})')
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
        """체크섬 부착 후 시리얼 1줄 전송"""
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
        """RX 전용 스레드"""
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
# BoardState : 보드 하나의 목표 상태
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
        self.pending_cmds = []  # STOP, ZERO 등 직접 명령


# ──────────────────────────────────────────────────────────────
# STM32BridgeNode
# ──────────────────────────────────────────────────────────────
HEARTBEAT_PERIOD = 0.050  # 100ms 주기
STAGGER_OFFSET   = HEARTBEAT_PERIOD * 0.3 


class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # ── 파라미터 ──────────────────────────────────────────
        self.declare_parameter('port_front', '/dev/ttyACM1')
        self.declare_parameter('port_rear',  '/dev/ttyACM0')
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

        # ── RX 스레드 ─────────────────────────────────────────
        threading.Thread(
            target=self.front.run_reader, daemon=True, name='rx_FRONT').start()
        threading.Thread(
            target=self.rear.run_reader, daemon=True, name='rx_REAR').start()

        # ── Heartbeat 스레드 (단일) ───────────────────────────
        self._hb_stop = threading.Event()
        threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='heartbeat').start()

        # ── 구독자 등록 ───────────────────────────────────────
        qos = 10

        # 인휠 (좌우 물리 배선 반전 보정)
        self.create_subscription(Float32, '/motor_1/inwheel',
            lambda msg: self._update_speed(self.front_state, 'R', msg), qos)
        self.create_subscription(Float32, '/motor_2/inwheel',
            lambda msg: self._update_speed(self.front_state, 'L', msg), qos)
        self.create_subscription(Float32, '/motor_3/inwheel',
            lambda msg: self._update_speed(self.rear_state,  'R', msg), qos)
        self.create_subscription(Float32, '/motor_4/inwheel',
            lambda msg: self._update_speed(self.rear_state,  'L', msg), qos)

        # 조향 (좌우 물리 배선 반전 보정)
        self.create_subscription(Float32, '/motor_1/steer',
            lambda msg: self._update_steer(self.front_state, 'R', msg), qos)
        self.create_subscription(Float32, '/motor_2/steer',
            lambda msg: self._update_steer(self.front_state, 'L', msg), qos)
        self.create_subscription(Float32, '/motor_3/steer',
            lambda msg: self._update_steer(self.rear_state,  'R', msg), qos)
        self.create_subscription(Float32, '/motor_4/steer',
            lambda msg: self._update_steer(self.rear_state,  'L', msg), qos)

        # 직접 명령
        self.create_subscription(String, '/motor/cmd', self._on_cmd, qos)

        self.get_logger().info(
            f'stm32_bridge 시작 (heartbeat {HEARTBEAT_PERIOD*1000:.0f}ms, '
            f'stagger {STAGGER_OFFSET*1000:.0f}ms, checksum ON)')

    # ── 상태 갱신 콜백 (전송 없음) ────────────────────────────
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

    # ── Heartbeat 전송 루프 ───────────────────────────────────
    def _heartbeat_loop(self):
        """
        100ms 주기:
          t=0ms   : front 전송
          t=50ms  : rear 전송
          t=100ms : 다음 사이클
        """
        while not self._hb_stop.is_set():
            t0 = time.monotonic()

            self._send_board(self.front, self.front_state)

            elapsed = time.monotonic() - t0
            wait = STAGGER_OFFSET - elapsed
            if wait > 0:
                time.sleep(wait)

            self._send_board(self.rear, self.rear_state)

            elapsed = time.monotonic() - t0
            wait = HEARTBEAT_PERIOD - elapsed
            if wait > 0:
                time.sleep(wait)

    def _send_board(self, board: BoardLink, state: BoardState):
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

        # 직접 명령 먼저
        for cmd in cmds:
            board.write_line(cmd)

        if not active:
            board.flush()
            return

        # 인휠 속도
        board.write_line(f'SPDL {sL:.3f}')
        board.write_line(f'SPDR {sR:.3f}')

        # 조향
        if auto:
            board.write_line(f'STERL {tL:.2f}')
            board.write_line(f'STERR {tR:.2f}')

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
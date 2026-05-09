#!/usr/bin/env python3
"""
asset_reporter.py

BarcodeEvent 메시지 구조:
  msg.serial     = "AAAP"  (시리얼, 알파벳 4자리)
  msg.unit       = 16      (유닛 번호, 정수)
  msg.scanner_id = 1 또는 2

마커 ID = 랙 번호 (1~10)
좌스캐너(scanner_id=1) → 왼쪽 카메라 마커의 랙
우스캐너(scanner_id=2) → 오른쪽 카메라 마커의 랙
"""

import json
import threading
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from scanner_interfaces.msg import BarcodeEvent
from ros2_aruco_interfaces.msg import ArucoMarkers


class AssetReporterNode(Node):

    def __init__(self):
        super().__init__('asset_reporter')

        self.declare_parameter('agent_host', 'http://localhost:5050')
        # self.declare_parameter('agent_host', 'http://10.168.237.214:5050')  # 다른 PC
        self.declare_parameter('robot_id',          'ROBOT-A01')
        self.declare_parameter('left_scanner_id',   1)
        self.declare_parameter('right_scanner_id',  2)

        self.agent_host       = self.get_parameter('agent_host').value
        self.robot_id         = self.get_parameter('robot_id').value
        self.left_scanner_id  = self.get_parameter('left_scanner_id').value
        self.right_scanner_id = self.get_parameter('right_scanner_id').value

        self._lock = threading.Lock()

        # 마지막으로 인식한 랙 번호 (마커 ID = 랙 번호)
        self._left_rack_no  = None
        self._right_rack_no = None

        # 스캔 버퍼: { scanner_id: { unit: serial } }
        self._scan_buffer = {}

        qos = 10
        self.create_subscription(BarcodeEvent, '/barcode/unit_event', self._on_barcode, qos)
        self.create_subscription(ArucoMarkers, '/aruco_markers_left',
            lambda msg: self._on_aruco(msg, 'left'), qos)
        self.create_subscription(ArucoMarkers, '/aruco_markers_right',
            lambda msg: self._on_aruco(msg, 'right'), qos)
        self.create_subscription(Bool, '/scan_done', self._on_scan_done, qos)

        self.result_pub = self.create_publisher(String, '/asset_report/result', qos)

        self.get_logger().info(
            f'[AssetReporter] 시작 | agent={self.agent_host} | robot={self.robot_id}'
        )

    def _on_aruco(self, msg: ArucoMarkers, side: str):
        if not msg.marker_ids:
            return
        rack_no = int(msg.marker_ids[0])
        with self._lock:
            if side == 'left':
                self._left_rack_no = rack_no
            else:
                self._right_rack_no = rack_no

    def _on_barcode(self, msg: BarcodeEvent):
        # BarcodeEvent에 serial, unit, scanner_id 이미 분리되어 들어옴
        serial     = msg.serial.strip().upper()
        unit       = int(msg.unit)
        scanner_id = int(msg.scanner_id)

        if not serial or unit <= 0:
            self.get_logger().warn(
                f'[AssetReporter] 유효하지 않은 바코드: serial="{serial}" unit={unit}')
            return

        with self._lock:
            if scanner_id not in self._scan_buffer:
                self._scan_buffer[scanner_id] = {}
            self._scan_buffer[scanner_id][unit] = serial

        self.get_logger().info(
            f'[AssetReporter] 바코드 수신 | scanner={scanner_id} unit={unit:02d} serial={serial}'
        )

    def _on_scan_done(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info('[AssetReporter] scan_done 수신 → 서버 전송 시작')
        threading.Thread(target=self._send_to_server, daemon=True).start()

    def _send_to_server(self):
        with self._lock:
            buffer_copy   = {k: dict(v) for k, v in self._scan_buffer.items()}
            left_rack_no  = self._left_rack_no
            right_rack_no = self._right_rack_no

        if left_rack_no is None:
            self.get_logger().warn('[AssetReporter] 왼쪽 마커 미인식')
        if right_rack_no is None:
            self.get_logger().warn('[AssetReporter] 오른쪽 마커 미인식')

        scanner_to_rack = {}
        if left_rack_no  is not None: scanner_to_rack[self.left_scanner_id]  = left_rack_no
        if right_rack_no is not None: scanner_to_rack[self.right_scanner_id] = right_rack_no

        if not scanner_to_rack:
            self.get_logger().error('[AssetReporter] 랙 번호 불명 → 전송 취소')
            self._clear_buffer()
            return

        items = []
        for scanner_id, slots in buffer_copy.items():
            rack_no = scanner_to_rack.get(scanner_id)
            if rack_no is None:
                self.get_logger().warn(f'[AssetReporter] scanner_id={scanner_id} 랙 없음 → 스킵')
                continue
            rack_qr = f'RACK-{rack_no}'
            for unit, serial in slots.items():
                items.append({
                    'rack_qr':   rack_qr,
                    'server_qr': f'SERVER-{serial}',
                    'rack_no':   rack_no,
                    'unit':      unit,
                    'serial':    serial,
                })

        if not items:
            self.get_logger().warn('[AssetReporter] 전송할 데이터 없음')
            self._clear_buffer()
            return

        self.get_logger().info(
            f'[AssetReporter] 배치 전송 | 총 {len(items)}건 | 랙: {list(scanner_to_rack.values())}'
        )

        url = f'{self.agent_host}/api/robot/verify/batch'
        try:
            resp = requests.post(
                url, json={'items': items, 'robot_id': self.robot_id}, timeout=10.0)
            resp.raise_for_status()
            data    = resp.json()
            summary = data.get('summary', {})
            self.get_logger().info(
                f'[AssetReporter] 전송 완료 ✓ '
                f'| 일치={summary.get("match",0)} '
                f'| 불일치={summary.get("mismatch",0)} '
                f'| 미확인={summary.get("not_found",0)} '
                f'| 빈슬롯={summary.get("empty",0)}'
            )
            result_msg = String()
            result_msg.data = json.dumps(data, ensure_ascii=False)
            self.result_pub.publish(result_msg)

            for r in data.get('results', []):
                if r.get('result') == 'MISMATCH':
                    self.get_logger().warn(
                        f'[AssetReporter] 불일치: {r.get("server_id")} | {r.get("detail")}')
                elif r.get('result') == 'NOT_FOUND':
                    self.get_logger().warn(
                        f'[AssetReporter] 미확인: server_qr={r.get("server_qr")}')

        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'[AssetReporter] 서버 연결 실패: {url}')
            self._report_error('CONN_TIMEOUT', f'서버 연결 실패: {url}')
        except requests.exceptions.Timeout:
            self.get_logger().error('[AssetReporter] 서버 응답 타임아웃')
            self._report_error('CONN_TIMEOUT', '서버 응답 타임아웃')
        except Exception as e:
            self.get_logger().error(f'[AssetReporter] 전송 오류: {e}')
            self._report_error('NETWORK_ERR', str(e))
        finally:
            self._clear_buffer()

    def _clear_buffer(self):
        with self._lock:
            self._scan_buffer.clear()
        self.get_logger().info('[AssetReporter] 버퍼 초기화 → 다음 랙 대기')

    def _report_error(self, code: str, message: str):
        try:
            requests.post(
                f'{self.agent_host}/api/robot/errors',
                json={
                    'source': 'ROBOT', 'level': 'ERROR',
                    'robot_id': self.robot_id,
                    'api_path': '/api/robot/verify/batch',
                    'error_code': code, 'error_message': message,
                }, timeout=3.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AssetReporterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
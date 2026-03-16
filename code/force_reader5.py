#!/usr/bin/env python3
"""
FORCE READER — 5-Direction Graphite Sponge Sensor via ESP32 Serial

Reads JSON from ESP32:
    {"r":[raw0,raw1,raw2,raw3,raw4],
     "p":[p0,p1,p2,p3,p4],
     "fx":1.23,"fy":-0.45,"fz":3.10,"fm":3.37}

Publishes:
    /slave/force     (geometry_msgs/WrenchStamped)
    /slave/force_raw (std_msgs/Float64)

Directions:
    0: Right
    1: Left
    2: Forward
    3: Back
    4: Bottom

USAGE:
    python3 force_reader5.py --port /dev/ttyUSB0
    python3 force_reader5.py --port /dev/ttyUSB0 --cal
"""

import argparse
import glob
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import serial


DIR_NAMES = ['Right', 'Left', 'Forward', 'Back', 'Bottom']


class ForceReader5Dir(Node):
    def __init__(self, port, baud=115200, calibration_mode=False, cal_window=80):
        super().__init__('force_reader_5dir')

        self.calibration_mode = calibration_mode
        self.cal_window = max(10, int(cal_window))

        self.wrench_pub = self.create_publisher(WrenchStamped, '/slave/force', 10)
        self.raw_pub = self.create_publisher(Float64, '/slave/force_raw', 10)

        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.fm = 0.0

        self.pressures = [0.0] * 5
        self.raw_adc = [0] * 5
        self.rx_count = 0
        self.running = True
        self.contact = False
        self.contact_threshold = 12.5

        self.raw_hist = [[] for _ in range(5)]
        self.raw_min_seen = [10**9] * 5
        self.raw_max_seen = [-10**9] * 5
        self.last_status_print = time.time()
        self.last_cal_print = time.time()

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(1.5)
            self.get_logger().info(f'Connected to {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                self.get_logger().error(f'Available ports: {ports}')
            sys.exit(1)

        threading.Thread(target=self._serial_loop, daemon=True).start()
        self.timer = self.create_timer(0.05, self._publish)

        self.get_logger().info('')
        self.get_logger().info('=' * 72)
        self.get_logger().info('5-DIRECTION FORCE READER')
        self.get_logger().info(f'Serial: {port} @ {baud}')
        self.get_logger().info('Directions: Right | Left | Forward | Back | Bottom')
        self.get_logger().info('Publishing: /slave/force , /slave/force_raw')
        self.get_logger().info(f'Contact threshold: {self.contact_threshold:.2f} N')
        self.get_logger().info(f'Calibration mode: {self.calibration_mode}')
        self.get_logger().info('=' * 72)
        self.get_logger().info('')

    def _push_raw_sample(self, values):
        for i, v in enumerate(values):
            self.raw_hist[i].append(int(v))
            if len(self.raw_hist[i]) > self.cal_window:
                self.raw_hist[i].pop(0)

            if v < self.raw_min_seen[i]:
                self.raw_min_seen[i] = int(v)
            if v > self.raw_max_seen[i]:
                self.raw_max_seen[i] = int(v)

    def _hist_stats(self, i):
        hist = self.raw_hist[i]
        if not hist:
            return 0, 0, 0.0
        mn = min(hist)
        mx = max(hist)
        avg = sum(hist) / len(hist)
        return mn, mx, avg

    def _print_calibration_summary(self):
        self.get_logger().info('--- CALIBRATION SNAPSHOT ---')
        for i, name in enumerate(DIR_NAMES):
            mn, mx, avg = self._hist_stats(i)
            self.get_logger().info(
                f'{name:<8} raw={self.raw_adc[i]:4d}   '
                f'win[min/avg/max]={mn:4d}/{avg:6.1f}/{mx:4d}   '
                f'seen[min/max]={self.raw_min_seen[i]:4d}/{self.raw_max_seen[i]:4d}'
            )

        baseline_guess = [int(round(self._hist_stats(i)[2])) for i in range(5)]
        pressed_guess = [self.raw_min_seen[i] if self.raw_min_seen[i] < 10**9 else 0 for i in range(5)]

        self.get_logger().info(f'BASELINE guess = {baseline_guess}')
        self.get_logger().info(f'PRESSED  guess = {pressed_guess}')

    def _print_normal_status(self):
        press_pct = [int(round(p * 100.0)) for p in self.pressures]
        bar = 'CONTACT' if self.contact else 'free'
        self.get_logger().info(
            f'Fx={self.fx:+6.2f}  Fy={self.fy:+6.2f}  Fz={self.fz:+6.2f}  |F|={self.fm:5.2f} N  [{bar}]'
        )
        self.get_logger().info(
            f'RAW  : R={self.raw_adc[0]:4d}  L={self.raw_adc[1]:4d}  '
            f'F={self.raw_adc[2]:4d}  B={self.raw_adc[3]:4d}  D={self.raw_adc[4]:4d}'
        )
        self.get_logger().info(
            f'PRES : R={press_pct[0]:3d}% L={press_pct[1]:3d}% '
            f'F={press_pct[2]:3d}% B={press_pct[3]:3d}% D={press_pct[4]:3d}%'
        )

    def _serial_loop(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('#') or line.startswith('==='):
                    continue

                if line.startswith('RAW:'):
                    try:
                        vals = [0] * 5
                        parts = line.replace('RAW:', '').strip().split()
                        for part in parts:
                            if '=' not in part:
                                continue
                            k, v = part.split('=')
                            k = k.strip().lower()
                            v = int(v.strip())

                            if k.startswith('right'):
                                vals[0] = v
                            elif k.startswith('left'):
                                vals[1] = v
                            elif k.startswith('forward'):
                                vals[2] = v
                            elif k.startswith('back'):
                                vals[3] = v
                            elif k.startswith('bottom') or k.startswith('down'):
                                vals[4] = v

                        self.raw_adc = vals
                        self._push_raw_sample(vals)
                        self.rx_count += 1
                    except Exception:
                        pass
                    continue

                try:
                    data = json.loads(line)

                    r = data.get('r', [0] * 5)
                    p = data.get('p', [0.0] * 5)

                    if len(r) == 5:
                        self.raw_adc = [int(x) for x in r]
                        self._push_raw_sample(self.raw_adc)

                    if len(p) == 5:
                        self.pressures = [float(x) for x in p]

                    self.fx = float(data.get('fx', 0.0))
                    self.fy = float(data.get('fy', 0.0))
                    self.fz = float(data.get('fz', 0.0))
                    self.fm = float(data.get('fm', 0.0))
                    self.contact = self.fm > self.contact_threshold
                    self.rx_count += 1

                except (json.JSONDecodeError, ValueError, TypeError):
                    pass

            except serial.SerialException:
                self.get_logger().warn('Serial disconnected')
                break
            except Exception:
                pass

    def _publish(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'R_link_eef'
        msg.wrench.force.x = self.fx
        msg.wrench.force.y = self.fy
        msg.wrench.force.z = self.fz
        self.wrench_pub.publish(msg)

        raw_msg = Float64()
        raw_msg.data = self.fm
        self.raw_pub.publish(raw_msg)

        now = time.time()
        if self.calibration_mode:
            if now - self.last_cal_print > 1.0:
                self.last_cal_print = now
                self._print_calibration_summary()
        else:
            if now - self.last_status_print > 2.0:
                self.last_status_print = now
                self._print_normal_status()

    def destroy_node(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--cal', action='store_true')
    parser.add_argument('--cal-window', type=int, default=80)
    args = parser.parse_args()

    rclpy.init()
    node = ForceReader5Dir(
        port=args.port,
        baud=args.baud,
        calibration_mode=args.cal,
        cal_window=args.cal_window,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
MASTER INPUT v2 — Control xArm Lite 6 with 6-direction sensors.
Keeps EF always pointing in the same orientation (no rotation).

Uses full 6x6 Jacobian: commands [vx, vy, vz, 0, 0, 0]
-> arm translates without rotating.

Now also listens to:
    /master_hold   (std_msgs/Bool)

When /master_hold == True:
    the master arm freezes and ignores sensor inputs until hold is released.
"""

import argparse
import glob
import json
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import serial

try:
    import tf2_ros
except ImportError:
    print("sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-py")
    sys.exit(1)


MASTER_JOINTS = [f'L_joint{i}' for i in range(1, 7)]
MASTER_TRAJ_TOPIC = '/L_lite6_traj_controller/joint_trajectory'

BASE_FRAME = 'L_link_base'
EE_FRAME = 'L_link_eef'

JOINT_FRAME_CANDIDATES = [
    [f'L_link{i}' for i in range(1, 7)],
    [f'L_link{i}' for i in range(0, 6)],
    ['L_link_base'] + [f'L_link{i}' for i in range(1, 6)],
]


def quat_to_rotmat(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]
    ])


class MasterInput(Node):
    def __init__(self, port, baud=115200, speed_scale=1.0, discover_mode=False):
        super().__init__('master_input')

        self.speed_scale = speed_scale
        self.discover_mode = discover_mode

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_joints = None
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        self.traj_pub = self.create_publisher(
            JointTrajectory, MASTER_TRAJ_TOPIC, 10)

        # NEW: hold signal from slave/mirror
        self.master_hold = False
        self.hold_sub = self.create_subscription(
            Bool, '/master_hold', self._hold_cb, 10
        )

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.pressures = [0.0] * 6
        self.rx_count = 0
        self.running = True

        self.joint_frames = None
        self.frames_ready = False

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(1.5)
            self.get_logger().info(f'ESP32 connected on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                self.get_logger().error(f'Available: {ports}')
            sys.exit(1)

        threading.Thread(target=self._serial_loop, daemon=True).start()
        self.create_timer(1.0, self._discover_frames_once)
        self.control_timer = self.create_timer(0.05, self._control_loop)
        self.last_print = time.time()

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('MASTER INPUT v2 - 6-Direction Sensor')
        self.get_logger().info('Waiting for TF tree...')
        self.get_logger().info('Listening for hold on /master_hold')
        self.get_logger().info('=' * 60)

    def _hold_cb(self, msg: Bool):
        prev = self.master_hold
        self.master_hold = bool(msg.data)

        if self.master_hold and not prev:
            self.get_logger().warn('MASTER HOLD ACTIVE -> slave detected contact, master frozen')
        elif (not self.master_hold) and prev:
            self.get_logger().info('MASTER HOLD RELEASED -> motion enabled again')

    def _lookup(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(
                source, target, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z])
            q = [t.transform.rotation.x, t.transform.rotation.y,
                 t.transform.rotation.z, t.transform.rotation.w]
            R = quat_to_rotmat(q)
            return pos, R, q
        except Exception:
            return None, None, None

    def _discover_frames_once(self):
        if self.frames_ready:
            return

        p_ee, R_ee, q_ee = self._lookup(EE_FRAME, BASE_FRAME)
        if p_ee is None:
            self.get_logger().warn(f'Waiting for TF: {EE_FRAME} -> {BASE_FRAME}')
            try:
                all_frames = self.tf_buffer.all_frames_as_string()
                if all_frames and not hasattr(self, '_frames_printed'):
                    self._frames_printed = True
                    self.get_logger().info(f'Available frames:\n{all_frames[:2000]}')
            except Exception:
                pass
            return

        for candidate in JOINT_FRAME_CANDIDATES:
            all_ok = True
            for frame in candidate:
                p, _, _ = self._lookup(frame, BASE_FRAME)
                if p is None:
                    all_ok = False
                    break
            if all_ok:
                self.joint_frames = candidate
                break

        if self.joint_frames is None:
            self.get_logger().warn('No matching joint frames found.')
            try:
                self.get_logger().info(self.tf_buffer.all_frames_as_string()[:2000])
            except Exception:
                pass
            return

        self.frames_ready = True

        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('TF FRAMES DISCOVERED:')
        self.get_logger().info(f'  Base:   {BASE_FRAME}')
        self.get_logger().info(f'  EE:     {EE_FRAME}')
        self.get_logger().info(f'  Joints: {self.joint_frames}')
        self.get_logger().info(f'  EE pos: [{p_ee[0]:.4f}, {p_ee[1]:.4f}, {p_ee[2]:.4f}]')
        self.get_logger().info('')
        self.get_logger().info('Orientation LOCKED (6x6 Jacobian, w=0)')
        self.get_logger().info('=' * 60)

        if self.discover_mode:
            self.get_logger().info('DISCOVER MODE - no movement. Restart without --discover.')

    def _serial_loop(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith('#') or line.startswith('='):
                    continue
                if line.startswith('RAW:'):
                    self.get_logger().info(f'[CAL] {line}')
                    continue
                data = json.loads(line)
                self.vx = float(data.get('vx', 0.0))
                self.vy = float(data.get('vy', 0.0))
                self.vz = float(data.get('vz', 0.0))
                self.pressures = data.get('p', [0.0] * 6)
                self.rx_count += 1
            except (json.JSONDecodeError, ValueError):
                pass
            except serial.SerialException:
                self.get_logger().warn('Serial disconnected')
                break

    def _joint_cb(self, msg: JointState):
        positions = []
        for jname in MASTER_JOINTS:
            if jname in msg.name:
                positions.append(msg.position[msg.name.index(jname)])
            else:
                return
        self.current_joints = np.array(positions)

    def _compute_jacobian_6x6(self):
        p_ee, _, _ = self._lookup(EE_FRAME, BASE_FRAME)
        if p_ee is None:
            return None

        J = np.zeros((6, 6))

        for i, frame in enumerate(self.joint_frames):
            p_i, R_i, _ = self._lookup(frame, BASE_FRAME)
            if p_i is None or R_i is None:
                return None

            z_i = R_i[:, 2]
            J[0:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:6, i] = z_i

        return J

    def _control_loop(self):
        if not self.frames_ready or self.current_joints is None:
            return
        if self.discover_mode:
            return

        # NEW: freeze master while slave is in contact hold
        if self.master_hold:
            return

        v_robot_x = self.vy * self.speed_scale
        v_robot_y = self.vx * self.speed_scale
        v_robot_z = self.vz * self.speed_scale

        v_cart = np.array([v_robot_x, v_robot_y, v_robot_z])

        if np.linalg.norm(v_cart) < 1e-6:
            return

        twist = np.array([v_cart[0], v_cart[1], v_cart[2], 0.0, 0.0, 0.0])

        J = self._compute_jacobian_6x6()
        if J is None:
            return

        damping = 0.01
        JJT = J @ J.T + damping**2 * np.eye(6)
        J_pinv = J.T @ np.linalg.inv(JJT)

        dq = J_pinv @ twist

        max_dq = 0.05
        max_abs = np.max(np.abs(dq))
        if max_abs > max_dq:
            dq *= max_dq / max_abs

        q_new = self.current_joints + dq

        q_min = np.array([-6.28, -2.61, -0.06, -6.28, -2.16, -6.28])
        q_max = np.array([6.28,   2.61,  5.24,  6.28,  2.16,  6.28])
        q_new = np.clip(q_new, q_min, q_max)

        traj = JointTrajectory()
        traj.joint_names = list(MASTER_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = q_new.tolist()
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points = [point]
        self.traj_pub.publish(traj)

        now = time.time()
        if now - self.last_print > 3.0:
            self.last_print = now
            p_ee, _, _ = self._lookup(EE_FRAME, BASE_FRAME)
            pos_str = f'[{p_ee[0]:.3f}, {p_ee[1]:.3f}, {p_ee[2]:.3f}]' if p_ee is not None else '?'
            self.get_logger().info(
                f'EE={pos_str} twist=[{twist[0]:+.4f},{twist[1]:+.4f},{twist[2]:+.4f}] '
                f'rx={self.rx_count}'
            )

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--discover', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = MasterInput(args.port, args.baud,
                       speed_scale=args.speed,
                       discover_mode=args.discover)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
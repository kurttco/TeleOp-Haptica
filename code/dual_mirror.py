#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool

try:
    from xarm_msgs.srv import SetInt16
    XARM_MSGS_AVAILABLE = True
except ImportError:
    XARM_MSGS_AVAILABLE = False


MASTER_TRAJ_TOPIC = '/L_lite6_traj_controller/joint_trajectory'
SLAVE_TRAJ_TOPIC  = '/R_lite6_traj_controller/joint_trajectory'


class DualMirrorNode(Node):
    def __init__(
        self,
        master='L',
        rate=10.0,
        delay=0.2,
        force_stop_threshold=12.5,
        force_resume_threshold=2.0,
        left_set_state_service='/L_xarm/set_state',
        right_set_state_service='/R_xarm/set_state',
        use_xarm_services=False,
    ):
        super().__init__('dual_mirror_node')

        if master.upper() == 'R':
            self.master_joints = [f'R_joint{i}' for i in range(1, 7)]
            self.slave_joints = [f'L_joint{i}' for i in range(1, 7)]
            self.slave_traj_topic = MASTER_TRAJ_TOPIC
            self.master_label = 'RIGHT'
            self.slave_label = 'LEFT'
        else:
            self.master_joints = [f'L_joint{i}' for i in range(1, 7)]
            self.slave_joints = [f'R_joint{i}' for i in range(1, 7)]
            self.slave_traj_topic = SLAVE_TRAJ_TOPIC
            self.master_label = 'LEFT'
            self.slave_label = 'RIGHT'

        self.delay = float(delay)
        self.force_stop_threshold = float(force_stop_threshold)
        self.force_resume_threshold = float(force_resume_threshold)

        self.last_master_positions = None
        self.last_slave_positions = None

        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.fm = 0.0

        self.contact_hold_active = False
        self.hold_slave_positions = None

        self.msg_count = 0
        self.last_print = time.time()
        self.last_force_print = time.time()

        self.use_xarm_services = bool(use_xarm_services and XARM_MSGS_AVAILABLE)
        self.left_set_state_cli = None
        self.right_set_state_cli = None

        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10
        )
        self.sub_force = self.create_subscription(
            WrenchStamped, '/slave/force', self.force_cb, 10
        )

        self.slave_traj_pub = self.create_publisher(
            JointTrajectory, self.slave_traj_topic, 10
        )

        # NEW: tell master to freeze/unfreeze
        self.master_hold_pub = self.create_publisher(Bool, '/master_hold', 10)

        if self.use_xarm_services:
            self.left_set_state_cli = self.create_client(SetInt16, left_set_state_service)
            self.right_set_state_cli = self.create_client(SetInt16, right_set_state_service)

            self.get_logger().info(f'Waiting for {left_set_state_service} ...')
            if not self.left_set_state_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{left_set_state_service} not available, disabling xArm services')
                self.use_xarm_services = False

            if self.use_xarm_services:
                self.get_logger().info(f'Waiting for {right_set_state_service} ...')
                if not self.right_set_state_cli.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warn(f'{right_set_state_service} not available, disabling xArm services')
                    self.use_xarm_services = False

        self.send_timer = self.create_timer(1.0 / rate, self.send_trajectory)
        self.hold_pub_timer = self.create_timer(0.05, self.publish_hold_state)

        self.get_logger().info('')
        self.get_logger().info('=' * 72)
        self.get_logger().info('DUAL LITE6 MIRROR + CONTACT HOLD (FX/FY/FZ)')
        self.get_logger().info(f'Master: {self.master_label}')
        self.get_logger().info(f'Slave:  {self.slave_label}')
        self.get_logger().info(f'Rate:   {rate} Hz')
        self.get_logger().info(f'Delay:  {delay} s')
        self.get_logger().info(f'Stop threshold:   {self.force_stop_threshold:.2f} N')
        self.get_logger().info(f'Resume threshold: {self.force_resume_threshold:.2f} N')
        self.get_logger().info(f'xArm services enabled: {self.use_xarm_services}')
        self.get_logger().info('Publishing hold state on: /master_hold')
        self.get_logger().info('=' * 72)
        self.get_logger().info('')

    def publish_hold_state(self):
        msg = Bool()
        msg.data = self.contact_hold_active
        self.master_hold_pub.publish(msg)

    def call_set_state(self, client, value, label):
        if client is None:
            return

        req = SetInt16.Request()
        req.data = int(value)
        future = client.call_async(req)

        def _done_cb(fut):
            try:
                _ = fut.result()
                self.get_logger().info(f'{label}: set_state({value}) sent')
            except Exception as e:
                self.get_logger().error(f'{label}: set_state({value}) failed: {e}')

        future.add_done_callback(_done_cb)

    def stop_both_arms(self):
        if self.use_xarm_services:
            self.call_set_state(self.left_set_state_cli, 4, 'LEFT')
            self.call_set_state(self.right_set_state_cli, 4, 'RIGHT')

    def release_both_arms(self):
        if self.use_xarm_services:
            self.call_set_state(self.left_set_state_cli, 0, 'LEFT')
            self.call_set_state(self.right_set_state_cli, 0, 'RIGHT')

    def joint_cb(self, msg: JointState):
        master_positions = []
        slave_positions = []

        for jname in self.master_joints:
            if jname not in msg.name:
                return
            idx = msg.name.index(jname)
            master_positions.append(msg.position[idx])

        for jname in self.slave_joints:
            if jname not in msg.name:
                return
            idx = msg.name.index(jname)
            slave_positions.append(msg.position[idx])

        self.last_master_positions = master_positions
        self.last_slave_positions = slave_positions

    def force_cb(self, msg: WrenchStamped):
        self.fx = float(msg.wrench.force.x)
        self.fy = float(msg.wrench.force.y)
        self.fz = float(msg.wrench.force.z)
        self.fm = math.sqrt(self.fx**2 + self.fy**2 + self.fz**2)

        if (not self.contact_hold_active) and (self.fm >= self.force_stop_threshold):
            self.contact_hold_active = True
            self.hold_slave_positions = (
                list(self.last_slave_positions) if self.last_slave_positions is not None else None
            )
            self.get_logger().warn(
                f'CONTACT DETECTED -> HOLD BOTH |F|={self.fm:.2f} N '
                f'(Fx={self.fx:+.2f}, Fy={self.fy:+.2f}, Fz={self.fz:+.2f})'
            )
            self.stop_both_arms()

        elif self.contact_hold_active and (self.fm <= self.force_resume_threshold):
            self.contact_hold_active = False
            self.get_logger().info(
                f'CONTACT RELEASED -> RESUME BOTH |F|={self.fm:.2f} N '
                f'(Fx={self.fx:+.2f}, Fy={self.fy:+.2f}, Fz={self.fz:+.2f})'
            )
            self.release_both_arms()

    def publish_positions(self, joint_names, positions):
        if positions is None:
            return

        traj = JointTrajectory()
        traj.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(self.delay),
            nanosec=int((self.delay % 1.0) * 1e9)
        )

        traj.points = [point]
        self.slave_traj_pub.publish(traj)

    def send_trajectory(self):
        if self.last_master_positions is None:
            return

        now = time.time()

        if self.contact_hold_active:
            self.publish_positions(self.slave_joints, self.hold_slave_positions)

            if now - self.last_force_print > 2.0:
                self.last_force_print = now
                self.get_logger().warn(
                    f'HOLD ACTIVE | Fx={self.fx:+.2f} Fy={self.fy:+.2f} '
                    f'Fz={self.fz:+.2f} |F|={self.fm:.2f} N'
                )
            return

        self.publish_positions(self.slave_joints, self.last_master_positions)
        self.msg_count += 1

        if now - self.last_print > 3.0:
            self.last_print = now
            deg_str = ', '.join([f'{math.degrees(p):.1f}' for p in self.last_master_positions])
            self.get_logger().info(
                f'Mirror #{self.msg_count}: [{deg_str}] deg | '
                f'Fx={self.fx:+.2f} Fy={self.fy:+.2f} Fz={self.fz:+.2f} |F|={self.fm:.2f}'
            )


def main():
    parser = argparse.ArgumentParser(description='Dual Lite6 mirror + contact hold (Fx/Fy/Fz)')
    parser.add_argument('--master', default='L', choices=['L', 'R'])
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--delay', type=float, default=0.2)
    parser.add_argument('--force-stop-threshold', type=float, default=3.0)
    parser.add_argument('--force-resume-threshold', type=float, default=2.0)
    parser.add_argument('--left-set-state-service', default='/L_xarm/set_state')
    parser.add_argument('--right-set-state-service', default='/R_xarm/set_state')
    parser.add_argument('--use-xarm-services', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = DualMirrorNode(
        master=args.master,
        rate=args.rate,
        delay=args.delay,
        force_stop_threshold=args.force_stop_threshold,
        force_resume_threshold=args.force_resume_threshold,
        left_set_state_service=args.left_set_state_service,
        right_set_state_service=args.right_set_state_service,
        use_xarm_services=args.use_xarm_services,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mirror node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
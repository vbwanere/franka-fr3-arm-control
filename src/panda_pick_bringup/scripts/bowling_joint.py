#!/usr/bin/env python3
"""
Bowling action — joint-space trajectory for Franka FR3.
Ball is held by gripper friction, released by opening gripper at T_RELEASE.
"""

import math
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

WORLD_NAME = 'cricket_world'

ACTION_NAME = '/fr3_arm_controller/follow_joint_trajectory'
GRIPPER_TOPIC = '/fr3_gripper_controller/commands'
BALL_SDF_PATH = '/home/vbwanere/Vaibhav-GitHub/franka-panda-arm-control/install/panda_pick_bringup/share/panda_pick_bringup/models/cricket_ball/model.sdf'

# Gripper response time — ~150ms typical
GRIPPER_OPEN_DELAY = 0.20  # seconds — open gripper this much *before* T_RELEASE

def deg(d):
    return d * math.pi / 180.0

# Constant joint values
J1_HOLD = deg(-160.0)
J3_HOLD = deg(160.0)
J5_HOLD = deg(-9.0)
J6_HOLD = deg(130.0)
J7_HOLD = deg(40.0)

# Swinging joints
J2_START = deg(100.0)
J2_END   = deg(-40.0)
J4_START = deg(-7.0)
J4_END   = deg(-65.0)

# Timing (seconds, from trajectory start)
T_WINDUP        = 2.00
T_J4_TRIGGER    = 2.80
T_RELEASE       = 2.9
T_FOLLOWTHROUGH = 3.45

# When to trigger gripper open so it actually releases AT T_RELEASE
T_GRIPPER_OPEN  = T_RELEASE - GRIPPER_OPEN_DELAY


def make_point(j1, j2, j3, j4, j5, j6, j7, t):
    pt = JointTrajectoryPoint()
    pt.positions = [j1, j2, j3, j4, j5, j6, j7]
    pt.velocities = [0.0] * 7
    pt.accelerations = [0.0] * 7
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
    return pt


class BowlingJointRunner(Node):
    def __init__(self):
        super().__init__('bowling_joint_runner')

        self.action_client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)

        for _ in range(50):  # up to 5 seconds
            if self.gripper_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.gripper_pub.get_subscription_count() == 0:
            self.get_logger().error('No subscriber found on gripper topic')
        else:
            self.get_logger().info(
                f'Gripper publisher connected ({self.gripper_pub.get_subscription_count()} subscribers)'
            )

        self.get_logger().info(f'Waiting for action server at {ACTION_NAME}...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Action server unavailable')
        self.get_logger().info('Action server ready.')

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.recorded_t = []
        self.recorded_j2 = []
        self.recorded_j4 = []
        self.recording = False
        self.start_time = None
        self.gripper_fired = False

    def _js_cb(self, msg):
        self.current_joint_state = msg
        if self.recording:
            try:
                j2_idx = msg.name.index('fr3_joint2')
                j4_idx = msg.name.index('fr3_joint4')
            except ValueError:
                return
            t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
            self.recorded_t.append(t)
            self.recorded_j2.append(msg.position[j2_idx] * 180.0 / math.pi)
            self.recorded_j4.append(msg.position[j4_idx] * 180.0 / math.pi)

    def wait_for_joint_state(self):
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_gripper(self, position):
        """position in meters. 0.04 = fully open, 0.0 = closed."""
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.gripper_pub.publish(msg)

    def go_to_ready(self):
        READY = [0.0, deg(-45), 0.0, deg(-135), 0.0, deg(90), deg(45)]
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = READY
        pt.velocities = [0.0] * 7
        pt.time_from_start = Duration(sec=3)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info('Moving to ready pose...')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if not gh.accepted:
            return False
        
        # Pre-warm the gripper while the arm is moving
        self.spin_wait(1.0)  # give arm 1s to start moving
        self.set_gripper(0.04)  # open gripper concurrently with arm motion
        
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        self.get_logger().info('At ready pose.')
        return True
    
    def spin_wait(self, duration):
        """Wait `duration` seconds while spinning the executor."""
        end_time = self.get_clock().now().nanoseconds * 1e-9 + duration
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spawn_ball_at_gripper(self):
        """Spawn ball between the gripper fingers."""
        # First remove if exists (ignore failure)
        remove_cmd = [
            'ign', 'service', '-s', '/world/{WORLD_NAME}/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', 'name: "cricket_ball", type: MODEL',
        ]
        subprocess.run(remove_cmd, capture_output=True, text=True, timeout=5)
        self.spin_wait(0.3)

        # Spawn at the gripper position (ready pose puts gripper around (0.31, 0, 0.49))
        create_req = (
            f'sdf_filename: "{BALL_SDF_PATH}", '
            f'name: "cricket_ball", '
            f'pose: {{ position: {{ x: 0.31, y: 0.018, z: 0.51 }} }}'
        )
        create_cmd = [
            'ign', 'service', '-s', '/world/{WORLD_NAME}/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', create_req,
        ]
        self.get_logger().info('Spawning ball at gripper position...')
        result = subprocess.run(create_cmd, capture_output=True, text=True, timeout=10)
        if 'data: true' not in result.stdout:
            self.get_logger().error(f'Ball spawn failed: {result.stdout}')
            return False
        self.spin_wait(0.5)
        self.get_logger().info('Ball spawned.')
        return True

    def build_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        traj.points.append(make_point(
            J1_HOLD, J2_START, J3_HOLD, J4_START,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_WINDUP
        ))

        j2_at_trigger = J2_START + (J2_END - J2_START) * \
            ((T_J4_TRIGGER - T_WINDUP) / (T_RELEASE - T_WINDUP))
        traj.points.append(make_point(
            J1_HOLD, j2_at_trigger, J3_HOLD, J4_START,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_J4_TRIGGER
        ))

        traj.points.append(make_point(
            J1_HOLD, J2_END, J3_HOLD, J4_END,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_RELEASE
        ))

        traj.points.append(make_point(
            J1_HOLD, J2_END, J3_HOLD, J4_END,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_FOLLOWTHROUGH
        ))

        return traj

    def _fire_gripper_open(self):
        if self.gripper_fired:
            return
        self.set_gripper(0.04)  # open
        self.gripper_fired = True
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        self.get_logger().info(
            f'GRIPPER OPENED at t={elapsed:.3f}s '
            f'(target was {T_GRIPPER_OPEN}s, expecting release at ~{T_RELEASE}s)'
        )
        self.gripper_timer.cancel()

    def run(self):
        self.wait_for_joint_state()
        self.get_logger().info('Got initial joint state.')

        # 1. Move to ready pose
        self.go_to_ready()

        # 1. Partially close gripper (3cm gap, narrower than 7.3cm ball diameter)
        self.set_gripper(0.0)
        self.get_logger().info('Setting gripper to half-closed...')
        self.spin_wait(0.1)

        # 2. Now spawn the ball — fingers are already in position
        if not self.spawn_ball_at_gripper():
            return
        
        # 3. Close gripper around the ball
        self.set_gripper(0.005)
        self.get_logger().info('Closing gripper around ball...')
        self.spin_wait(0.1)

        # 5. Build and send bowling trajectory
        traj = self.build_trajectory()
        self.get_logger().info(f'Built bowling trajectory with {len(traj.points)} points')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info('Sending bowling trajectory...')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return

        # Start recording + schedule gripper open
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.recording = True
        self.gripper_fired = False
        self.gripper_timer = self.create_timer(T_GRIPPER_OPEN, self._fire_gripper_open)

        self.get_logger().info(
            f'Goal accepted. Gripper will open at t={T_GRIPPER_OPEN}s '
            f'(release expected at t≈{T_RELEASE}s)'
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.recording = False

        if hasattr(self, 'gripper_timer') and not self.gripper_fired:
            self.gripper_timer.cancel()
            self.get_logger().warn('Gripper timer never fired')

        result = result_future.result().result
        self.get_logger().info(f'Execution complete. Error code: {result.error_code}')

        self.set_gripper(0.0)
        self.get_logger().info('Closing gripper for cleanup...')
        self.spin_wait(1.0)

        self.plot_recorded(traj)

    def plot_recorded(self, traj):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        plan_t = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                  for pt in traj.points]
        plan_j2 = [pt.positions[1] * 180.0 / math.pi for pt in traj.points]
        plan_j4 = [pt.positions[3] * 180.0 / math.pi for pt in traj.points]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax1.plot(self.recorded_t, self.recorded_j2, 'b-', linewidth=1.5, label='Actual J2')
        ax1.plot(plan_t, plan_j2, 'bo--', markersize=8, label='Planned J2')
        ax1.axvline(T_GRIPPER_OPEN, color='orange', linestyle=':', alpha=0.7, label='Gripper open')
        ax1.axvline(T_RELEASE, color='red', linestyle=':', alpha=0.5, label='Release')
        ax1.set_ylabel('J2 — shoulder (deg)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Bowling trajectory: J2 and J4 vs time')

        ax2.plot(self.recorded_t, self.recorded_j4, 'g-', linewidth=1.5, label='Actual J4')
        ax2.plot(plan_t, plan_j4, 'gs--', markersize=8, label='Planned J4')
        ax2.axvline(T_GRIPPER_OPEN, color='orange', linestyle=':', alpha=0.7, label='Gripper open')
        ax2.axvline(T_RELEASE, color='red', linestyle=':', alpha=0.5, label='Release')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('J4 — elbow (deg)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('/tmp/bowling_actual.png', dpi=100)
        self.get_logger().info('Plot saved to /tmp/bowling_actual.png')


def main():
    rclpy.init()
    node = BowlingJointRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
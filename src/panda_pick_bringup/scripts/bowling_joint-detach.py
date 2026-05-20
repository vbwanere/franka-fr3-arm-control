#!/usr/bin/env python3
"""
Bowling action — joint-space trajectory for Franka FR3.

Direct control over shoulder (J2) and elbow (J4) with a staged release:
  - Phase A (0.00 → 0.85s): J2 sweeps -100° → +40°, J4 stays at -7°
  - Phase B (0.85s → 1.02s): J4 snaps -7° → -65° as J2 finishes
  - Phase C (1.02s → 1.45s): brief hold (follow-through)

All other joints (J1, J3, J5, J6, J7) held constant.

Bypasses MoveIt Cartesian planning entirely. Sends directly to the FR3
joint trajectory controller via FollowJointTrajectory action.
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
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty


# --- FR3 joint names (must match controller config exactly) ---
JOINT_NAMES = [
    'fr3_joint1',
    'fr3_joint2',
    'fr3_joint3',
    'fr3_joint4',
    'fr3_joint5',
    'fr3_joint6',
    'fr3_joint7',
]

# Action server topic for FR3 joint trajectory controller.
# This is the standard name for ros2_control's JointTrajectoryController.
# If your sim uses a different controller name, change this.
ACTION_NAME = '/fr3_arm_controller/follow_joint_trajectory'


def deg(d):
    return d * math.pi / 180.0


# --- Constant joint values (radians) ---
J1_HOLD = deg(-160.0)   # backed off from -166 limit
J3_HOLD = deg( 160.0)   # backed off from +166 limit
J5_HOLD = deg(  -9.0)
J6_HOLD = deg( 130.0)
J7_HOLD = deg(  40.0)

# --- Swinging joints ---
J2_START = deg( 100.0)
J2_END   = deg( -40.0)
J4_START = deg(  -7.0)
J4_END   = deg( -65.0)

# --- Timing (seconds) ---
T_WINDUP        = 4.00   # time to move from current pose to back-swing
T_J4_TRIGGER    = 4.85   # J4 starts moving (J2 nearly done)
T_RELEASE       = 5.02   # J2 and J4 reach end values
T_FOLLOWTHROUGH = 5.45   # hold at end pose

def make_point(j1, j2, j3, j4, j5, j6, j7, t, j2_vel=0.0, j4_vel=0.0):
    pt = JointTrajectoryPoint()
    pt.positions = [j1, j2, j3, j4, j5, j6, j7]
    pt.velocities = [0.0, j2_vel, 0.0, j4_vel, 0.0, 0.0, 0.0]
    pt.accelerations = [0.0] * 7
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
    return pt


class BowlingJointRunner(Node):
    def __init__(self):
        super().__init__('bowling_joint_runner')

        self.action_client = ActionClient(
            self, FollowJointTrajectory, ACTION_NAME
        )

        self.detach_pub = self.create_publisher(Empty, '/cricket_ball/detach', 10)

        self.get_logger().info(f'Waiting for action server at {ACTION_NAME}...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'Action server {ACTION_NAME} not available. '
                'Check controller name with: ros2 control list_controllers'
            )
            raise RuntimeError('Action server unavailable')
        self.get_logger().info('Action server ready.')

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        self.current_joint_state = msg

    def _fire_detach(self):
        if self.detach_fired:
            return
        self.detach_pub.publish(Empty())
        self.detach_fired = True
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        self.get_logger().info(f'BALL RELEASED at t={elapsed:.3f}s (target was {T_RELEASE}s)')
        self.detach_timer.cancel()

    def wait_for_joint_state(self):
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def go_to_ready(self):
        """Send arm to the default 'ready' pose and wait for it to arrive."""
        READY_POSE = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = READY_POSE
        pt.velocities = [0.0] * 7
        pt.time_from_start = Duration(sec=3, nanosec=0)  # 3 seconds to get there
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info('Moving to ready pose...')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Ready-pose goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('At ready pose.')
        return True
    
    def respawn_ball(self):
        """Remove existing ball (if any) and spawn a fresh one.
        The new ball's plugin re-attaches it to fr3_link7."""

        sdf_path = '/home/vbwanere/Vaibhav-GitHub/franka-panda-arm-control/install/panda_pick_bringup/share/panda_pick_bringup/models/cricket_ball/model.sdf'
        world_name = 'sensor_demo'

        # Step 1: Remove existing ball (ignore failure — first run won't have one)
        remove_cmd = [
            'ign', 'service',
            '-s', f'/world/{world_name}/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', 'name: "cricket_ball", type: MODEL',
        ]
        self.get_logger().info('Removing existing ball...')
        result = subprocess.run(remove_cmd, capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            self.get_logger().warn(f'Remove failed (may be first run): {result.stderr.strip()}')

        # Brief pause to let Ignition process the removal
        time.sleep(0.3)

        # Step 2: Spawn fresh ball at fr3_link7's approximate world pose
        # (Plugin in model.sdf will create the DetachableJoint to fr3_link7
        # at whatever pose the FR3 currently has.)
        create_req = (
            f'sdf_filename: "{sdf_path}", '
            f'name: "cricket_ball", '
            f'pose: {{ position: {{ x: 0.31, y: 0.0, z: 0.45 }} }}'
        )
        create_cmd = [
            'ign', 'service',
            '-s', f'/world/{world_name}/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', create_req,
        ]
        self.get_logger().info('Spawning fresh ball...')
        result = subprocess.run(create_cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            self.get_logger().error(f'Spawn failed: {result.stderr.strip()}')
            return False
        if 'data: true' not in result.stdout:
            self.get_logger().error(f'Spawn returned non-success: {result.stdout.strip()}')
            return False

        # Give plugin a moment to create the joint
        time.sleep(1.5)
        self.get_logger().info('Ball spawned and attached.')
        return True

    def build_trajectory(self):

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        # Point 0: back-swing top (windup destination)
        traj.points.append(make_point(
            J1_HOLD, J2_START, J3_HOLD, J4_START,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_WINDUP
        ))

        # Point 1: mid-swing, J4 trigger instant
        j2_at_trigger = J2_START + (J2_END - J2_START) * \
            ((T_J4_TRIGGER - T_WINDUP) / (T_RELEASE - T_WINDUP))
        traj.points.append(make_point(
            J1_HOLD, j2_at_trigger, J3_HOLD, J4_START,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_J4_TRIGGER
        ))

        # Point 2 (release): J2 still moving at sweep velocity
        # Sweep rate: (J2_END - J2_START) / (T_RELEASE - T_WINDUP) = (-40 - 100)/(3.02 - 2.0) = -137°/s
        j2_vel_at_release = (J2_END - J2_START) / (T_RELEASE - T_WINDUP)  # rad/s, negative

        traj.points.append(make_point(
            J1_HOLD, J2_END, J3_HOLD, J4_END,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_RELEASE,
            j2_vel=j2_vel_at_release,  # still moving!
            j4_vel=0.0                   # J4 stops at -65° (snap complete)
        ))

        # Point 3 (follow-through): J2 has continued past -40°, now decelerating
        J2_FOLLOWTHROUGH = deg(-80.0)   # arm swept further through

        traj.points.append(make_point(
            J1_HOLD, J2_FOLLOWTHROUGH, J3_HOLD, J4_END,
            J5_HOLD, J6_HOLD, J7_HOLD,
            T_FOLLOWTHROUGH,
            j2_vel=0.0,  # now stopped
            j4_vel=0.0
        ))

        return traj

    def run(self):
        self.wait_for_joint_state()
        self.get_logger().info('Got initial joint state.')

        # 1. Send arm to known "ready" pose so link7 is at a predictable location
        self.go_to_ready()

        # # 2. Respawn the ball (re-attaches to link7 at ready pose)
        # if not self.respawn_ball():
        #     self.get_logger().error('Ball respawn failed, aborting')
        #     return

        # 3. Build and execute the bowling trajectory (the existing code)
        traj = self.build_trajectory()
        self.get_logger().info(
            f'Built bowling trajectory with {len(traj.points)} points '
            f'over {T_FOLLOWTHROUGH}s'
        )

        # Storage for recorded joint states during execution
        self.recorded_t = []
        self.recorded_j2 = []
        self.recorded_j4 = []
        self.recording = False
        self.start_time = None

        # Override joint state callback to record during execution
        def record_cb(msg):
            if not self.recording:
                return
            # joint_states comes in a weird order — find J2 and J4 by name
            try:
                j2_idx = msg.name.index('fr3_joint2')
                j4_idx = msg.name.index('fr3_joint4')
            except ValueError:
                return
            t = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
            self.recorded_t.append(t)
            self.recorded_j2.append(msg.position[j2_idx] * 180.0 / math.pi)
            self.recorded_j4.append(msg.position[j4_idx] * 180.0 / math.pi)

        # Replace existing subscription
        self.destroy_subscription(self._sub if hasattr(self, '_sub') else None)
        self._record_sub = self.create_subscription(
            JointState, '/joint_states', record_cb, 100
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info('Sending bowling trajectory...')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return

        # Start recording
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.recording = True

        # Schedule the detach exactly T_RELEASE seconds from now
        self.detach_fired = False
        # self.detach_timer = self.create_timer(T_RELEASE, self._fire_detach)

        self.get_logger().info(
            f'Goal accepted. Executing, recording, and scheduled detach at t={T_RELEASE}s...'
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.recording = False

        # Safety: cancel timer if it hasn't fired (shouldn't happen)
        if not self.detach_fired:
            # self.detach_timer.cancel()
            self.get_logger().warn('Detach timer never fired — trajectory completed too fast?')

        result = result_future.result().result
        self.get_logger().info(f'Execution complete. Error code: {result.error_code}')

        # Plot recorded data
        self.plot_recorded(traj)

    def plot_recorded(self, traj):
        import matplotlib.pyplot as plt

        # Planned values from keyframes
        plan_t = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                for pt in traj.points]
        plan_j2 = [pt.positions[1] * 180.0 / math.pi for pt in traj.points]
        plan_j4 = [pt.positions[3] * 180.0 / math.pi for pt in traj.points]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # J2
        ax1.plot(self.recorded_t, self.recorded_j2, 'b-', linewidth=1.5,
                label='Actual J2', alpha=0.8)
        ax1.plot(plan_t, plan_j2, 'bo--', markersize=8, label='Planned J2',
                alpha=0.5)
        ax1.axvline(T_J4_TRIGGER, color='gray', linestyle=':', alpha=0.5)
        ax1.axvline(T_RELEASE, color='red', linestyle=':', alpha=0.5,
                    label='Release')
        ax1.set_ylabel('J2 — shoulder (deg)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Bowling trajectory: J2 and J4 vs time')

        # J4
        ax2.plot(self.recorded_t, self.recorded_j4, 'g-', linewidth=1.5,
                label='Actual J4', alpha=0.8)
        ax2.plot(plan_t, plan_j4, 'gs--', markersize=8, label='Planned J4',
                alpha=0.5)
        ax2.axvline(T_J4_TRIGGER, color='gray', linestyle=':', alpha=0.5,
                    label=f'J4 trigger')
        ax2.axvline(T_RELEASE, color='red', linestyle=':', alpha=0.5,
                    label='Release')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('J4 — elbow (deg)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('/tmp/bowling_actual.png', dpi=100)
        self.get_logger().info('Plot saved to /tmp/bowling_actual.png')
        plt.show()


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
#!/usr/bin/env python3
"""
Bowling action trajectory for Franka FR3.

Phase 1: just get the swing shape right in sim. No ball, no release event.
Trajectory is scaled to fit FR3 reachable workspace (~600mm radius from a
virtual 'shoulder pivot' that lives inside the FR3's natural reach zone).

Coordinates are in fr3_link0 frame:
  +X = down-pitch (toward batsman)
  +Y = side (right-arm bowler bowls in -Y plane)
  +Z = up
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


# --- FR3 MoveIt config ---
PLANNING_FRAME = 'fr3_link0'
GROUP_NAME = 'fr3_arm'
EEF_LINK = 'fr3_hand_tcp'

# --- Bowling keyframes ---
# (time_from_start_s, x, y, z, qx, qy, qz, qw)
# Scaled to fit FR3 workspace. Shoulder pivot virtually at (0.30, 0.0, 0.50).
# Swing radius ~0.30m. Action is in the X-Z plane (Y near zero through release).
#
# Orientation convention: fr3_hand_tcp Z-axis = "approach" direction
# (what would point at the ball). We sweep this Z-axis through the bowling arc.
keyframes = [
    # 0. Stance — low and slightly back
    (0.00,  0.40, -0.10, 0.25,  1.0, 0.0, 0.0, 0.0),
    # 1. Back-swing top — pulled back and lifted
    (0.50,  0.25, -0.15, 0.55,  1.0, 0.0, 0.0, 0.0),
    # 2. Vertical load — highest point, slightly behind release
    (0.85,  0.35, -0.05, 0.75,  1.0, 0.0, 0.0, 0.0),
    # 3. Pre-release — forward & descending
    (0.95,  0.50, -0.05, 0.65,  1.0, 0.0, 0.0, 0.0),
    # 4. Release — full forward extension
    (1.02,  0.58, -0.05, 0.50,  1.0, 0.0, 0.0, 0.0),
    # 5. Follow-through mid — sweeping across
    (1.20,  0.50,  0.10, 0.30,  1.0, 0.0, 0.0, 0.0),
    # 6. Follow-through end — low, across body
    (1.45,  0.35,  0.20, 0.20,  1.0, 0.0, 0.0, 0.0),
]


def make_pose(kf):
    p = Pose()
    p.position.x, p.position.y, p.position.z = kf[1], kf[2], kf[3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        kf[4], kf[5], kf[6], kf[7]
    return p


class BowlingRunner(Node):
    def __init__(self):
        super().__init__('bowling_runner')

        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for MoveIt services...')
        self.cart_client.wait_for_service()
        self.exec_client.wait_for_server()
        self.get_logger().info('MoveIt ready.')

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        self.current_joint_state = msg

    def wait_for_joint_state(self):
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def retime_trajectory(self, trajectory):
        """
        Rescale trajectory point timestamps to match keyframe timing.
        Strategy: compute each point's Cartesian-distance progress along the
        full waypoint path, then map that progress onto the piecewise-linear
        time profile defined by `keyframes`.
        """
        points = trajectory.joint_trajectory.points
        if not points:
            return

        # Cumulative Cartesian distance between keyframes
        kf_cum_dist = [0.0]
        for i in range(1, len(keyframes)):
            dx = keyframes[i][1] - keyframes[i-1][1]
            dy = keyframes[i][2] - keyframes[i-1][2]
            dz = keyframes[i][3] - keyframes[i-1][3]
            kf_cum_dist.append(kf_cum_dist[-1] + math.sqrt(dx*dx + dy*dy + dz*dz))

        total_dist = kf_cum_dist[-1]
        if total_dist <= 0:
            self.get_logger().warn('Zero-length path, skipping retime')
            return

        kf_fracs = [d / total_dist for d in kf_cum_dist]
        kf_times = [kf[0] for kf in keyframes]

        # Cumulative joint-space distance per trajectory point (proxy for progress)
        # We use joint-space because Cartesian positions aren't in the trajectory msg.
        joint_cum = [0.0]
        for i in range(1, len(points)):
            d = sum(abs(a - b) for a, b in zip(points[i].positions, points[i-1].positions))
            joint_cum.append(joint_cum[-1] + d)

        total_joint = joint_cum[-1]
        if total_joint <= 0:
            return

        # Map each point's joint-distance fraction onto the keyframe time profile
        for i, pt in enumerate(points):
            frac = joint_cum[i] / total_joint

            # Find which keyframe segment this fraction falls into
            seg = len(kf_fracs) - 1
            for j in range(1, len(kf_fracs)):
                if frac <= kf_fracs[j]:
                    seg = j
                    break

            prev_frac, next_frac = kf_fracs[seg-1], kf_fracs[seg]
            prev_time, next_time = kf_times[seg-1], kf_times[seg]

            if next_frac > prev_frac:
                ratio = (frac - prev_frac) / (next_frac - prev_frac)
                t = prev_time + ratio * (next_time - prev_time)
            else:
                t = next_time

            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)

        # Ensure monotonically increasing timestamps (safety)
        for i in range(1, len(points)):
            t_prev = points[i-1].time_from_start.sec + points[i-1].time_from_start.nanosec * 1e-9
            t_cur = points[i].time_from_start.sec + points[i].time_from_start.nanosec * 1e-9
            if t_cur <= t_prev:
                t_cur = t_prev + 0.001
                points[i].time_from_start = Duration(
                    sec=int(t_cur),
                    nanosec=int((t_cur - int(t_cur)) * 1e9)
                )

    def run(self):
        self.wait_for_joint_state()
        self.get_logger().info('Got initial joint state.')

        waypoints = [make_pose(kf) for kf in keyframes]
        self.get_logger().info(f'Generated {len(waypoints)} bowling keyframes')

        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = GROUP_NAME
        req.link_name = EEF_LINK
        req.waypoints = waypoints
        req.max_step = 0.01           # 1cm step — match ellipse demo
        req.jump_threshold = 0.0      # disabled (matches ellipse demo)
        req.avoid_collisions = True

        start_state = RobotState()
        start_state.joint_state = self.current_joint_state
        req.start_state = start_state

        self.get_logger().info('Computing Cartesian path...')
        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        self.get_logger().info(
            f'Path planning result: {resp.fraction*100:.1f}% achieved, '
            f'{len(resp.solution.joint_trajectory.points)} trajectory points'
        )

        if resp.fraction < 0.90:
            self.get_logger().error(
                f'Cartesian planning achieved only {resp.fraction*100:.1f}%. '
                'Likely workspace or orientation issue. Inspect the keyframes '
                'or relax the orientation requirements.'
            )
            return

        self.get_logger().info('Retiming trajectory to match keyframe timing...')
        # self.retime_trajectory(resp.solution)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution

        self.get_logger().info('Executing bowling action...')
        send_future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected by /execute_trajectory')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Bowling action complete.')


def main():
    rclpy.init()
    node = BowlingRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
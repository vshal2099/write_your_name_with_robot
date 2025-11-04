#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class TextDrawVishal(Node):
    def __init__(self):
        super().__init__('text_draw_vishal')
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory...')
        self.arm_client.wait_for_server()
        self.get_logger().info('✅ Connected to arm controller!')

        self.joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]
        self.base_pose = [0.0, -1.0, 0.6, -0.4, 0.3, 0.0]

        # For marker visualization
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.draw_text("VISHAL")

    def interpolate_pattern(self, pattern, samples=20):
        """Interpolate between defining waypoints for smoothness."""
        pattern_array = np.array(pattern)
        if len(pattern_array) < 2:
            return [pattern_array[0].tolist()]
        interp_points = []
        for i in range(len(pattern_array) - 1):
            start, end = pattern_array[i], pattern_array[i+1]
            for alpha in np.linspace(0.0, 1.0, samples):  # 20 steps between each pair
                pt = (1-alpha)*start + alpha*end
                interp_points.append(pt.tolist())
        return interp_points

    def get_letter_pattern(self, letter):
        b = self.base_pose
        p = {
            'V': [
                [b[0], b[1], b[2]],
                [b[0] + 0.15, b[1], b[2] - 0.15],
                [b[0] + 0.30, b[1], b[2]],
            ],
            'I': [
                [b[0], b[1], b[2]],
                [b[0], b[1], b[2] - 0.2],
            ],
            'S': [
                [b[0], b[1], b[2]],
                [b[0] + 0.15, b[1], b[2]],
                [b[0] + 0.15, b[1], b[2] - 0.1],
                [b[0], b[1], b[2] - 0.1],
                [b[0], b[1], b[2] - 0.2],
                [b[0] + 0.15, b[1], b[2] - 0.2],
            ],
            'H': [
                [b[0], b[1], b[2]],
                [b[0], b[1], b[2] - 0.2],
                [b[0] + 0.15, b[1], b[2] - 0.1],
                [b[0] + 0.30, b[1], b[2]],
                [b[0] + 0.30, b[1], b[2] - 0.2],
            ],
            'A': [
                [b[0], b[1], b[2] - 0.2],
                [b[0] + 0.15, b[1], b[2]],
                [b[0] + 0.30, b[1], b[2] - 0.2],
                [b[0] + 0.075, b[1], b[2] - 0.1],
                [b[0] + 0.225, b[1], b[2] - 0.1],
            ],
            'L': [
                [b[0], b[1], b[2]],
                [b[0], b[1], b[2] - 0.2],
                [b[0] + 0.25, b[1], b[2] - 0.2],
            ],
        }
        # Smooth/interpolate for more waypoints
        return self.interpolate_pattern(p.get(letter, [b]), samples=20)

    def send_arm_trajectory(self, waypoints, publish_trace=False):
        """Send trajectory and optionally publish writing trace."""
        pts = []
        t = 0
        trace_points = []
        for wp in waypoints:
            point = JointTrajectoryPoint()
            positions = [wp[0], wp[1], wp[2], -0.4, 0.3, 0.0]
            point.positions = positions
            t += 0.3
            point.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9))
            pts.append(point)
            # Only collect marker trace points if actively writing (for letters)
            if publish_trace:
                trace_points.append(Point(x=wp[0], y=wp[1], z=wp[2]))

        # Send trajectory to arm
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = pts
        self.arm_client.send_goal_async(goal)
        self.get_logger().info(f"Drawing segment with {len(pts)} points...")
        time.sleep(len(pts) * 0.32)  # crude wait for motion to finish

        # Publish marker if requested (only during actual writing)
        if publish_trace and trace_points:
            self.publish_marker(trace_points)

    def publish_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "writing_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = points
        self.marker_pub.publish(marker)
        self.get_logger().info("Published marker trace for letter.")

    def draw_text(self, text):
        spacing = 0.35
        start_pose = [self.base_pose[0], self.base_pose[1], self.base_pose[2]]
        for idx, ch in enumerate(text):
            # Offset each letter along x for spacing
            pat = self.get_letter_pattern(ch)
            if len(pat) <= 1:
                self.get_logger().warn(f"No stroke data for '{ch}', skipping.")
                continue
            for p in pat:
                p[0] += idx * spacing
            self.get_logger().info(f"✏️ Drawing letter {ch}")

            # Move to start of letter (no marker)
            pre_pos = [pat[0][0], pat[0][1], pat[0][2]]
            self.send_arm_trajectory([pre_pos], publish_trace=False)
            time.sleep(0.2)
            # Draw letter (marker trace ON)
            self.send_arm_trajectory(pat, publish_trace=True)
            time.sleep(0.6)

        self.get_logger().info("✅ Finished writing VISHAL!")

def main(args=None):
    rclpy.init(args=args)
    node = TextDrawVishal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
"""
Node to set up permanent Allowed Collision Matrix entries for octomap.
This runs once at startup and allows specified robot links to collide with the octomap.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry


class OctomapACMNode(Node):
    def __init__(self):
        super().__init__('octomap_acm_node')
        
        # Links to allow collision with octomap
        self.declare_parameter('links_to_allow', ['base_link', 'shoulder_link'])
        self.links_to_allow = self.get_parameter('links_to_allow').get_parameter_value().string_array_value
        
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # Wait for move_group to be ready, then publish ACM
        self.timer = self.create_timer(2.0, self.publish_acm)
        
        self.get_logger().info(f'Octomap ACM node started. Will allow collision for: {self.links_to_allow}')
    
    def publish_acm(self):
        # Only run once
        self.timer.cancel()
        
        msg = PlanningScene()
        msg.is_diff = True
        
        # Build ACM entries: each link paired with <octomap>
        all_names = self.links_to_allow + ['<octomap>']
        n = len(all_names)
        
        msg.allowed_collision_matrix.entry_names = all_names
        msg.allowed_collision_matrix.entry_values = []
        
        for i, name in enumerate(all_names):
            entry = AllowedCollisionEntry()
            entry.enabled = []
            for j, other_name in enumerate(all_names):
                if name == '<octomap>' and other_name in self.links_to_allow:
                    entry.enabled.append(True)  # Allow octomap to collide with robot links
                elif name in self.links_to_allow and other_name == '<octomap>':
                    entry.enabled.append(True)  # Allow robot links to collide with octomap
                else:
                    entry.enabled.append(False)  # Don't modify other pairs
            msg.allowed_collision_matrix.entry_values.append(entry)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published ACM: allowed {self.links_to_allow} to collide with octomap')
        
        # Shutdown after publishing
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OctomapACMNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

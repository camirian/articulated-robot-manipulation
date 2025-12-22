#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import sys

class PerformPick(Node):
    def __init__(self):
        super().__init__('perform_pick')
        
        # Subscribe to Object Pose (from Perception Node)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.pose_callback,
            1)
            
        # Publisher to MoveIt Interface (move_to_pose.cpp)
        self.target_publisher = self.create_publisher(PoseStamped, '/target_pose', 1)
        
        self.buffer = []
        self.samples_needed = 1 # Fire immediately on first detection
        self.get_logger().info('Looking for object...')

    def pose_callback(self, msg):
        self.buffer.append(msg)
        self.get_logger().info(f'Detected potential target ({len(self.buffer)}/{self.samples_needed})')
        
        if len(self.buffer) >= self.samples_needed:
            self.execute_pick()

    def execute_pick(self):
        # Average the poses for stability (Simplified: Just take the last one or average)
        # For robustness, we'll just take the latest valid one + an offset for "Picking"
        
        target = self.buffer[-1]
        
        # Add a Grasp Offset?
        # The detected pose is the Center of the Cube.
        # We want to grip it.
        # The Cube is small (5cm).
        # We probably want to approach from above (Z-axis of object?).
        # Or simple: Move to (X, Y, Z + 0.1) aka Pre-Grasp.
        
        # NOTE: This simple node just forwards the pose to "move_to_pose".
        # "move_to_pose" simply moves the arm to that coordinate.
        
        # Strategy:
        # 1. Publish Pre-Grasp Pose (Z + 10cm)
        # 2. Wait
        # 3. Publish Grasp Pose (Z)
        
        # Because 'move_to_pose' is a simple subscriber, we can't easily sequence unless we sleep.
        # This is a 'Scripted' behavior.
        
        self.get_logger().info('Target Acquired! Sending Command to MoveIt...')
        
        # Adjust target for simple "Go To" test
        # Let's try to go 5cm above the object to verify alignment
        # Note: If target frame is camera, 'Z' is depth. "Above" usually means -Y in Camera frame or something else.
        # This is tricky without TF.
        # Let's TRUST MoveIt TF. We send the pose as is.
        # But we want to modify the pose relative to WORLD or HAND?
        # The message is in `panda_hand_camera`.
        # The 'Z' in camera frame is 'forward' (depth). 'Y' is down? 'X' is right?
        # Standard Camera: Z forward, X right, Y down.
        # Red Cube is on table. Camera looks down.
        # So Z is distance to table.
        # We want to stop *before* hitting the table.
        # So target Z should be slightly less (closer to camera)?
        # Or if we want to hover, we want Z - 0.1m.
        
        target.pose.position.z -= 0.2 # Stop 20cm before the object
        
        self.target_publisher.publish(target)
        self.get_logger().info(f'Sent Target: {target.pose.position}')
        
        # Stop listening
        self.destroy_subscription(self.subscription)
        self.get_logger().info('Mission Complete. Exiting.')
        
        # We allow some time for the message to send then exit
        # rclpy.spin_once(self, timeout_sec=1) # Need main loop handling
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PerformPick()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('perform_pick').info('Done.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

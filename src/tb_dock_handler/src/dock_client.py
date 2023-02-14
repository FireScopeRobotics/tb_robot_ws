#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
from irobot_create_msgs.action import Undock, DockServo

# ros2 action send_goal /undock irobot_create_msgs/action/Undock {}

class ROS_Dock_Client(Node):

    def __init__(self):
        super().__init__('tb_dock_handle_client')
        self._undock_action_client = ActionClient(self, Undock, '/undock')
        
        self._dock_action_client = ActionClient(self, DockServo, '/dock')

        self.tb_dock_sub = self.create_subscription(
            String, '/tb_dock_commands', self.dock_command_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.tb_dock_pub = self.create_publisher(String, '/tb_dock_commands_result', 10)

    def dock_command_callback(self, msg):
        if msg.data=='u':
            self.send_undock_goal()
        elif msg.data=='d':
            self.send_dock_goal()

    def send_undock_goal(self):
        """Send ActionServer a new goal"""

        goal_msg = Undock.Goal()
        # Send goal to ActionServer
        self._undock_action_client.wait_for_server()
        # self._send_goal_future = self._undock_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future = self._undock_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_undock_response_callback)

    def goal_undock_response_callback(self, future):
        """Parse whether ActionServer has accepted new goal"""
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            feedback = "Goal Rejected"
            self.get_logger().info(feedback)
        else:    
            self.get_logger().info('Goal accepted')
            feedback = "Goal Accepted"
            # Setup callback for ActionServer result
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.undock_get_result_callback)

        msg = String()
        msg.data = feedback
        self.tb_dock_pub.publish(msg)

    def undock_get_result_callback(self, future):
        """Parse ActionServer result"""
        result = future.result().result
        self.get_logger().info(f"{result}")
        msg = String()
        msg.data = f"{result}"
        self.tb_dock_pub.publish(msg)


    def send_dock_goal(self):
        """Send ActionServer a new goal"""

        goal_msg = DockServo.Goal()

        # Send goal to ActionServer
        self._dock_action_client.wait_for_server()
        # self._send_goal_future = self._dock_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future = self._dock_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_dock_response_callback)

    def goal_dock_response_callback(self, future):
        """Parse whether ActionServer has accepted new goal"""
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            feedback = "Goal Rejected"
            self.get_logger().info(feedback)
        else:    
            self.get_logger().info('Goal accepted')
            feedback = "Goal Accepted"
            # Setup callback for ActionServer result
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.dock_get_result_callback)

        msg = String()
        msg.data = feedback
        self.tb_dock_pub.publish(msg)
        
    def dock_get_result_callback(self, future):
        """Parse ActionServer result"""
        result = future.result().result
        self.get_logger().info(f"{result}")
        msg = String()
        msg.data = f"{result}"
        self.tb_dock_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = ROS_Dock_Client()
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

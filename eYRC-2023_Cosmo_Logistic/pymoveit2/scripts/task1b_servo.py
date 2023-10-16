#!/usr/bin/env python3

from threading import Thread
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

linear_speed = 1.0
angular_speed = 1.0

# Initialize message based on passed arguments
__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()
__twist_msg.twist.linear.x = linear_speed
__twist_msg.twist.linear.y = linear_speed
__twist_msg.twist.linear.z = linear_speed
__twist_msg.twist.angular.x = angular_speed
__twist_msg.twist.angular.y = angular_speed
__twist_msg.twist.angular.z = angular_speed

def main():
    
    rclpy.init()

    # Create node for this example
    node = Node("task1b_servo")
    
    # Declare a list of goal poses
    goal_poses = [{
                    "position": [0.35, 0.1, 0.68],
                    "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
                    "cartesian": False
                  },
                  {
                    "position": [-0.37, 0.12, 0.397],
                    "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
                    "cartesian": False
                  },
                  {
                    "position": [0.194, -0.43, 0.701],
                    "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
                    "cartesian": False
                  },
                  {
                    "position": [-0.37, 0.12, 0.397],
                    "quat_xyzw": [0.5, 0.5, 0.5, 0.5],
                    "cartesian": False
                  }]

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    __twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    
    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),  # You need to define 'ur5' or use the actual robot's information.
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    
    linear = [0.16, 0.0, 0.22]
    angular = [0.0, 0.0, 0.0]
    
    def servo_motion():
        
        twist_msg = deepcopy(__twist_msg)
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.twist.linear.x *= linear[0]
        twist_msg.twist.linear.y *= linear[1]
        twist_msg.twist.linear.z *= linear[2]
        twist_msg.twist.angular.x *= angular[0]
        twist_msg.twist.angular.y *= angular[1]
        twist_msg.twist.angular.z *= angular[2]
        
        __twist_pub.publish(twist_msg)

    # Create timer for moving in a circular motion
    node.create_timer(0.02, servo_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()
    
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

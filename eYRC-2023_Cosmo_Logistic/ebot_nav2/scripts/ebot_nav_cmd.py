#! /usr/bin/env python3

# Import necessary libraries and modules
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time

def main():
    # Initialize the ROS2 system
    rclpy.init()

    # Create an instance of the BasicNavigator from nav2_simple_commander
    navigator = BasicNavigator()

    # Set the initial pose for navigation
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait until the navigation system becomes active
    navigator.waitUntilNav2Active()
    
    # Define a list of goal poses for the robot to navigate to
    goal_poses = [[1.8, 1.5, 0.707, 0.707],
                  [2.0, -7.0, -0.707, 0.707],
                  [-3.0, 2.5, 0.707, 0.707]]

    goal = PoseStamped()
    index = 0

    # Loop through the list of goal poses
    while(index < len(goal_poses)):
        
        # Set the goal pose
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = goal_poses[index][0]
        goal.pose.position.y = goal_poses[index][1]
        goal.pose.orientation.w = goal_poses[index][2]
        goal.pose.orientation.z = goal_poses[index][3]

        # Navigate to the goal pose
        navigator.goToPose(goal)

        # Wait for the navigation task to complete
        while not navigator.isTaskComplete():
        
            # Get feedback from the navigation task
            feedback = navigator.getFeedback()
            if feedback and index % 5 == 0:

                # If the navigation time exceeds 600 seconds, cancel the task
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

        # Check the result of the navigation task
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            index = index + 1
            time.sleep(2)
            print('Goal succeeded!')
            
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            
        else:
            print('Goal has an invalid return status!')

    # Shutdown the navigator and exit the program
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()

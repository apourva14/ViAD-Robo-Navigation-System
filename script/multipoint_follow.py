#! /usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Define the initial position and orientation of the robot
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0510573
    initial_pose.pose.position.y = 0.02532
    initial_pose.pose.orientation.z = -0.0077
    initial_pose.pose.orientation.w = 0.9997

    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # Define list of goals as PoseStamped objects
    goal_pose = []

    # Goal 1
    goal_pose0 = PoseStamped()
    goal_pose0.header.frame_id = 'map'
    goal_pose0.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose0.pose.position.x = 1.060
    goal_pose0.pose.position.y = -8.404
    goal_pose0.pose.orientation.w = 0.6994
    goal_pose0.pose.orientation.z = 0.7146
    goal_pose.append(goal_pose0)

    # Goal 2
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -5.04046
    goal_pose1.pose.position.y = -3.27783
    goal_pose1.pose.orientation.z = -0.01523 
    goal_pose1.pose.orientation.w = 0.99988
    goal_pose.append(goal_pose1)

    # Goal 3
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.7924
    goal_pose2.pose.position.y = 5.6262
    goal_pose2.pose.orientation.z = -0.7031
    goal_pose2.pose.orientation.w = 0.7110
    goal_pose.append(goal_pose2)

    # Goal 4
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -4.37672
    goal_pose3.pose.position.y = -3.279634
    goal_pose3.pose.orientation.z = -0.01584
    goal_pose3.pose.orientation.w = 0.9998
    goal_pose.append(goal_pose3)

    # Navigate to each goal individually
    for goal in goal_pose:
        navigator.goToPose(goal)

        # Monitor the navigation progress
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Check for timeout (optional, if desired)
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            print("Goal was canceled!")
        elif result == TaskResult.FAILED:
            print("Goal failed!")
        else:
            print("Unknown result received!")

    # Shutdown navigation lifecycle once done
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()

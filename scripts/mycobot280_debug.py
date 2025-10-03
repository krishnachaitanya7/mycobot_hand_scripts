#!/usr/bin/env python3

import sys
from math import pi
from time import sleep

import rospy
import moveit_commander
import geometry_msgs.msg

from scripts.mycobot280_library_tutorial import add_box_to_scene
from scripts.mycobot280_library_tutorial import MyCobot280Operator


def main():
    try:
        # Initialize ROS and MoveIt
        rospy.init_node("mycobot280_debug", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        # Create robot operator
        operator = MyCobot280Operator()

        # Print robot info for debugging
        rospy.loginfo("============ Printing robot info for debugging")
        operator.print_robot_info()

        # Define safe joint positions
        DEFAULT_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Define more conservative positions for testing
        # Using smaller values to stay well within joint limits
        BOX_PICKUP_POSITION = [0.2, 0.3, -0.3, 0.3, 0.3, -0.2]
        BOX_DROP_POSITION = [0.0, 0.2, 0.3, -0.3, 0.1, -0.2]

        # Clear existing objects
        scene = moveit_commander.PlanningSceneInterface()
        rospy.loginfo("Clearing planning scene...")
        scene.clear()
        rospy.sleep(1.0)  # Give time for the scene to update

        # Try moving to initial position first
        rospy.loginfo("\n============ Moving to initial position")
        if operator.go_to_joint_state(DEFAULT_POSITION):
            rospy.loginfo("✅ Successfully moved to initial pose")
        else:
            rospy.logerr("❌ Failed to move to initial pose")
            return

        rospy.sleep(1)

        # Option to add obstacles
        add_obstacles = input("Do you want to add obstacles to the scene? (y/n): ")

        if add_obstacles.lower() == "y":
            # Add boxes far from the robot's work area
            rospy.loginfo("Adding Box 1 to the scene...")
            box_1_posestamped = geometry_msgs.msg.PoseStamped()
            box_1_posestamped.header.frame_id = "world"
            box_1_posestamped.pose.position.x = -0.35
            box_1_posestamped.pose.position.y = 0.35
            box_1_posestamped.pose.position.z = 0.0
            box_1_posestamped.pose.orientation.w = 1.0
            add_box_to_scene(box_1_posestamped, 0.05, 0.05, 0.05, "box_1")

            rospy.loginfo("Adding Box 2 to the scene...")
            box_2_posestamped = geometry_msgs.msg.PoseStamped()
            box_2_posestamped.header.frame_id = "world"
            box_2_posestamped.pose.position.x = 0.35
            box_2_posestamped.pose.position.y = -0.35
            box_2_posestamped.pose.position.z = 0.0
            box_2_posestamped.pose.orientation.w = 1.0
            add_box_to_scene(box_2_posestamped, 0.05, 0.05, 0.05, "box_2")

            # Wait for the planning scene to update
            rospy.sleep(1.0)

        # Try incremental movements
        rospy.loginfo("\n============ Testing small joint movements")

        # Start with small movements from current position
        current_joints = operator.move_group.get_current_joint_values()
        rospy.loginfo(f"Current joint values: {current_joints}")

        # Test joint 1 movement
        test_joints = current_joints.copy()
        test_joints[0] += 0.1  # Small change to first joint
        rospy.loginfo(f"Testing small change to joint 1: {test_joints}")

        if operator.go_to_joint_state(test_joints):
            rospy.loginfo("✅ Small joint 1 movement succeeded")
        else:
            rospy.logerr("❌ Failed to make small joint 1 movement")

        # Try the pickup position
        rospy.loginfo("\n============ Testing pickup position")
        rospy.loginfo(f"Target position: {BOX_PICKUP_POSITION}")

        if operator.go_to_joint_state(BOX_PICKUP_POSITION):
            rospy.loginfo("✅ Successfully moved to box pickup position")
        else:
            rospy.logerr("❌ Failed to move to box pickup position")

            # Try with intermediate waypoints
            rospy.loginfo("Attempting to reach position through intermediate waypoints...")

            # Current position
            current_joints = operator.move_group.get_current_joint_values()

            # Calculate waypoint at 50% of the way
            waypoint = []
            for i in range(len(current_joints)):
                if i < len(BOX_PICKUP_POSITION):
                    mid_val = current_joints[i] + (BOX_PICKUP_POSITION[i] - current_joints[i]) * 0.5
                    waypoint.append(mid_val)
                else:
                    waypoint.append(current_joints[i])

            rospy.loginfo(f"Moving to intermediate waypoint: {waypoint}")
            if operator.go_to_joint_state(waypoint):
                rospy.loginfo("✅ Successfully moved to intermediate waypoint")

                rospy.sleep(1)

                # Now try moving to the final destination
                rospy.loginfo("Now trying to move to final destination...")
                if operator.go_to_joint_state(BOX_PICKUP_POSITION):
                    rospy.loginfo("✅ Successfully moved to box pickup position")
                else:
                    rospy.logerr("❌ Failed to move to final destination")
            else:
                rospy.logerr("❌ Failed to move to intermediate waypoint")

        # Return to default position
        rospy.loginfo("\n============ Returning to initial position")
        if operator.go_to_joint_state(DEFAULT_POSITION):
            rospy.loginfo("✅ Successfully moved back to initial pose")
        else:
            rospy.logerr("❌ Failed to move back to initial pose")

        rospy.loginfo("\n============ Motion test completed")
        input("Press Enter to exit...")

    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
    finally:
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Program ended")


if __name__ == "__main__":
    main()

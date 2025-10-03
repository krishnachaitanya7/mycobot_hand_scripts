from math import pi
from time import sleep

import rospy
import actionlib
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg


def add_box_to_scene(
    box_pose: geometry_msgs.msg.PoseStamped, l: float, w: float, h: float, box_name, r=0.0, g=1.0, b=0.0, a=1.0
):

    scene = moveit_commander.PlanningSceneInterface()
    # The box_pose contains the coordinates of the box which starts at the bottom left corner
    # with length l, width w and height h
    # As Rviz considers the box coordinates to be the center of the box, we need to shift the box_pose
    # by half the length, width and height
    box_pose.pose.position.x += w / 2
    box_pose.pose.position.y += l / 2
    box_pose.pose.position.z += h / 2

    scene.add_box(box_name, box_pose, size=(w, l, h))


class MyCobot280Operator:
    def __init__(self) -> None:
        """
        This class is used to control the Franka Emika Panda robot.
        Be sure to activate ROS node and initialize moveit_commander before using this class.
        """
        self.robot = moveit_commander.RobotCommander()
        group_name = "arm_group"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planning_time(15.0)

        # Create a publisher for trajectory visualization
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
        )

    def display_trajectory(self, plan):
        """
        Display a trajectory in RViz
        :param plan: The trajectory to display
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.loginfo("Visualizing the trajectory in RViz")

    def print_robot_info(self):
        """
        Print information about the robot's joint limits and current state
        Useful for debugging when motion planning fails
        """
        rospy.loginfo("======= Robot Information =======")
        rospy.loginfo(f"Planning Group Names: {self.robot.get_group_names()}")

        # Get the names of all joints
        joint_names = self.move_group.get_active_joints()
        rospy.loginfo(f"Active Joints: {joint_names}")

        # Print current joint values
        current_joints = self.move_group.get_current_joint_values()
        rospy.loginfo(f"Current Joint Values: {current_joints}")

        # Try to get joint limits
        try:
            for i, joint_name in enumerate(joint_names):
                bounds = self.move_group.get_joint_bounds(joint_name)
                if bounds:
                    rospy.loginfo(f"Joint {joint_name} limits: {bounds}")
        except Exception as e:
            rospy.logwarn(f"Could not get joint bounds: {e}")

        # Print robot capabilities
        rospy.loginfo(f"Has end effector: {self.move_group.has_end_effector()}")
        rospy.loginfo(f"Planning frame: {self.move_group.get_planning_frame()}")
        rospy.loginfo(f"End effector link: {self.move_group.get_end_effector_link()}")

        rospy.loginfo("=================================")

    def attach_box(self, box_name):
        """
        Attach a box to the robot's end effector
        :param box_name: The name of the box to attach
        """
        eef_link = self.move_group.get_end_effector_link()
        touch_links = self.robot.get_link_names(group=self.move_group.get_name())
        scene = moveit_commander.PlanningSceneInterface()
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        rospy.loginfo(f"Attached box {box_name} to {eef_link}")

    def detach_box(self, box_name):
        """
        Detach a box from the robot's end effector
        :param box_name: The name of the box to detach
        """
        eef_link = self.move_group.get_end_effector_link()
        scene = moveit_commander.PlanningSceneInterface()
        scene.remove_attached_object(eef_link, name=box_name)
        rospy.loginfo(f"Detached box {box_name} from {eef_link}")

    def go_to_joint_state(self, target_pose):
        """
        This function is used to move the robot to a specific joint state.
        :param target_pose: A list of joint values (6 for a 6-DOF robot, 12 for a 12-DOF robot).
        :return: True if the execution was successful, False otherwise.
        """
        try:
            # Get current joint values for fallback and debugging
            current_joints = self.move_group.get_current_joint_values()
            # Round the current joints for better logging
            current_joints = [round(j, 3) for j in current_joints]
            rospy.loginfo(f"Current joints: {current_joints}")
            rospy.loginfo(f"Target joints: {target_pose}")

            # Increase planning time for complex movements
            self.move_group.set_planning_time(15.0)

            # Set the joint target
            self.move_group.set_joint_value_target(target_pose)

            # Plan the trajectory - check the return value format
            plan_result = self.move_group.plan()

            # Different MoveIt versions return different formats
            if isinstance(plan_result, tuple):
                # Newer MoveIt versions return (success, trajectory)
                success = plan_result[0]
                trajectory = plan_result[1]
            else:
                # Older versions just return the trajectory
                success = plan_result is not None and len(plan_result.joint_trajectory.points) > 0
                trajectory = plan_result

            if not success or trajectory is None:
                rospy.logwarn("Planning failed! No valid trajectory found.")
                return False

            # Display the planned trajectory in RViz
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(trajectory)
            self.display_trajectory_publisher.publish(display_trajectory)
            rospy.loginfo("Visualizing planned trajectory in RViz")

            # Execute the planned trajectory
            rospy.loginfo("Executing trajectory...")
            execution_status = self.move_group.execute(trajectory, wait=True)

            if execution_status:
                rospy.loginfo("Execution completed successfully")
            else:
                rospy.logwarn("Execution failed!")

            return execution_status

        except Exception as e:
            rospy.logerr(f"Error in go_to_joint_state: {str(e)}")
            return False


if __name__ == "__main__":
    rospy.init_node("mycobot280_operator", anonymous=True)
    moveit_commander.roscpp_initialize([])
    operator = MyCobot280Operator()
    print("============ Printing robot state")
    print(operator.robot.get_current_state())
    print("")
    print("============ Generating plan 1")
    input("Enter to Add Box 1 to the Scene: ")
    box_1_posestamped = geometry_msgs.msg.PoseStamped()
    box_1_posestamped.header.frame_id = "world"
    box_1_posestamped.pose.position.x = 0.15
    box_1_posestamped.pose.position.y = 0.0
    box_1_posestamped.pose.position.z = 0.0
    box_1_posestamped.pose.orientation.x = 0.0
    box_1_posestamped.pose.orientation.y = 0.0
    box_1_posestamped.pose.orientation.z = 0.0
    box_1_posestamped.pose.orientation.w = 1.0
    add_box_to_scene(box_1_posestamped, 0.2, 0.2, 0.2, "box_1")
    input("Enter to Add Box 2 to the Scene: ")
    box_2_posestamped = geometry_msgs.msg.PoseStamped()
    box_2_posestamped.header.frame_id = "world"
    box_2_posestamped.pose.position.x = 0.0
    box_2_posestamped.pose.position.y = 0.2
    box_2_posestamped.pose.position.z = 0.0
    box_2_posestamped.pose.orientation.x = 0.0
    box_2_posestamped.pose.orientation.y = 0.0
    box_2_posestamped.pose.orientation.z = 0.0
    box_2_posestamped.pose.orientation.w = 1.0
    add_box_to_scene(box_2_posestamped, 0.2, 0.2, 0.2, "box_2")
    input("Enter to Add Box 3 to the Scene: ")
    box_3_posestamped = geometry_msgs.msg.PoseStamped()
    box_3_posestamped.header.frame_id = "world"
    box_3_posestamped.pose.position.x = 0.2
    box_3_posestamped.pose.position.y = 0.08
    box_3_posestamped.pose.position.z = 0.22
    box_3_posestamped.pose.orientation.x = 0.0
    box_3_posestamped.pose.orientation.y = 0.0
    box_3_posestamped.pose.orientation.z = 0.0
    box_3_posestamped.pose.orientation.w = 1.0
    add_box_to_scene(box_3_posestamped, 0.02, 0.02, 0.02, "box_3")

    # Safe joint positions - adjusted to be within typical myCobot 280 joint limits
    DEFAULT_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    BOX_PICKUP_POSITION = [-2.844, 0.077, 0.833, 2.218, 0.268, 3.073]
    BOX_DROP_POSITION = [1.33, -1.99, 1.55, 0.31, 0.26, -0.01]

    input("Enter to Move the Robot to initial pose: ")
    # Move to initial pose
    if operator.go_to_joint_state(DEFAULT_POSITION):
        print("Successfully moved to initial pose")
    else:
        print("Failed to move to initial pose")
    sleep(2)

    input("Enter to Move the Robot to pick Box: ")
    # Move to pick box 1
    if operator.go_to_joint_state(BOX_PICKUP_POSITION):
        print("Successfully moved to box pickup position")
    else:
        print("Failed to move to box pickup position")
    sleep(2)

    input("Enter to Attach the Box to the Robot: ")
    # Attach the box to the robot
    operator.attach_box("box_3")
    if operator.go_to_joint_state(DEFAULT_POSITION):
        print("Successfully moved to initial pose")
    else:
        print("Failed to move to initial pose")
    sleep(2)

    input("Enter to Move the Robot to drop Box: ")
    # Move to drop box 1
    if operator.go_to_joint_state(BOX_DROP_POSITION):
        print("Successfully moved to box drop position")
    else:
        print("Failed to move to box drop position")
    sleep(2)

    input("Enter to Detach the Box from the Robot: ")
    # Detach the box from the robot
    operator.detach_box("box_3")
    sleep(2)
    input("Enter to Move the Robot to initial pose: ")
    # Move to initial pose
    if operator.go_to_joint_state(DEFAULT_POSITION):
        print("Successfully moved back to initial pose")
    else:
        print("Failed to move back to initial pose")

    print("============ Python tutorial demo complete!")

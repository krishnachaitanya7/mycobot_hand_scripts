import sys
import threading
from time import sleep
from tkinter import ttk
import tkinter as tk

import tf
import rospy
import rospkg
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


def clear_planning_scene():
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()


def add_box_to_scene(box_pose: geometry_msgs.msg.PoseStamped, l: float, w: float, h: float, box_name):
    scene = moveit_commander.PlanningSceneInterface()
    # The box_pose contains the coordinates of the box which starts at the bottom left corner
    # with length l, width w and height h
    # As Rviz considers the box coordinates to be the center of the box, we need to shift the box_pose
    # by half the length, width and height
    # TODO: Cross Check with Panda Link0 frame
    box_pose.pose.position.x += w / 2
    box_pose.pose.position.y += l / 2
    box_pose.pose.position.z += h / 2

    scene.add_box(box_name, box_pose, size=(w, l, h))


class BoxPositionGUI:
    def __init__(self, root):
        self.root = root
        root.title("Box Position Control")

        # Initial values
        self.x_pos = 0.0
        self.y_pos = 0.3
        self.z_pos = 0.0
        self.box_size = 0.1
        self.box_name = "box_2"
        self.scene = None

        # Initialize MoveIt scene
        self.setup_scene()

        # Create GUI elements
        self.create_widgets()

        # Update box for the first time
        self.update_box_position()

    def setup_scene(self):
        # Initialize moveit scene
        self.scene = moveit_commander.PlanningSceneInterface()
        self.scene.clear()

        # Create pose stamped for box position
        self.pose_stamped = geometry_msgs.msg.PoseStamped()
        self.pose_stamped.header.frame_id = "world"

        # Set orientation (fixed)
        self.pose_stamped.pose.orientation.x = 0.0
        self.pose_stamped.pose.orientation.y = 0.0
        self.pose_stamped.pose.orientation.z = 0.0
        self.pose_stamped.pose.orientation.w = 1.0

    def create_widgets(self):
        # Create frame for sliders
        slider_frame = ttk.LabelFrame(self.root, text="Box Position Control", padding="10 10 10 10")
        slider_frame.grid(column=0, row=0, padx=10, pady=10, sticky=(tk.W, tk.E, tk.N, tk.S))

        # X position slider
        ttk.Label(slider_frame, text="X Position:").grid(column=0, row=0, sticky=tk.W)
        self.x_slider = ttk.Scale(
            slider_frame,
            from_=-1.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            length=300,
            value=self.x_pos,
            command=self.on_x_change,
        )
        self.x_slider.grid(column=1, row=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.x_value_label = ttk.Label(slider_frame, text=f"{self.x_pos:.2f}")
        self.x_value_label.grid(column=2, row=0, sticky=tk.W)

        # Y position slider
        ttk.Label(slider_frame, text="Y Position:").grid(column=0, row=1, sticky=tk.W)
        self.y_slider = ttk.Scale(
            slider_frame,
            from_=-1.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            length=300,
            value=self.y_pos,
            command=self.on_y_change,
        )
        self.y_slider.grid(column=1, row=1, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.y_value_label = ttk.Label(slider_frame, text=f"{self.y_pos:.2f}")
        self.y_value_label.grid(column=2, row=1, sticky=tk.W)

        # Z position slider
        ttk.Label(slider_frame, text="Z Position:").grid(column=0, row=2, sticky=tk.W)
        self.z_slider = ttk.Scale(
            slider_frame,
            from_=0.0,
            to=1.0,
            orient=tk.HORIZONTAL,
            length=300,
            value=self.z_pos,
            command=self.on_z_change,
        )
        self.z_slider.grid(column=1, row=2, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.z_value_label = ttk.Label(slider_frame, text=f"{self.z_pos:.2f}")
        self.z_value_label.grid(column=2, row=2, sticky=tk.W)

        # Size slider
        ttk.Label(slider_frame, text="Box Size:").grid(column=0, row=3, sticky=tk.W)
        self.size_slider = ttk.Scale(
            slider_frame,
            from_=0.05,
            to=0.3,
            orient=tk.HORIZONTAL,
            length=300,
            value=self.box_size,
            command=self.on_size_change,
        )
        self.size_slider.grid(column=1, row=3, padx=5, pady=5, sticky=(tk.W, tk.E))
        self.size_value_label = ttk.Label(slider_frame, text=f"{self.box_size:.2f}")
        self.size_value_label.grid(column=2, row=3, sticky=tk.W)

        # Exit button
        self.exit_button = ttk.Button(slider_frame, text="Exit", command=self.exit_program)
        self.exit_button.grid(column=1, row=4, pady=10)

    def on_x_change(self, value):
        self.x_pos = float(value)
        self.x_value_label.configure(text=f"{self.x_pos:.2f}")
        self.update_box_position()

    def on_y_change(self, value):
        self.y_pos = float(value)
        self.y_value_label.configure(text=f"{self.y_pos:.2f}")
        self.update_box_position()

    def on_z_change(self, value):
        self.z_pos = float(value)
        self.z_value_label.configure(text=f"{self.z_pos:.2f}")
        self.update_box_position()

    def on_size_change(self, value):
        self.box_size = float(value)
        self.size_value_label.configure(text=f"{self.box_size:.2f}")
        self.update_box_position()

    def update_box_position(self):
        # Remove existing box
        self.scene.remove_world_object(self.box_name)

        # Update pose
        self.pose_stamped.pose.position.x = self.x_pos
        self.pose_stamped.pose.position.y = self.y_pos
        self.pose_stamped.pose.position.z = self.z_pos

        # Add updated box
        add_box_to_scene(self.pose_stamped, self.box_size, self.box_size, self.box_size, self.box_name)

    def exit_program(self):
        self.root.quit()
        rospy.signal_shutdown("User requested exit")


def run_ros_thread():
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down from keyboard interrupt")


if __name__ == "__main__":
    # Initialize ROS
    rospy.init_node("rviz_spawn_library", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # Start ROS spinner in a separate thread
    ros_thread = threading.Thread(target=run_ros_thread)
    ros_thread.daemon = True
    ros_thread.start()

    # Start Tkinter GUI
    root = tk.Tk()
    gui = BoxPositionGUI(root)

    # Run the GUI main loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("Shutting down")

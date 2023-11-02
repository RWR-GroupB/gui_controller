#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import ttk

class GuiInterface:
    def __init__(self, master):
        self.master = master

        # GUI elements 
        self.master.title("GUI Interface")

        # Joint 1
        self.joint_1_slider_label = ttk.Label(master, text="Joint 1 Position:")
        self.joint_1_slider_label.pack(pady=20)

        self.cmd_joint_1_slider = ttk.Scale(master, from_=0, to_=360, orient="horizontal", command=self.publish_cmd_joint_angle_1)
        self.cmd_joint_1_slider.pack(pady=20)

        self.joint_1_cmd_value_label = ttk.Label(master, text="Current Value: 0")
        self.joint_1_cmd_value_label.pack(pady=20)

        # Joint 2 
        self.joint_2_slider_label = ttk.Label(master, text="Joint 2 Position:")
        self.joint_2_slider_label.pack(pady=20)

        self.cmd_joint_2_slider = ttk.Scale(master, from_=0, to_=360, orient="horizontal", command=self.publish_cmd_joint_angle_2)
        self.cmd_joint_2_slider.pack(pady=20)

        self.joint_2_cmd_value_label = ttk.Label(master, text="Current Value: 0")
        self.joint_2_cmd_value_label.pack(pady=20)

        # Initialize ROS 
        rospy.init_node('gui_interface_node', anonymous=True)

        # Publishers 
        self.cmd_joint_angles_pub = rospy.Publisher('/hand/motors/cmd_joint_angles', Float32MultiArray, queue_size=1)

        self.joint_angles = [0.0, 0.0]

    def publish_cmd_joint_angle_1(self, value):
        joint_1_new_position = float(value)
        self.joint_angles[0] = joint_1_new_position

        self.joint_1_cmd_value_label.config(text=f"Joint 1 Commanded Position: {joint_1_new_position:.2f}") 

        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = self.joint_angles
        self.cmd_joint_angles_pub.publish(joint_angles_msg)
    
    def publish_cmd_joint_angle_2(self, value):
        joint_2_new_position = float(value)
        self.joint_angles[1] = joint_2_new_position

        self.joint_2_cmd_value_label.config(text=f"Joint 2 Commanded Position: {joint_2_new_position:.2f}") 

        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = self.joint_angles
        self.cmd_joint_angles_pub.publish(joint_angles_msg)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1000x750")
    app = GuiInterface(root)
    root.mainloop()

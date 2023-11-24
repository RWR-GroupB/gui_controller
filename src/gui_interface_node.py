#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import ttk

# Numbers correspond to position in list 
hand_finger_joint_map = {
    'index': {
        'index_mcp': 0,
        'index_pip-dip' : 1,
    },

    'middle': {
        'middle_mcp' : 2,
        'middle_pip-dip' : 3,
    },

    'pinky' : {
        'pinky_mcp' : 4,
        'pinky_pip-dip' : 5,
    },

    'thumb' : {
        'thumb_flexion-extension' : 6,
        'thumb_adduction-abduction' : 7,
        'thumb_mcp' : 8,
        'thumb_pip-dip': 9
    }
}

class GuiInterface:
    def __init__(self, master):
        self.master = master

        # GUI elements 
        self.master.title("GUI Control") 

        # Initialize ROS 
        rospy.init_node('gui_interface_node', anonymous=True)
        self.cmd_joint_angles_pub = rospy.Publisher('/hand/motors/cmd_joint_angles', Float32MultiArray, queue_size=1)

        self.joint_angles = [
            0.0,    # Index MCP
            0.0,    # Index PIP/DIP
            0.0,    # Middle MCP
            0.0,    # Middle PIP/DIP
            0.0,    # Pinky MCP
            0.0,    # Pinky PIP/DIP
            0.0,    # Thumb Flexion/Extension
            0.0,    # Thumb Adduction/Abduction
            0.0,    # Thumb MCP
            0.0,    # Thumb PIP/DIP
        ]

        # Creating the GUI components
        for i, finger_key in enumerate(hand_finger_joint_map):
            group_frame = ttk.LabelFrame(master, text=f"{finger_key}")
            group_frame.pack(padx=10, pady=10, fill="both", expand="yes")
            self.create_slider_group(group_frame, i, finger_key)


    def create_slider_group(self, frame, group_index, finger):
        finger_subcomponents_map = hand_finger_joint_map[finger]

        for j, finger_subcomponent_key in enumerate(finger_subcomponents_map):
            slider_label = ttk.Label(frame, text=f"{finger_subcomponent_key}")
            slider_label.pack(pady=5)
            slider = ttk.Scale(frame, from_=0, to_=359, orient="horizontal", 
                               command=lambda value, index=finger_subcomponents_map[finger_subcomponent_key]: self.update_slider_value(value, index))
            slider.pack(pady=5)

    def update_slider_value(self, value, index):
        self.joint_angles[index] = float(value)

    def start_ros_publish_loop(self):
        def publish_values():
            msg = Float32MultiArray()
            msg.data = self.joint_angles
            self.pub.publish(msg)
            self.master.after(50, publish_values)  # Publish at 20 Hz

        publish_values()


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1000x750")
    app = GuiInterface(root)
    root.mainloop()
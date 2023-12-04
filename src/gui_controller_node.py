#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import ttk

import numpy as np

# Numbers correspond to position in list self.joint_angles
hand_finger_joint_map = {
    'thumb' : {
        'thumb_adduction-abduction' : (0, (-35, 35)),
        'thumb_mcp' : (1, (0, 90)),
        'thumb_pip-dip': (2, (0, 90)),
    },

    'index': {
        'index_mcp': (3, (0, 90)),
        'index_pip-dip' : (4, (0, 90)),
    },

    'middle': {
        'middle_mcp' : (5, (0, 90)),
        'middle_pip-dip' : (6, (0, 90)),
    },

    'pinky' : {
        'pinky_mcp' : (7, (0, 90)),
        'pinky_pip-dip' : (8, (0, 90)),
    },
}

class GuiInterface:
    def __init__(self, master):
        self.master = master

        # GUI elements 
        self.master.title("GUI Control") 

        # Data storage
        self.joint_angles = [
            0.0,    # Thumb Adduction/Abduction
            0.0,    # Thumb MCP
            0.0,    # Thumb PIP/DIP
            0.0,    # Index MCP
            0.0,    # Index PIP/DIP
            0.0,    # Middle MCP
            0.0,    # Middle PIP/DIP
            0.0,    # Pinky MCP
            0.0,    # Pinky PIP/DIP
        ]

        self.value_labels = [None] * len(self.joint_angles)
        self.sliders = [None] * len(self.joint_angles)

        # Initialize ROS 
        rospy.init_node('gui_interface_node', anonymous=True)
        self.cmd_joint_angles_pub = rospy.Publisher('hand/motors/cmd_joint_angles', Float32MultiArray, queue_size=1)

        # self.get_joint_angles_sub = rospy.Subscriber('hand/motors/get_joint_angles', Float32MultiArray, self.get_joint_angles_callback, queue_size=1)

        # Creating the GUI components
        for i, finger_key in enumerate(hand_finger_joint_map):
            group_frame = ttk.LabelFrame(master, text=f"{finger_key}")
            group_frame.pack(padx=10, pady=10, fill="both", expand="yes")
            self.create_slider_group(group_frame, i, finger_key)

        # Slider to create gripping motion 
        self.grasp_slider_frame = ttk.Frame(master)
        self.grasp_slider_frame.pack(pady=10)

        self.grasp_slider_label = ttk.Label(self.grasp_slider_frame, text="Grasp Control")
        self.grasp_slider_label.pack(side=tk.LEFT, padx=5)

        self.grasp_slider = ttk.Scale(self.grasp_slider_frame, from_=0, to_=90, orient="horizontal", command=self.update_grasp_slider)
        self.grasp_slider.pack(side=tk.LEFT, padx=5)

        self.start_ros_publish_loop()

    def create_slider_group(self, frame, group_index, finger):
        finger_subcomponents_map = hand_finger_joint_map[finger]

        for j, finger_subcomponent_key in enumerate(finger_subcomponents_map):
            # Create frame for each slider and value label
            slider_frame = ttk.Frame(frame)
            slider_frame.pack(pady=5)

            # Create and pack the slider 
            slider_label = ttk.Label(frame, text=f"{finger_subcomponent_key}")
            slider_label.pack(pady=5)

            # Create and pack the slider 
            slider_index = finger_subcomponents_map[finger_subcomponent_key][0]
            slider_limit_min, slider_limit_max = finger_subcomponents_map[finger_subcomponent_key][1]
            slider = ttk.Scale(slider_frame, from_=slider_limit_min, to_=slider_limit_max, orient="horizontal")
            slider.pack(side=tk.LEFT, padx=5)
            slider.bind("<B1-Motion>", lambda event, index=slider_index: self.update_slider_value(event.widget.get(), index))

            # Storing the slider for later access
            self.sliders[slider_index] = slider

            # Create and pack the value label 
            value_label = ttk.Label(slider_frame, text="0")
            value_label.pack(side=tk.LEFT, padx=5)

            # Store the value label in a dictionary for later access
            self.value_labels[slider_index] = value_label

    def update_slider_value(self, value, index):
        self.joint_angles[index] = float(value)
        self.value_labels[index].config(text=f"{value:.2f}")

    def start_ros_publish_loop(self):
        def publish_values():
            msg = Float32MultiArray()
            msg.data = self.joint_angles
            self.cmd_joint_angles_pub.publish(msg)
            self.master.after(50, publish_values)  # Publish at 20 Hz

        publish_values()

    # def get_joint_angles_callback(self, msg):
    #     print(msg.data)
    #     for i, value in enumerate(msg.data):
    #         self.joint_angles[i] = value
    #         self.value_labels[i].config(text=f"{value:.2f}")

    #         if self.sliders[i] is not None:
    #             self.sliders[i].set(value)
        
    #     self.get_joint_angles_sub.unregister()

    def update_grasp_slider(self, value):
        thumb_adduction_abduction_index = hand_finger_joint_map['thumb']['thumb_adduction-abduction'][0]

        for i, slider in enumerate(self.sliders):
            # Thumb adduction-abduction should be at 0 when performing simple grasping
            slider_value = 0.0 if i == thumb_adduction_abduction_index else value

            if slider is not None:
                slider.set(slider_value)
        
        # Update all joint angles and value labels
        for i in range(len(self.joint_angles)):
            # Thumb adduction-abduction should be at 0 when performing simple grasping
            joint_angle_value = 0.0 if i == thumb_adduction_abduction_index else value

            self.update_slider_value(joint_angle_value, i, update_grasp=False)

    def update_slider_value(self, value, index, update_grasp=False):
        value = float(value)
        self.joint_angles[index] = value
        self.value_labels[index].config(text=f"{value:.2f}")

        # Update grasp_slider only if necessary to avoid infinite loops
        if update_grasp:
            self.grasp_slider.set(np.mean(self.joint_angles))

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1000x750")
    app = GuiInterface(root)
    root.mainloop()

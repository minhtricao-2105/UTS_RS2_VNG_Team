import os,sys,rospy,datetime,subprocess
import tkinter as tk
import cv2 as cv

from tkinter import Tk, Label, ttk, Canvas
from cv_bridge import CvBridge
from PIL import Image
from ttkthemes import ThemedStyle
from sensor_msgs.msg import Image
from PIL import Image as PILImage


# Function to add logo:
def add_logo(window, location):
    logo_image = tk.PhotoImage(file=location)
    logo = logo_image.subsample(x=30,y=30)
    logo_label = Label(window, image=logo)
    logo_label.pack()
    
# Function to run the robot node of the package:
def run_robot_node():
    # Define the robot command:
    ros_driver_commands = [
        "rosrun robothon_package robot.py",
    ]
    # Run the command in seperate terminal windows:
    for command in ros_driver_commands:
        subprocess.Popen(["gnome-terminal", "-e", command])

# Function to run the computer vision node of the package:
def run_vision_node():
    # Define the vision command:
    ros_driver_commands = [
        "rosrun robothon_package computer_vision.py",
    ]
    # Run the vision command in seperate terminal windows:
    for command in ros_driver_commands:
        subprocess.Popen(["gnome-terminal", "-e", command])

def close_terminal():
    os.system("pkill gnome-terminal")



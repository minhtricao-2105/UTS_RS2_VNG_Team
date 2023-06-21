import os,sys,rospy,datetime,subprocess
import tkinter as tk
import cv2 as cv
import pyautogui
import webbrowser

from tkinter import Label, ttk, Canvas
from cv_bridge import CvBridge
from ttkthemes import ThemedStyle
from sensor_msgs.msg import Image as SensorImage  # Importing the Image class from sensor_msgs.msg
from PIL import ImageTk, Image

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

def open_github_link():
    github_link = "https://github.com/minhtricao-2105/UTS_RS2_VNG_Team.git"
    webbrowser.open_new(github_link)

def open_youtube():
    webbrowser.open_new("https://www.youtube.com/watch?v=iCPoGm3XHzU&t=56s")
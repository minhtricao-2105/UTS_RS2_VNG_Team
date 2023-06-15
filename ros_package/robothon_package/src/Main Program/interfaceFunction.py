import tkinter as tk
import subprocess
from tkinter import Tk, Label, ttk, Canvas
from PIL import Image
import datetime
from ttkthemes import ThemedStyle

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

def update_clock(canvas, remaining_time=10):
   
    remaining_time -= 0.1

    # Calculate the percentage of work completed
    percentage = (10 - remaining_time) / 10 * 100

    # Clear the canvas
    canvas.delete("all")

    # Calculate the coordinates and radius of the circle
    x_center = canvas.winfo_width() // 2
    y_center = canvas.winfo_height() // 2
    radius = min(canvas.winfo_width(), canvas.winfo_height()) // 2 - 10

    # Calculate the start and end angles for the arc based on the percentage
    start_angle = 90  # Start at the top
    end_angle = start_angle - (percentage / 100) * 360  # Convert percentage to degrees

    # Draw the background circle
    canvas.create_oval(x_center - radius, y_center - radius, x_center + radius, y_center + radius, outline="#dcdcdc", width=10, fill="#476930")

    # Draw multiple smaller arcs to create a thicker arc with straight caps
    num_arcs = 10
    arc_width = 25
    arc_gap = 2
    arc_angle = (end_angle - start_angle) / num_arcs
    for i in range(num_arcs):
        arc_start = start_angle + i * arc_angle
        arc_end = arc_start + arc_angle - arc_gap
        canvas.create_arc(x_center - radius, y_center - radius, x_center + radius, y_center + radius,
                          start=arc_start, extent=arc_angle - arc_gap, outline="#86B049",
                          width=arc_width, style="arc")

    # Draw the text indicating the percentage
    text = f"{int(percentage)}%"
    canvas.create_text(x_center, y_center, text=text, font=("Calibri", 40, "bold"), fill="#C8B88A")

    # Check if the time has reached 0
    if remaining_time > 0:
        # Update the clock after 1 second
        canvas.after(1000, update_clock)
    else:
        # Perform actions when the time is up
        print("Time's up!")

# # Configure grid weights to allow label expansion
# window.grid_rowconfigure(0, weight=1)
# window.grid_columnconfigure(0, weight=1)

# Logo from here:
# logo_image = tk.PhotoImage(file="media/uts.png")
# logo = logo_image.subsample(x=15,y=15)
# logo_label = Label(window, image=logo)
# logo_label.place(x=20,y=5)
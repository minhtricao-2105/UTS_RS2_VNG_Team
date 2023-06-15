from interfaceFunction import*

# Create the Tkinter application window
window = tk.Tk()
window.title("Autonomous E-Waste Sorting System")
window.geometry("1400x800")

# Background here:
canvas = Canvas(window, width=1400, height=800)
background_photo = tk.PhotoImage(file="media/hello.png")
background = Label(window, image=background_photo)
background.pack()

# UnderBackground:
frame_1 = tk.Frame(window, bg="lightgrey")
frame_1.place(x=0, y=200, relwidth=1, relheight=1)

# LEFT FRAME:
inner_frame = tk.Frame(window, bg="white", highlightthickness=3, highlightbackground="black")

# Create the text on that frame:
decore_left = tk.Frame(window, bg="dark green",highlightthickness=3, highlightbackground="black")
text_left = tk.Label(decore_left, text="ROBOT CONTROL", font=("Calibri", 16, "bold"), foreground='white',bg="dark green")
text_left.place(relx=0.5, rely=0.5, anchor="center")

# MIDDLE FRAME:
middle_frame = tk.Frame(window, bg="white", highlightthickness=3, highlightbackground="black")


# Create a frame for grouping widgets
frame = tk.Frame(window, bg ="")
frame.place(relx=0.5, rely=0.15, anchor="center")

# Add labels using normal text
label1 = tk.Label(frame, text="AUTONOMOUS E-WASTE DETECTION AND SORTING SYSTEM", font=("Calibri", 22, "bold"), bd=2, relief="solid",foreground='green',highlightbackground="black")
label1.pack()

# Apply some formatting options to the clock label
label1.config(borderwidth=2, relief="solid", padx=10, pady=5)

# Pack the clock label to fill the frame
label1 .pack(fill="both", expand=True)


# Clock Frame:
# Create a canvas to draw the clock

frame_3 = tk.Frame(window, bg="white")
# Create a button to run the ROS drivers
# Create a frame for the button
button_frame = tk.Frame(window)
button_frame.place(relx=0.5, rely=0.5, anchor="center")

# Create a button with no background color
# button_1 = tk.Button(button_frame, text="Run the Robot Node", command=run_robot_node, highlightbackground=button_frame.cget("bg"), fg="blue", font=("Arial", 14))
# button_1.pack(side="left", padx=10, pady=10)

# # Create a button to run the ROS drivers
# button_2 = tk.Button(window, text="Run the Robot Node", command=run_vision_node, bg = "blue", fg = "white", font = ("Arial", 14))
# button_2.pack(side="right", padx=30, pady=10)

#Clock and date here:

# Create a frame for grouping widgets
def adjust_frame(event):
    frame_2.place(x=window.winfo_width() - 180, y = 30, relwidth=0.08, relheight=0.05)
    inner_frame.place(x=window.winfo_width()*0.0214, y=230, relwidth=0.24, relheight=0.45)
    decore_left.place(x=window.winfo_width()*0.0214, y=230, relwidth=0.24, relheight=0.08)
    middle_frame.place(x=window.winfo_width()*0.35714, y=230, relwidth=0.35, relheight=0.45)

# Function to update the clock and date
def update_time():
    current_time = datetime.datetime.now().strftime("%H:%M:%S")
    clock_label.config(text=current_time)
    window.after(1000, update_time)  # Update every 1 second (1000 milliseconds)

frame_2 = tk.Frame(window, bg="white")
# Bind the adjust_frame function to the window resize event
window.bind("<Configure>", adjust_frame)

# Pack the frame to fill the window horizontally
frame_2.pack(expand=True, fill="x")

clock_label = tk.Label(frame_2, font=("Arial", 15), fg="black")

# Pack the labels
clock_label.pack()

# Call the update_time function to start updating the clock and date
update_time()

# Apply some formatting options to the clock label
clock_label.config(borderwidth=2, relief="solid", padx=10, pady=5)

# Pack the clock label to fill the frame
clock_label.pack(fill="both", expand=True)

import tkinter as tk

def update_clock():
    global remaining_time
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

# Get the input time from the user
remaining_time = 10



canvas = tk.Canvas(frame_3, width=300, height=300)
canvas.pack()

# Start the clock update
update_clock()

# Start the Tkinter event loop
window.mainloop()


# Start the Tkinter event loop
window.mainloop()
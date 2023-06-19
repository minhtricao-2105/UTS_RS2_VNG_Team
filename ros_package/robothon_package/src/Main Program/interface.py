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
frame_1 = Canvas(window, bg="#DFEAE2")
frame_1.place(x=0, y=200, relwidth=1, relheight=1)

# LEFT FRAME:
inner_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2")

# Create the text on that frame:
decore_left = tk.Frame(window, bg="#358873", highlightbackground="#DFEAE2")
text_left = tk.Label(decore_left, text="ROBOT CONTROL", font=("Calibri", 16, "bold"), foreground='white',bg="#358873")
text_left.place(relx=0.5, rely=0.55, anchor="center")

# MIDDLE FRAME:
middle_frame = tk.Frame(window, bg="white", highlightthickness=3, highlightbackground="black")

# RIGHT FRAME:
right_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2", relief="groove")

# Create the text on that frame:
decore_right = tk.Frame(window, bg="#358873", highlightbackground="#DFEAE2")
text_right = tk.Label(decore_right, text="COMPUTER VISION", font=("Calibri", 16, "bold"), foreground='white',bg="#358873")
text_right.place(relx=0.5, rely=0.55, anchor="center")

# Create a frame for grouping widgets
frame = tk.Frame(window, bg ="", highlightthickness=0)
frame.place(relx=0.5, rely=0.15, anchor="center")

# Add labels using normal text
label1 = tk.Label(frame, text="AUTONOMOUS E-WASTE DETECTION AND SORTING SYSTEM", font=("Calibri", 22, "bold"), bd=2, relief="solid", foreground='green')
label1.config(bg="#B4D6C1", padx=10, pady=10, width=50)
label1.config(borderwidth=0, relief="raised")
label1.config(highlightthickness=0, highlightcolor="blue")

label1.pack()

# Apply some formatting options to the clock label
label1.config(borderwidth=2, relief="solid", padx=10, pady=5)

# Pack the clock label to fill the frame
label1 .pack(fill="both", expand=True)

# Clock Frame:
# Create a canvas to draw the clock
frame_3 = tk.Frame(window, bg="#B4D6C1")


# Create a button to run the ROS drivers
# Create a frame for the button
button_frame_1 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)
button_frame_2 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)
button_frame_3 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)
button_frame_4 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)

#Clock and date here:

# Create a frame for grouping widgets
def adjust_frame(event):
    frame_2.place(x=window.winfo_width() - 180, y = 30, relwidth=0.08, relheight=0.05)
    #Left Frame:
    inner_frame.place(x=window.winfo_width()*0.0214, y=230, relwidth=0.28, relheight=0.5)
    decore_left.place(x=window.winfo_width()*0.0214, y=230, relwidth=0.28, relheight=0.08)
    # #Middle Frame:
    middle_frame.place(x=window.winfo_width()*0.35714, y=230, relwidth=0.288, relheight=0.5)
    # Clock Frame:
    frame_3.place(x=window.winfo_width()*0.05,y=320, relwidth=0.22, relheight=0.27)
    #Right Frame:
    right_frame.place(x=window.winfo_width()*0.7,y=230, relwidth=0.28, relheight=0.5)
    decore_right.place(x=window.winfo_width()*0.7,y=230, relwidth=0.28, relheight=0.08)
    #Button Frame:
    button_frame_1.place(x=window.winfo_width()*0.095,y=530, relwidth=0.15, relheight=0.07)
    button_frame_2.place(x=window.winfo_width()*0.125,y=573, relwidth=0.15, relheight=0.06)
    button_frame_3.place(x=window.winfo_width()*0.786,y=540, relwidth=0.15, relheight=0.08)

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
                          start=arc_start, extent=arc_angle - arc_gap, outline="#207567",
                          width=arc_width, style="arc")

    # Draw the text indicating the percentage
    text = f"{int(percentage)}%"
    canvas.create_text(x_center, y_center, text=text, font=("Calibri", 40, "bold"), fill="#DFEAE2")

    # Check if the time has reached 0
    if remaining_time > 0:
        # Update the clock after 1 second
        canvas.after(1000, update_clock)
    else:
        # Perform actions when the time is up
        print("Time's up!")

def reset_clock():
    global remaining_time
    remaining_time = 0.1
    button_1.config(state="disable")
    button_3.config(state="normal")

# Define a function to disable the button after it is clicked
def disable_button():
    global remaining_time
    remaining_time = 10
    button_1.config(state="disabled")

def vision_button():
    button_1.config(state="normal")
    button_3.config(state="disabled")

canvas = tk.Canvas(frame_3, width=300, height=300,bg="#B4D6C1", borderwidth=0, highlightthickness=0)
canvas.pack()

# Button Declaration here:
button_1 = tk.Button(button_frame_1, text="START MISSION", command=lambda: [disable_button(), update_clock(),run_robot_node()], bg="#207567", fg="#DFEAE2",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5, state="disabled")

button_2 = tk.Button(button_frame_2, text="Reset", command=lambda: [reset_clock(), close_terminal()], bg="#DD2C00", fg="white", font=("Calibri", 12, "bold"), borderwidth=2, relief="raised", padx=10, pady=10)

button_3 = tk.Button(button_frame_3, text="RUN VISION", command=lambda: [vision_button(), run_vision_node()], bg="#207567", fg="#DFEAE2",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5)

# Pack the button
button_1.pack(side="left", padx=10, pady=10)
button_2.pack(side="left", padx=10, pady=10)
button_3.pack(side="left", padx=10, pady=10)

# Start the Tkinter event loop
window.mainloop()
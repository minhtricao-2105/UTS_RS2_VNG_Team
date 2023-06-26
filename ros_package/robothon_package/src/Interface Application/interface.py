from interfaceFunction import*

### Belong to ROS here ###
time_len = 0

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    pil_image = Image.fromarray(cv.cvtColor(cv_image, cv.COLOR_BGR2RGB))
    resized_image = pil_image.resize((int(window.winfo_width()*0.2857),  int(window.winfo_height()*0.3125)))
    tk_image = ImageTk.PhotoImage(resized_image)
    # Create the image label widget
    image_label.configure(image=tk_image)
    image_label.image = tk_image  # Keep a reference to avoid garbage collection

def clear_image():
    image_label.configure(image=None)
    image_label.image = None
    image_capture.configure(image=None)
    image_capture.image = None

def screenshot_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    pil_image = Image.fromarray(cv.cvtColor(cv_image, cv.COLOR_BGR2RGB))
    resized_image = pil_image.resize((int(window.winfo_width()*0.7857), int(window.winfo_height()*0.375)))
    tk_image = ImageTk.PhotoImage(resized_image)
    # Create the image label widget
    image_capture.configure(image=tk_image)
    image_capture.image = tk_image  # Keep a reference to avoid garbage collection

def time_callback(msg):
    global time_len
    time_len = msg.data

rospy.init_node("interface_node")
sub_1 = rospy.Subscriber('Image_Vision', SensorImage, image_callback)
sub_2 = rospy.Subscriber("screenshot", SensorImage, screenshot_callback)
sub_3 = rospy.Subscriber('Calculate_time',Int32, time_callback)
### Belong to ROS here ###

# Create the Tkinter application window
window = tk.Tk()
window.title("Autonomous E-Waste Sorting System")
window.geometry("1400x800")

### 1. Background here:
background = tk.Frame(window, bg = "#4E9C81")
text_frame = tk.Frame(window, bg = "#4E9C81")
text_mid = tk.Label(text_frame, text="AUTONOMOUS DETECTING AND SORTING E-WASTE SYSTEM", font=("Calibri", 24, "bold"), foreground='white',bg="#4E9C81")
text_mid.place(relx=0.5, rely=0.55, anchor="center")

### 2. Menu frame:
menu_frame = tk.Frame(window, bg = "#207567")

#### 2.1 Button Menu Frame:
button_menu_1_frame = Canvas(window, bg="#207567", borderwidth=0, highlightthickness=0)
button_menu_2_frame = Canvas(window, bg="#207567", borderwidth=0, highlightthickness=0)
button_menu_3_frame = Canvas(window, bg="#207567", borderwidth=0, highlightthickness=0)

# UnderBackground:
frame_1 = tk.Frame(window, bg="#DFEAE2")


### 3. CLOCK FRAME:
frame_2 = tk.Frame(window, bg="#207567")

### 4. LEFT FRAME:
inner_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2")
decore_left = tk.Frame(window, bg="#207567", highlightbackground="#207567")
text_left = tk.Label(decore_left, text="ROBOT CONTROL", font=("Calibri", 16, "bold"), foreground='white',bg="#207567")
text_left.place(relx=0.5, rely=0.55, anchor="center")

### 5. MIDDLE FRAME:
middle_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2")
decore_mid = tk.Frame(window, bg="#207567", highlightbackground="#DFEAE2")
text_mid = tk.Label(decore_mid, text="ROBOT SIMULATION", font=("Calibri", 16, "bold"), foreground='white',bg="#207567")
text_mid.place(relx=0.5, rely=0.55, anchor="center")

### 6. RIGHT FRAME:
right_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2", relief="groove")
decore_right = tk.Frame(window, bg="#207567", highlightbackground="#DFEAE2")
text_right = tk.Label(decore_right, text="COMPUTER VISION", font=("Calibri", 16, "bold"), foreground='white',bg="#207567")
text_right.place(relx=0.5, rely=0.55, anchor="center")

### 7. CLOCK FRAME:
frame_3 = tk.Frame(window, bg="#B4D6C1")

### 8. Image Frame to display figure from Vision Node:
figure_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2", highlightthickness=0,highlightcolor="#DFEAE2")
image_label = tk.Label(figure_frame, bg="#B4D6C1")
image_label.pack()

### 9. CAPTURE FRAME:
capture_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2")
image_capture = tk.Label(capture_frame, bg="#B4D6C1", highlightbackground="#DFEAE2")
image_capture.pack()

### 10. BUTTON FRAME:
button_frame_1 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)
button_frame_2 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)
button_frame_3 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)
button_frame_4 = Canvas(window, bg="#B4D6C1", borderwidth=0, highlightthickness=0)

### 11. LOGO FRAME:
logo_frame = tk.Frame(window, bg="#B4D6C1", highlightbackground="#DFEAE2")

# Create a frame for grouping widgets
def adjust_frame(event):
    #Background:
    background.place(x=0, y=0, relwidth=1, relheight=1)
    text_frame.place(x=0, y=0, relwidth=1, relheight=0.25)
    #Menu Frame:
    menu_frame.place(x=window.winfo_width()*0.4714, y = window.winfo_height()*0.185, relwidth=0.5, relheight=0.045)
    button_menu_1_frame.place(x=window.winfo_width()*0.475, y = window.winfo_height()*0.185, relwidth=0.1, relheight=0.045)
    button_menu_2_frame.place(x=window.winfo_width()*0.575, y = window.winfo_height()*0.185, relwidth=0.1, relheight=0.045)
    button_menu_3_frame.place(x=window.winfo_width()*0.675, y = window.winfo_height()*0.185, relwidth=0.1, relheight=0.045)
    #Logo Frame:
    logo_frame.place(x=window.winfo_width()*0.1, y = window.winfo_height()*0.875, relwidth=0.1, relheight=0.1)
    
    frame_1.place(x=0, y=window.winfo_height()*0.25, relwidth=1, relheight=1)
    frame_2.place(x=window.winfo_width()* 0.825, y = window.winfo_height()*0.0275, relwidth=0.15, relheight=0.078)
    #Left Frame:
    inner_frame.place(x=window.winfo_width()*0.0214, y=window.winfo_height()*0.2875, relwidth=0.28, relheight=0.5)
    decore_left.place(x=window.winfo_width()*0.0214, y=window.winfo_height()*0.2875, relwidth=0.28, relheight=0.08)
    # #Middle Frame:
    middle_frame.place(x=window.winfo_width()*0.315, y=window.winfo_height()*0.2875, relwidth=0.372, relheight=0.5)
    decore_mid.place(x=window.winfo_width()*0.315, y=window.winfo_height()*0.2875, relwidth=0.372, relheight=0.08)
    # Clock Frame:
    frame_3.place(x=window.winfo_width()*0.05,y=window.winfo_height()*0.4, relwidth=0.22, relheight=0.27)
    #Right Frame:
    right_frame.place(x=window.winfo_width()*0.7,y=window.winfo_height()*0.2875, relwidth=0.28, relheight=0.5)
    decore_right.place(x=window.winfo_width()*0.7,y=window.winfo_height()*0.2875, relwidth=0.28, relheight=0.08)
    #Button Frame:
    button_frame_1.place(x=window.winfo_width()*0.09,y=window.winfo_height()*0.6625, relwidth=0.15, relheight=0.07)
    button_frame_2.place(x=window.winfo_width()*0.09,y=window.winfo_height()*0.7225, relwidth=0.15, relheight=0.05)
    button_frame_3.place(x=window.winfo_width()*0.765,y=window.winfo_height()*0.675, relwidth=0.15, relheight=0.07)
    #Figure Frame:
    figure_frame.place(x=window.winfo_width()*0.7,y=window.winfo_height()*0.3875, relwidth=0.28, relheight=0.275)
    capture_frame.place(x=window.winfo_width()*0.315, y=window.winfo_height()*0.3875, relwidth=0.372, relheight=0.4)

# Function to update the clock and date
def update_time():
    current_time = datetime.datetime.now().strftime("%H : %M : %S")
    clock_label.config(text=current_time)
    window.after(1000, update_time)  # Update every 1 second (1000 milliseconds)


# Bind the adjust_frame function to the window resize event
window.bind("<Configure>", adjust_frame)

# Pack the frame to fill the window horizontally
frame_2.pack(expand=True, fill="x")

clock_label = tk.Label(frame_2, font=("CALIBRI", 20, "bold"), fg="white", bg="#207567")

# Pack the labels
clock_label.place(relx=0.5, rely=0.5, anchor="center")

# Call the update_time function to start updating the clock and date
update_time()

# Apply some formatting options to the clock label
clock_label.config(borderwidth=0, relief="solid", padx=10, pady=5)


def update_clock():

    global remaining_time
    remaining_time -= 1

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
    global time_len
    remaining_time = 10
    button_1.config(state="disabled")

def vision_button():
    button_1.config(state="normal")
    button_3.config(state="disabled")

# Function to capture and display the browser screen
def capture_screen():
    # Capture the screen
    screenshot = pyautogui.screenshot()

    # Convert the screenshot to a format compatible with Tkinter
    image_screen = ImageTk.PhotoImage(screenshot)

    # Display the image in a Tkinter label
    image_capture.configure(image=image_screen)
    image_capture.image = image_screen

canvas = tk.Canvas(frame_3, width=300, height=300,bg="#B4D6C1", borderwidth=0, highlightthickness=0)
canvas.pack()

# Button Declaration here:
button_1 = tk.Button(button_frame_1, text="START MISSION", command=lambda: [disable_button(), update_clock(),run_robot_node()], bg="#207567", fg="#DFEAE2",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5, state="disabled")

button_2 = tk.Button(button_frame_2, text="RESET", command=lambda: [reset_clock(), close_terminal(), clear_image()], bg="#DD2C00", fg="white", font=("Calibri", 12, "bold"), borderwidth=2, relief="raised", padx=10, pady=5)

button_3 = tk.Button(button_frame_3, text="RUN VISION", command=lambda: [vision_button(), run_vision_node()], bg="#207567", fg="#DFEAE2",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5)

# Buttton of the menu
button_menu_1 = tk.Button(button_menu_1_frame, text="Github", command=open_github_link, bg="#207567", fg="white",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5, highlightthickness=0)
button_menu_2 = tk.Button(button_menu_2_frame, text="Youtube", command=open_youtube, bg="#207567", fg="white",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5, highlightthickness=0)
button_menu_3 = tk.Button(button_menu_3_frame, text="Close", command=lambda: [close_terminal(), window.quit()], bg="#207567", fg="white",
                     font=("Calibri", 12, "bold"), borderwidth=0, relief="raised", padx=10, pady=5, highlightthickness=0)
# Pack the button
button_1.place(relx=0.5, rely=0.5, anchor="center")
button_2.place(relx=0.5, rely=0.5, anchor="center")
button_3.place(relx=0.5, rely=0.5, anchor="center")
button_menu_1.place(relx=0.5, rely=0.5, anchor="center")
button_menu_2.place(relx=0.5, rely=0.5, anchor="center")
button_menu_3.place(relx=0.5, rely=0.5, anchor="center")
# Start the Tkinter event loop
window.mainloop()

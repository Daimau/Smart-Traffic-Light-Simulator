import cv2
import numpy as np
from gpiozero import LED, DigitalInputDevice
import time
from gpiozero.pins.pigpio import PiGPIOFactory
import os
import sys
import tkinter as tk
from tkinter import messagebox
import _tkinter

print("LOADING YOLO")
# Read the custom tiny-yolo-v4 weights and configuration files created for this system
net = cv2.dnn.readNet("yolov4-tiny-custom_best.weights", "yolov4-tiny-custom.cfg")
# save all the names in file o the list classes
classes = []
with open("obj.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
    print(classes)
# get layers of the network
layer_names = net.getLayerNames()
# Determine the output layer names from the YOLO model
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
print("YOLO LOADED")

# Create a PiGPIOFactory object with the specified IP address
ip_address = '192.168.100.21'
factory = PiGPIOFactory(host=ip_address)

# Create LED objects connected to GPIO pins using the PiGPIOFactory
red_led = LED(16, pin_factory=factory)
yellow_led = LED(21, pin_factory=factory)
green_led = LED(26, pin_factory=factory)

# Create a DigitalInputDevice connected to a GPIO pin using the PiGPIOFactory
infrared_sensor = DigitalInputDevice(18, pull_up=False, pin_factory=factory)

# Traffic light timings
red_time = 10
yellow_time = 3
base_green_time = 5
green_time = 0

video_capture = cv2.VideoCapture(1)
video_capture.set(3, 1280)
video_capture.set(4, 720)

# Create a folder to save traffic violations if it doesnt exist
save_folder = 'traffic_violations'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)


def login_gui():
    def validate_credentials():
        username = username_var.get()
        password = password_var.get()

        # The correct username and password stored in the system
        correct_username = "officer"
        correct_password = "1234"

        if username == correct_username and password == correct_password:
            login_window.destroy()
            main_menu_gui()
        else:
            messagebox.showerror("Error", "Invalid username or password.")

    def exit_button():
        login_window.destroy()
        red_led.off()
        yellow_led.off()
        green_led.off()
        video_capture.release()
        print("Exiting...")
        sys.exit()

    # The GUI code and button binding to their functionalities
    login_window = tk.Tk()
    login_window.title("Traffic Light Control System")

    # Create username and password entry boxes labels
    username_label = tk.Label(login_window, text="Username:", font=("Helvetica", 12))
    username_label.grid(row=0, column=0, padx=(20, 10), pady=(20, 10), sticky="w")

    password_label = tk.Label(login_window, text="Password:", font=("Helvetica", 12))
    password_label.grid(row=1, column=0, padx=(20, 10), pady=(0, 10), sticky="w")

    # automatically store the username and password in entry boxes
    username_var = tk.StringVar()
    password_var = tk.StringVar()

    # username and password entry boxes
    username_entry = tk.Entry(login_window, textvariable=username_var)
    username_entry.grid(row=0, column=1, padx=(10, 20), pady=(20, 10), sticky="ew")

    password_entry = tk.Entry(login_window, textvariable=password_var, show="*")
    password_entry.grid(row=1, column=1, padx=(10, 20), pady=(0, 10), sticky="ew")

    # GUI code for buttons
    login_button = tk.Button(login_window, text="Login", command=validate_credentials, font=("Helvetica", 12),
                             bg="#4CAF50", fg="white")
    login_button.grid(row=2, column=0, columnspan=2, padx=20, pady=(10, 20), sticky="ew")

    exit_button = tk.Button(login_window, text="Exit", command=exit_button, font=("Helvetica", 12),
                            bg="#607D8B", fg="white")
    exit_button.grid(row=3, column=0, columnspan=2, padx=20, pady=(0, 20), sticky="ew")

    login_window.mainloop()


def main_menu_gui():
    def automated_button():
        window.destroy()
        automated_system()

    def semi_automated_button():
        window.destroy()
        semi_automated_system()

    def manual_button():
        window.destroy()
        manual_system()

    def exit_button():
        window.destroy()
        red_led.off()
        yellow_led.off()
        green_led.off()
        video_capture.release()
        print("Exiting...")
        sys.exit()

    # The GUI code and button binding to their functionalities
    window = tk.Tk()
    window.title("Traffic Light Control System")

    window.configure(bg="#F0F0F0")

    # Create "Select a Mode" label
    title_label = tk.Label(window, text="Select a Mode", font=("Helvetica", 16), bg="#F0F0F0")
    title_label.grid(row=0, column=0, columnspan=2, padx=20, pady=(20, 10))

    # Create buttons
    automated_button = tk.Button(window, text="Turn the Automated System on", command=automated_button,
                                 font=("Helvetica", 12), bg="#4CAF50", fg="white")
    automated_button.grid(row=1, column=0, padx=(20, 10), pady=10, sticky="ew")

    semi_automated_button = tk.Button(window, text="Set to Semi-Automated", command=semi_automated_button,
                                      font=("Helvetica", 12), bg="#FFC107")
    semi_automated_button.grid(row=1, column=1, padx=(10, 20), pady=10, sticky="ew")

    manual_button = tk.Button(window, text="Control Manually", command=manual_button,
                              font=("Helvetica", 12), bg="#2196F3", fg="white")
    manual_button.grid(row=2, column=0, padx=(20, 10), pady=10, sticky="ew")

    exit_button = tk.Button(window, text="Exit", command=exit_button,
                            font=("Helvetica", 12), bg="#607D8B", fg="white")
    exit_button.grid(row=2, column=1, padx=(10, 20), pady=10, sticky="ew")

    window.grid_columnconfigure(0, weight=1)
    window.grid_columnconfigure(1, weight=1)

    window.mainloop()


def automated_system():
    # Initialize the start time for the red light
    red_start_time = time.time()
    # Initialize the start time for the green light when ambulance is detected
    green_start_time = None
    # Set the traffic light state to red
    traffic_light_state = 'RED'
    # Variable to keep track of whether the ambulance is in view
    ambulance_in_view = False
    # Variable to keep track of the traffic light state before the ambulance was detected
    previous_traffic_light_state = None
    # Variable to keep track of the waiting start time
    waiting_start_time = None
    # Variable to keep track of the waiting time
    waiting_time = 5
    # Variable to keep track of the waiting yellow time
    waiting_yellow_time = 2

    while True:

        # Read the state of the infrared sensor
        infrared_sensor_state = infrared_sensor.value

        # Capture frame-by-frame
        re, img = video_capture.read()
        img = cv2.resize(img, None, fx=0.4, fy=0.4)
        height, width, channels = img.shape

        # If the infrared sensor is active and the traffic light is red, take a picture
        if infrared_sensor_state == 0 and traffic_light_state == 'RED':
            pic_name = f'{save_folder}/img_{time.strftime("%Y%m%d_%H%M%S")}.jpg'
            cv2.imwrite(pic_name, img)

        # USing blob function of opencv to preprocess image
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)
        # Detecting objects
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Showing information on the screen
        class_ids = []
        confidences = []
        boxes = []
        car_count = 0  # initialize car count to 0
        ambulance_detected = False
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.30:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

                    # Check if the detected object is an ambulance
                    if classes[class_id] == 'Ambulance':
                        ambulance_detected = True

        # Update the ambulance_in_view variable
        if ambulance_detected:
            ambulance_in_view = True
        else:
            ambulance_in_view = False

        # We use NMS function in opencv to perform Non-maximum Suppression
        # we give it score threshold and nms threshold as arguments.
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        colors = np.random.uniform(0, 255, size=(len(classes), 3))
        for i in range(len(boxes)):
            if i in indexes:
                car_count += 1  # increment car count for each car detected
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 2, color, 3)

        # Initialize base time back to its value of 5 for next cycle
        base_green_time = 5

        # Calculate the green time for the next cycle
        next_green_time = base_green_time + 2 * car_count

        # Calculate elapsed time for the red light2
        red_elapsed_time = time.time() - red_start_time

        # Control traffic light
        if ambulance_detected:  # Check if ambulance_detected is True
            previous_traffic_light_state = traffic_light_state

            # Set the traffic light state to green until the ambulance is out of view
            green_led.on()
            yellow_led.off()
            red_led.off()
            traffic_light_state = 'GREEN'
            text = "Green light: Ambulance in view"

        # If ambulance is in view and traffic light is green
        elif ambulance_in_view and traffic_light_state == 'GREEN':
            text = "Green light: Ambulance in view"

        # checks if there was a previous state
        elif previous_traffic_light_state:
            # checks if the waiting time variable is not zero or none if yes record a time
            if not waiting_start_time:
                waiting_start_time = time.time()
            waiting_elapsed_time = time.time() - waiting_start_time

            # If waiting time for green has not elapsed, sets waiting time green state
            if waiting_elapsed_time < waiting_time:
                green_led.on()
                yellow_led.off()
                red_led.off()
                traffic_light_state = 'WAITING'
                text = f"Waiting: {waiting_time - waiting_elapsed_time:.1f}s"
            #  If waiting time for yellow light has not elapsed, set it to waiting yellow state
            elif waiting_elapsed_time < waiting_time + waiting_yellow_time:
                green_led.off()
                yellow_led.on()
                red_led.off()
                traffic_light_state = 'WAITING_YELLOW'
                text = f"Waiting yellow: {waiting_time + waiting_yellow_time - waiting_elapsed_time:.1f}s"
            else:
                previous_traffic_light_state = None
                waiting_start_time = None
                # Reset the start time for the red light
                red_start_time = time.time()
                traffic_light_state = 'RED'
        # Continue with routine traffic light operations
        else:
            # Set the traffic light state to red for specified duration
            if red_elapsed_time < red_time:
                red_led.on()
                yellow_led.off()
                green_led.off()
                green_time = next_green_time
                text = f"Red light: {red_time - red_elapsed_time:.1f}s"
                traffic_light_state = 'RED'
            # After red light time finished
            else:
                red_led.off()

                # Calculate elapsed time for the green and yellow lights
                green_elapsed_time = time.time() - red_start_time - red_time
                yellow_elapsed_time = green_elapsed_time - green_time

                # Set traffic light to green for duration calculated
                if green_elapsed_time < green_time:
                    green_led.on()
                    yellow_led.off()
                    text = f"Green light: {green_time - green_elapsed_time:.1f}s"
                    traffic_light_state = 'GREEN'
                # Set traffic light to yellow light for specified duration
                elif yellow_elapsed_time < yellow_time:
                    yellow_led.on()
                    green_led.off()
                    text = f"Yellow light: {yellow_time - yellow_elapsed_time:.1f}s"
                    traffic_light_state = 'YELLOW'
                # Reset to red light after green and yellow are over for next cycle
                else:
                    # Reset the start time for the red light
                    red_start_time = time.time()
                    # Update the green time for the next cycle
                    green_time = next_green_time
                    text = f"Red light: {red_time:.1f}s"
                    traffic_light_state = 'RED'

        # Add traffic light timings and state to the image
        cv2.putText(img, text, (10, 50), font, 1.25, (0, 0, 255), 2)
        cv2.putText(img, f"Green time: {green_time}s", (10, 150), font, 1.25, (0, 255, 0), 2)
        cv2.putText(img, "Car count: " + str(car_count), (10, 100), font, 1.25, (0, 255, 0), 2)
        cv2.putText(img, f"Status: {traffic_light_state}", (10, 200), font, 1.25, (0, 0, 255), 2)
        cv2.imshow("Image", cv2.resize(img, (800, 600)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    red_led.on()
    yellow_led.off()
    green_led.off()
    main_menu_gui()


def semi_automated_system():
    def clear_entry(event, entry):
        entry.delete(0, 'end')

    def set_timings():
        try:
            red_time = red_var.get()
            yellow_time = yellow_var.get()
            green_time = green_var.get()

            if red_time > 0 and yellow_time > 0 and green_time > 0:
                traffic_light_control(red_time, yellow_time, green_time)
            else:
                messagebox.showerror("Error", "Please enter valid timings (greater than 0).")
        except _tkinter.TclError:
            messagebox.showerror("Error", "Please enter only numbers.")

    def exit_button():
        root.destroy()
        main_menu_gui()

    def traffic_light_control(red_time, yellow_time, green_time):
        start_time = time.time()
        current_state = 'red'
        font = cv2.FONT_HERSHEY_PLAIN

        while True:
            elapsed_time = time.time() - start_time

            if current_state == 'red':
                red_led.on()
                yellow_led.off()
                green_led.off()

                if elapsed_time >= red_time:
                    start_time = time.time()
                    current_state = 'green'

            elif current_state == 'green':
                red_led.off()
                yellow_led.off()
                green_led.on()

                if elapsed_time >= green_time:
                    start_time = time.time()
                    current_state = 'yellow'

            elif current_state == 'yellow':
                red_led.off()
                yellow_led.on()
                green_led.off()

                if elapsed_time >= yellow_time:
                    start_time = time.time()
                    current_state = 'red'

            # Read video capture frames and return the frame and a boolean value
            ret, img = video_capture.read()

            # Add traffic light timings and state to the image
            text = f"{current_state.capitalize()} light: {red_time - elapsed_time:.1f}s" if current_state == 'red' else f"{current_state.capitalize()} light: {yellow_time - elapsed_time:.1f}s" if current_state == 'yellow' else f"{current_state.capitalize()} light: {green_time - elapsed_time:.1f}s"
            cv2.putText(img, text, (10, 50), font, 3, (0, 0, 255), 3)
            cv2.putText(img, f"Status: {current_state.capitalize()}", (10, 100), font, 3, (0, 0, 255), 3)

            img = cv2.resize(img, None, fx=0.4, fy=0.4)
            cv2.imshow("Image", cv2.resize(img, (800, 600)))

            # Capture traffic light violations
            infrared_sensor_state = infrared_sensor.value
            if current_state == 'red' and infrared_sensor_state == 0:
                pic_name = f'{save_folder}/img_{time.strftime("%Y%m%d_%H%M%S")}.jpg'
                cv2.imwrite(pic_name, img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                red_led.on()
                yellow_led.off()
                green_led.off()
                break

    # The GUI code and button binding to their functionalities
    root = tk.Tk()
    root.title("Semi-Automated Mode")

    # Entry Boxes labels
    red_label = tk.Label(root, text="Red light duration (s):", font=("Helvetica", 12))
    red_label.grid(row=0, column=0, padx=(20, 10), pady=(20, 10), sticky="w")

    yellow_label = tk.Label(root, text="Yellow light duration (s):", font=("Helvetica", 12))
    yellow_label.grid(row=1, column=0, padx=(20, 10), pady=(0, 10), sticky="w")

    green_label = tk.Label(root, text="Green light duration (s):", font=("Helvetica", 12))
    green_label.grid(row=2, column=0, padx=(20, 10), pady=(0, 10), sticky="w")

    # Create entry boxes for timings
    red_var = tk.DoubleVar()
    yellow_var = tk.DoubleVar()
    green_var = tk.DoubleVar()

    red_entry = tk.Entry(root, textvariable=red_var)
    red_entry.grid(row=0, column=1, padx=(10, 20), pady=(20, 10), sticky="ew")
    red_entry.bind('<FocusIn>', lambda event: clear_entry(event, red_entry))

    yellow_entry = tk.Entry(root, textvariable=yellow_var)
    yellow_entry.grid(row=1, column=1, padx=(10, 20), pady=(0, 10), sticky="ew")
    yellow_entry.bind('<FocusIn>', lambda event: clear_entry(event, yellow_entry))

    green_entry = tk.Entry(root, textvariable=green_var)
    green_entry.grid(row=2, column=1, padx=(10, 20), pady=(0, 10), sticky="ew")
    green_entry.bind('<FocusIn>', lambda event: clear_entry(event, green_entry))

    # Creating buttons
    start_button = tk.Button(root, text="Start", command=set_timings,
                             font=("Helvetica", 12), bg="#4CAF50", fg="white")
    start_button.grid(row=3, column=0, padx=(20, 10), pady=(10, 20), sticky="ew")

    exit_button = tk.Button(root, text="Exit", command=exit_button,
                            font=("Helvetica", 12), bg="#607D8B", fg="white")
    exit_button.grid(row=3, column=1, padx=(10, 20), pady=(10, 20), sticky="ew")

    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)

    root.mainloop()


def manual_system():
    def turn_red():
        red_led.on()
        yellow_led.off()
        green_led.off()

    def turn_yellow():
        red_led.off()
        yellow_led.on()
        green_led.off()

    def turn_green():
        red_led.off()
        yellow_led.off()
        green_led.on()

    def turn_off():
        red_led.off()
        yellow_led.off()
        green_led.off()

    def exit_button():
        manual_window.destroy()
        main_menu_gui()

    # The GUI code and button binding to their functionalities
    manual_window = tk.Tk()
    manual_window.title("Manual Mode")

    manual_window.configure(bg="#F0F0F0")

    # GUI code for buttons
    red_button = tk.Button(manual_window, text="Turn Red", command=turn_red, font=("Helvetica", 12), bg="#F44336",
                           fg="white")
    red_button.grid(row=0, column=0, padx=(20, 10), pady=(20, 10), sticky="ew")

    yellow_button = tk.Button(manual_window, text="Turn Yellow", command=turn_yellow, font=("Helvetica", 12),
                              bg="#FFC107")
    yellow_button.grid(row=0, column=1, padx=(10, 20), pady=(20, 10), sticky="ew")

    green_button = tk.Button(manual_window, text="Turn Green", command=turn_green, font=("Helvetica", 12), bg="#4CAF50",
                             fg="white")
    green_button.grid(row=1, column=0, padx=(20, 10), pady=(10, 20), sticky="ew")

    off_button = tk.Button(manual_window, text="Turn Off", command=turn_off, font=("Helvetica", 12), bg="#9E9E9E",
                           fg="white")
    off_button.grid(row=1, column=1, padx=(10, 20), pady=(10, 20), sticky="ew")

    exit_button = tk.Button(manual_window, text="Exit", command=exit_button, font=("Helvetica", 12), bg="#607D8B",
                            fg="white")
    exit_button.grid(row=2, column=0, columnspan=2, padx=20, pady=(10, 20), sticky="ew")

    manual_window.grid_columnconfigure(0, weight=1)
    manual_window.grid_columnconfigure(1, weight=1)

    manual_window.mainloop()


# Main function
def main():
    login_gui()


if __name__ == "__main__":
    main()

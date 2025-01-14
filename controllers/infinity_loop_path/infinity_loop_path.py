from controller import Robot, GPS, InertialUnit, Gyro, Motor
import math
import numpy as np
import matplotlib.pyplot as plt

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
imu = robot.getDevice("inertial unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

motors = [
    robot.getDevice("front left propeller"),
    robot.getDevice("front right propeller"),
    robot.getDevice("rear left propeller"),
    robot.getDevice("rear right propeller"),
]
for motor in motors:
    motor.setPosition(float('inf'))  # Velocity control
    motor.setVelocity(0.0)

# Constants for PID
k_vertical_thrust = 67.0
k_vertical_offset = 0.6
k_vertical_p = 4.0
k_roll_p, k_roll_d = 30.0, 10.0
k_pitch_p, k_pitch_d = 30.0, 10.0
k_yaw_p = 1.0  # Yaw control
k_position_p = 0.3  # Fine-tuned proportional gain
k_position_d = 0.1  # Fine-tuned derivative gain

# Variables
target_altitude = 3.0
altitude_tolerance = 0.1
current_state = "Grounded"
hovering = False
waypoint_index = 0
rover_path = []  # For storing the trajectory of the drone

# Generate two hexagons
def generate_two_hexagons():
    hexagon1 = []
    hexagon2 = []
    center1 = [-40, 0]
    center2 = [40, 0]
    radius = 10.0
    for i in range(6):
        angle = 2 * np.pi * i / 6
        hexagon1.append([center1[0] + radius * np.cos(angle), center1[1] + radius * np.sin(angle)])
        hexagon2.append([center2[0] + radius * np.cos(angle), center2[1] + radius * np.sin(angle)])
    return hexagon1 + hexagon2 + [center1]  # Return to the starting point

two_hexagons_path = generate_two_hexagons()

# Helper Functions
def clamp(value, low, high):
    return max(low, min(value, high))

def stabilize_drone():
    global waypoint_index, rover_path

    roll, pitch, yaw = imu.getRollPitchYaw()
    gps_values = gps.getValues()
    x_position = gps_values[0]
    y_position = gps_values[1]
    altitude = gps_values[2]
    roll_velocity, pitch_velocity, yaw_velocity = gyro.getValues()

    # Altitude stabilization
    altitude_error = target_altitude - altitude + k_vertical_offset
    vertical_input = k_vertical_p * clamp(altitude_error, -1.0, 1.0)

    # Initialize position and yaw control inputs
    x_input, y_input, yaw_input = 0.0, 0.0, 0.0

    if current_state == "Path-Following":
        # Get current waypoint
        waypoint = two_hexagons_path[waypoint_index]
        x_error = waypoint[0] - x_position
        y_error = waypoint[1] - y_position
        distance_to_waypoint = math.hypot(x_error, y_error)

        # Proportional-Derivative control for position
        x_input = k_position_p * x_error + k_position_d * (x_error / timestep)
        y_input = k_position_p * y_error + k_position_d * (y_error / timestep)

        # Clamp inputs
        x_input = clamp(x_input, -1.0, 1.0)
        y_input = clamp(y_input, -1.0, 1.0)

        # Adjust yaw
        desired_yaw = math.atan2(y_error, x_error)
        yaw_error = desired_yaw - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
        yaw_input = clamp(k_yaw_p * yaw_error, -1.0, 1.0)

        # Check waypoint tolerance
        if distance_to_waypoint < 1.5:  # Increased tolerance
            waypoint_index += 1
            if waypoint_index >= len(two_hexagons_path):
                print("Path following completed.")
                plot_combined_path(two_hexagons_path, rover_path)
                robot.simulationQuit(0)
                exit()

        # Record path
        rover_path.append([x_position, y_position])

        # Debug
        print(f"Waypoint: {waypoint}, Distance to waypoint: {distance_to_waypoint:.2f}")
        print(f"x_input: {x_input:.2f}, y_input: {y_input:.2f}, yaw_input: {yaw_input:.2f}")

    # Roll, pitch, and yaw stabilization
    roll_input = k_roll_p * roll + k_roll_d * roll_velocity + x_input
    pitch_input = k_pitch_p * pitch + k_pitch_d * pitch_velocity - y_input

    # Motor control
    front_left_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    front_right_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    rear_left_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    rear_right_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

    motor_inputs = [
        clamp(front_left_input, -100.0, 100.0),
        clamp(-front_right_input, -100.0, 100.0),
        clamp(-rear_left_input, -100.0, 100.0),
        clamp(rear_right_input, -100.0, 100.0),
    ]
    for motor, value in zip(motors, motor_inputs):
        motor.setVelocity(value)

def plot_combined_path(reference_path, drone_trajectory):
    ref_x, ref_y = zip(*reference_path)
    drone_x, drone_y = zip(*drone_trajectory)

    plt.figure(figsize=(10, 6))
    plt.plot(ref_x, ref_y, 'b-', label="Reference Path (Two Hexagons)")
    plt.plot(drone_x, drone_y, 'r--', label="Drone Tracked Path")
    plt.title("Path Following Simulation")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid()
    plt.show()

# Main Loop
while robot.step(timestep) != -1:
    gps_altitude = gps.getValues()[2]
    print(f"Current Altitude: {gps_altitude:.2f} m, State: {current_state}")

    if current_state == "Grounded":
        if gps_altitude < target_altitude - 0.5:
            current_state = "Take-Off"
            print("Transitioned to Take-Off state.")

    elif current_state == "Take-Off":
        if abs(gps_altitude - target_altitude) <= altitude_tolerance:
            current_state = "Hover"
            print("Transitioned to Hover state.")
            waypoint_index = 0  # Reset waypoint index for path following

    elif current_state == "Hover":
        print("Hovering stabilized. Starting path following.")
        current_state = "Path-Following"

    # Stabilize and control drone
    stabilize_drone()

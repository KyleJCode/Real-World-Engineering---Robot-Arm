# Author: Carson Baker
# Date Last Modified: 1/28/2025
# Description: Calculates joint angles for built in target position (single)
# Input: DH parameters, target position/rotation, joint bounds position/rotation
# Output: Joint angles to reach target position if converges (to cpp)
# Version changes: no simulations, communication to cpp-motor code

# Imports and such
import serial
import time
from numpy import *

# Open serial communication with Arduino on COM5
try:
    arduino = serial.Serial('COM5', 9600, timeout=1)
    print("Serial connection established.")
    time.sleep(2)  # Give time for Arduino to initialize
except serial.SerialException as e:
    print(f"Error: {e}")
    exit()

# Function to generate DH transformation matrix using NumPy
# Input: DH parameters in order d, theta, a, alpha
# Output: Corresponding transformation matrix (4x4)
def DH_trans_matrix(d, theta, a, alpha):
    return array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward Kinematics
# Input: Joint angles, DH paramaters, Degrees of Freedom
# Output: The position (x,y,z) of each joint relative to origin
def forward_kinematics(joints, DH_params, DOF):
    positions = [array([0, 0, 0])]  # Start with the origin
    trans = eye(4)
    for i in range(DOF):
        d, a, alpha = DH_params[i]
        theta = joints[i]
        trans = trans @ DH_trans_matrix(d, theta, a, alpha)
        positions.append(trans[:3, 3])  # Store the x, y, z position of each joint
    return array(positions)

# Inverse Kinematics
# Input: Initial joint angles, target position, DH parameters, joint rotation limits, joint position limits, max iterations, allowable tolerance
# Output: Solved joint angles, joint positions (x,y,z), end position error, if converged or not
def inverse_kinematics(joints_init, target, DH_params, joint_limits, joint_position_limits, DOF, iterations=100, tolerance=0.01):
    joints = joints_init.copy()
    target_pos = target[:3, 3]
    converged = False
    prev_error = float('inf')

    for i in range(iterations):
        current_pos = forward_kinematics(joints, DH_params, DOF)[-1]  # End-effector position
        position_error = target_pos - current_pos
        error_norm = linalg.norm(position_error)

        if error_norm < tolerance:
            converged = True
            print(f"Converged in {i + 1} iterations with error: {error_norm:.6f}")
            break

        if error_norm >= prev_error:
            print("Warning: Potential divergence or oscillation detected.")
            break
        prev_error = error_norm

        jacobian = zeros((3, DOF))
        delta = 1e-6
        for j in range(DOF):
            joints_delta = joints.copy()
            joints_delta[j] += delta
            pos_delta = forward_kinematics(joints_delta, DH_params, DOF)[-1]
            jacobian[:, j] = (pos_delta - current_pos) / delta

        joint_update = linalg.pinv(jacobian).dot(position_error)
        joints += joint_update

        for j in range(DOF):
            joints[j] = clip(joints[j], joint_limits[j][0], joint_limits[j][1])

        joint_positions = forward_kinematics(joints, DH_params, DOF)
        for j in range(DOF):
            joint_positions[j, 2] = max(joint_positions[j, 2], joint_position_limits[j][2][0])

    final_position = forward_kinematics(joints, DH_params, DOF)[-1]
    final_error = linalg.norm(target_pos - final_position)

    if not converged:
        print(f"Failed to converge within {iterations} iterations. Final error: {final_error:.6f}")
        raise ValueError("Inverse kinematics did not converge.")

    return joints, forward_kinematics(joints, DH_params, DOF), final_error, converged

# Convert Euler angles to rotational matrix
# Input: Euler angles (pitch, yaw, roll)
# Output: Coresponding rotational matrix 
def euler_to_rotation_matrix(pitch, yaw, roll):
    Rx = array([
        [1, 0, 0],
        [0, cos(pitch), -sin(pitch)],
        [0, sin(pitch), cos(pitch)]
    ])
    Ry = array([
        [cos(yaw), 0, sin(yaw)],
        [0, 1, 0],
        [-sin(yaw), 0, cos(yaw)]
    ])
    Rz = array([
        [cos(roll), -sin(roll), 0],
        [sin(roll), cos(roll), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

# Function to send a float to Arduino and get the result
# Input: Joint angles (decimals)
# Output: Send joint angles to Arduino/C++ via serial terminal
def send_floats_to_arduino(joint_angles):
    try:
        if not joint_angles:
            print("Error: The list of joint angles is empty")
            return
        
        # Convert the list of floats to a comma-seperated string
        data_to_send = ",".join(map(str, joint_angles)) + "/n"
        arduino.write(data_to_send.encode())
        print(f"Sent to Arduino: {data_to_send.strip()}")

        time.sleep(2)
        # Wait for the response from Arduino
        response = arduino.readline().decode().strip()


        if response:
            print(f"Response from Arduino: {response}")
        else:
            print("No response received from Arduino.")
    except Exception as e:
        print(f"Error during communication: {e}")

# Main function of code, always ran and calls other functions
def main():
    # Define DH parameters
    # d parameter
    d1, d2, d3, d4, d5, d6 = 119.770, 0, 0, 222.63, 0, 41.0
    # a parameter
    a1, a2, a3, a4, a5, a6 = 64.2, 305, 0, 0, 0, 0
    # alpha parameter
    alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = (
        radians(-90), radians(0), radians(90), radians(-90), radians(90), radians(0)
    )

    # DH parameter matrix
    DH_params = [
        [d1, a1, alpha1],
        [d2, a2, alpha2],
        [d3, a3, alpha3],
        [d4, a4, alpha4],
        [d5, a5, alpha5],
        [d6, a6, alpha6]
    ]

    # Joint limits matrix (radians)
    joint_limits = [
        [-pi, pi],
        [-pi / 2, pi / 2],
        [-pi, pi],
        [-pi, pi],
        [-3 * pi / 4, 3 * pi / 4],
        [-pi, pi]
    ]

    # Joint position limits (ensure z pos > 0 so no ground collision)
    joint_position_limits = [
        [(-inf, inf), (-inf, inf), (0, inf)],
        [(-inf, inf), (-inf, inf), (0, inf)],
        [(-inf, inf), (-inf, inf), (0, inf)],
        [(-inf, inf), (-inf, inf), (0, inf)],
        [(-inf, inf), (-inf, inf), (0, inf)],
        [(-inf, inf), (-inf, inf), (0, inf)]
    ]

    DOF = 6  # Degrees of freedom

    # Step 1: Define target positions (x, y, z)
    x_tar, y_tar, z_tar = 180.0, 60.0, 150.0  # Define the target positions here
    print(f"Target positions: x={x_tar}, y={y_tar}, z={z_tar}")

    # Step 2: Convert to rotation matrix (assuming no rotation for simplicity)
    pitch, yaw, roll = radians(0), radians(0), radians(0)
    rotation_tar = euler_to_rotation_matrix(pitch, yaw, roll)

    target = array([
        [rotation_tar[0, 0], rotation_tar[0, 1], rotation_tar[0, 2], x_tar],
        [rotation_tar[1, 0], rotation_tar[1, 1], rotation_tar[1, 2], y_tar],
        [rotation_tar[2, 0], rotation_tar[2, 1], rotation_tar[2, 2], z_tar],
        [0, 0, 0, 1]
    ])

    # Initial joint position/guess
    joints_init = array([0, -pi / 2, 0, pi / 2, 0, 0])

    # Step 3: Solve IK
    try:
        joints_solution_rad, joint_positions, error_distance, converged = inverse_kinematics(
            joints_init, target, DH_params, joint_limits, joint_position_limits, DOF
        )
        print("Joint angles calculated:")
        joints_solution_deg = joints_solution_rad * 180/pi
        print(joints_solution_deg)

        # Step 4: Send joint angles to Arduino
        joint_angles = list(joints_solution_deg)
        send_floats_to_arduino(joint_angles[:2])

    except ValueError as e:
        print(f"Error in IK solution: {e}")

    # Close the serial connection
    arduino.close()
    print("Serial connection closed.")


if __name__ == "__main__":
    main()

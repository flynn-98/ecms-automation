import numpy as np
import json
from spatialmath import SE3
from robot import Robot  # Import Robot class from robot.py

class RobotCalibration:
    def __init__(self, robot: Robot):
        """
        Initializes the RobotCalibration class.
        :param robot: An instance of the Robot class
        """
        self.robot = robot  # Robot instance
        self.data = []  # List to store collected TCP position data
    
    def collect_data(self, x: float, y: float):
        """
        Collects the TCP position data from the robot and stores it with the corresponding world point.
        :param x: X coordinate of the world position (user input)
        :param y: Y coordinate of the world position (user input)
        """
        ee_pose = self.robot.T_base_tcp  # Get End-Effector pose in base frame
        world_position = [x, y, 0.0]  # Fix Z-coordinate to 0

        sample = {
            "robot_position": ee_pose.A[:3, 3].tolist(),  # Extract the translation vector from SE3
            "world_position": world_position  # Store given world position
        }
        self.data.append(sample)

    def save_data(self, filename="tcp_calibration_data.json"):
        """
        Saves collected TCP and world position data to a JSON file.
        This function is expected to be called 15 times.
        :param filename: Name of the file to save the data
        """
        with open(filename, "w") as f:
            json.dump(self.data, f, indent=4)
        print(f"Data saved to {filename} ({len(self.data)} entries)")

    def load_data(self, filename="tcp_calibration_data.json"):
        """
        Loads TCP and world position data from a JSON file.
        :param filename: Name of the file to load the data from
        """
        with open(filename, "r") as f:
            self.data = json.load(f)
        print(f"Data loaded from {filename}")

    def compute_calibration(self, points_robot, points_world):
        """
        Computes the optimal rotation (R) and translation (T) that aligns TCP coordinates with the world frame.
        :param points_robot: List of TCP positions collected from the robot
        :param points_world: Corresponding world coordinates
        :return: Rotation matrix (R), translation vector (T)
        """
        points_robot = np.array(points_robot)
        points_world = np.array(points_world)

        centroid_robot = np.mean(points_robot, axis=0)
        centroid_world = np.mean(points_world, axis=0)

        q_robot = points_robot - centroid_robot
        q_world = points_world - centroid_world

        H = np.zeros((3, 3))
        for qr, qw in zip(q_robot, q_world):
            H += np.outer(qr, qw)

        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Ensure the rotation matrix has a determinant of 1 (to prevent reflection)
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        T = centroid_world - R @ centroid_robot
        return R, T

    def calibrate(self):
        """
        Performs TCP calibration using the collected data.
        :return: Rotation matrix R and translation vector T
        """
        if len(self.data) < 15:
            print(f"Not enough data collected ({len(self.data)}/15). Calibration requires 15 samples.")
            return None, None

        points_robot = [sample["robot_position"] for sample in self.data]
        points_world = [sample["world_position"] for sample in self.data]

        R, T = self.compute_calibration(points_robot, points_world)
        
        print("Calibration Completed:")
        print("Rotation Matrix (R):\n", R)
        print("Translation Vector (T):\n", T)

        return R, T

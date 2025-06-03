import json

import customtkinter as ctk
import numpy as np

from config import T_W_BASE
from robot import Robot
from utils import make_tf, transform_w_to_base


def load_target_positions(filename):
    """
    Load target positions from a JSON file.

    Parameters
    ----------
    filename : str
        Path to the JSON file containing target positions.

    Returns
    ----------
    list
        List of 3D positions loaded from the file.
    """
    try:
        with open(filename, "r") as f:
            positions = json.load(f)
        return positions
    except Exception as e:
        print(f"Error loading target positions: {e}")
        return []


def move_robot(target_pos_w):
    """
    Function to calculate the target position and move the robot.

    Parameters
    ----------
    target_pos_w : np.ndarray
        A 3-element list representing the target position in the world frame.
    """
    try:
        robot = Robot("192.168.0.12", False)
        Tcp_T = make_tf(pos=robot.T_base_tcp.t, ori=robot.T_base_tcp.R)
        T_W_Tcp = T_W_BASE @ Tcp_T
        T_W_Target = make_tf(target_pos_w, T_W_Tcp.R)
        # Transform target position from world to base
        T_BASE_Target = transform_w_to_base(T_W_Tcp, T_W_Target, T_W_BASE)

        # Create target transformation matrix
        robot.moveL(T_BASE_Target)
        print(f"Robot moveL executed to position: {target_pos_w}!")
    except Exception as e:
        print("An error occurred:", e)


def main():
    """
    Main UI function to create buttons that trigger the moveL event with different positions.
    """
    # Load target positions from an external file
    target_positions = load_target_positions("target_positions.json")

    if not target_positions:
        print("No target positions loaded. Exiting.")
        return

    # Initialize the UI window
    app = ctk.CTk()
    app.title("Robot Controller")
    app.geometry("400x300")

    # Create buttons for each position
    for i, pos in enumerate(target_positions):
        button = ctk.CTkButton(
            app,
            text=f"Move to Position {i + 1}",
            command=lambda p=pos: move_robot(
                np.array(p)
            ),  # Pass the position as a parameter
        )
        button.pack(pady=10)

    # Run the UI loop
    app.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")

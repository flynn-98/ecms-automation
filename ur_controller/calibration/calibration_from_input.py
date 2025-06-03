import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np

# Table Robot Calibration

# Function to compute calibration
def compute_calibration(points_robot, points_world):
    centroid_robot = np.mean(points_robot, axis=0)
    centroid_world = np.mean(points_world, axis=0)
    q_robot = points_robot - centroid_robot
    q_world = points_world - centroid_world
    H = np.zeros((3, 3))
    for qr, qw in zip(q_robot, q_world):
        H += np.outer(qr, qw)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    T = centroid_world - R @ centroid_robot
    return R, T

# Function to plot points
def plot_points(points_robot, points_world, transformed_robot):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(points_robot[:, 0], points_robot[:, 1], points_robot[:, 2], color='red', label='Robot Base Points')
    ax.scatter(points_world[:, 0], points_world[:, 1], points_world[:, 2], color='blue', label='World Points')
    ax.scatter(transformed_robot[:, 0], transformed_robot[:, 1], transformed_robot[:, 2], color='green', marker='^', label='Transformed Robot Points')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title('Robot-World Calibration Visualization')
    plt.show()

# Create the UI
class CalibrationApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Robot-World Calibration")
        self.geometry("600x500")

        # Input fields for points_robot
        self.robot_label = ctk.CTkLabel(self, text="Points Robot (comma-separated):")
        self.robot_label.pack(pady=10)
        self.robot_entry = ctk.CTkEntry(self, width=500)
        self.robot_entry.pack(pady=5)

        # Input fields for points_world
        self.world_label = ctk.CTkLabel(self, text="Points World (comma-separated):")
        self.world_label.pack(pady=10)
        self.world_entry = ctk.CTkEntry(self, width=500)
        self.world_entry.pack(pady=5)

        # Button to compute calibration
        self.compute_button = ctk.CTkButton(self, text="Compute Calibration", command=self.compute_calibration)
        self.compute_button.pack(pady=20)

        # Output fields
        self.output_label = ctk.CTkLabel(self, text="Output:")
        self.output_label.pack(pady=10)
        self.output_text = ctk.CTkTextbox(self, width=500, height=150)
        self.output_text.pack(pady=5)

    def parse_points(self, text):
        try:
            return np.array([list(map(float, point.split())) for point in text.split(',')])
        except ValueError:
            self.output_text.insert("end", "\nError parsing points. Please check the format.")
            return None

    def compute_calibration(self):
        self.output_text.delete("1.0", "end")

        points_robot = self.parse_points(self.robot_entry.get())
        points_world = self.parse_points(self.world_entry.get())

        if points_robot is None or points_world is None:
            return

        if points_robot.shape != points_world.shape:
            self.output_text.insert("end", "\nError: Points sets must have the same shape.")
            return

        # Compute R and T
        R, T = compute_calibration(points_robot, points_world)

        # Create the homogeneous transformation matrix
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = T

        # Transform robot points
        transformed_robot = (R @ points_robot.T).T + T

        # Display results
        self.output_text.insert("end", f"Rotation Matrix (R):\n{R}\n\n")
        self.output_text.insert("end", f"Translation Vector (T):\n{T}\n\n")
        self.output_text.insert("end", f"Homogeneous Transformation Matrix (H):\n{H}\n")

        # Plot points
        plot_points(points_robot, points_world, transformed_robot)

# Run the app
if __name__ == "__main__":
    app = CalibrationApp()
    app.mainloop()

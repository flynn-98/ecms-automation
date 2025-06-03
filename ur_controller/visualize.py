import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R


def visualize_tfs(transform_list, labels=None):
    """
    Visualize multiple homogeneous transformation matrices.

    :param transform_list: List of 4x4 homogeneous transformation matrices
    :param labels: Optional list of labels corresponding to each transformation
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Visualization parameters
    axis_length = 0.2  # Length of the axes for visualization

    # Iterate through the list of transformations
    for i, transform in enumerate(transform_list):
        label = labels[i] if labels is not None else f"Transformation {i + 1}"

        # Extract origin and rotation matrix
        origin = transform[:3, 3]
        rotation = transform[:3, :3]

        # Compute the axes
        x_axis = origin + rotation @ [axis_length, 0, 0]
        y_axis = origin + rotation @ [0, axis_length, 0]
        z_axis = origin + rotation @ [0, 0, axis_length]

        # Plot the origin
        ax.scatter(*origin, label=label)

        # Plot the axes
        ax.quiver(*origin, *(x_axis - origin), color="r", label=f"{label} X-axis")
        ax.quiver(*origin, *(y_axis - origin), color="g", label=f"{label} Y-axis")
        ax.quiver(*origin, *(z_axis - origin), color="b", label=f"{label} Z-axis")

    # Set labels and limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.legend()
    plt.title("Visualization of Multiple Transformations")
    plt.show()


# Example usage
if __name__ == "__main__":
    # Define example transformations
    T1 = np.eye(4)
    T1[:3, 3] = [0.5, 0.2, 0.1]

    T2 = np.eye(4)
    T2[:3, 3] = [-0.3, -0.4, 0.2]
    T2[:3, :3] = R.from_euler("xyz", [30, 45, 60], degrees=True).as_matrix()

    T3 = np.eye(4)
    T3[:3, 3] = [0.1, -0.5, 0.3]
    T3[:3, :3] = R.from_euler("xyz", [-45, 0, 90], degrees=True).as_matrix()

    # Visualize the transformations
    visualize_tfs([T1, T2, T3], labels=["T1", "T2", "T3"])

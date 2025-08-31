import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def create_cuboid(center, x_size, y_size, z_size):
    """Create vertices for a cuboid with the given dimensions and center."""
    half_x = x_size / 2
    half_y = y_size / 2
    half_z = z_size / 2
    return np.array([
        [center[0] - half_x, center[1] - half_y, center[2] - half_z],
        [center[0] + half_x, center[1] - half_y, center[2] - half_z],
        [center[0] + half_x, center[1] + half_y, center[2] - half_z],
        [center[0] - half_x, center[1] + half_y, center[2] - half_z],
        [center[0] - half_x, center[1] - half_y, center[2] + half_z],
        [center[0] + half_x, center[1] - half_y, center[2] + half_z],
        [center[0] + half_x, center[1] + half_y, center[2] + half_z],
        [center[0] - half_x, center[1] + half_y, center[2] + half_z]
    ])

def draw_cuboid(ax, vertices):
    """Draw the cuboid given its vertices."""
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom square
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top square
        [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
    ]
    for edge in edges:
        points = vertices[edge, :]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'k')

def draw_drone(ax):
    """Draw a drone-like cuboid with arms extending from each face."""
    # Central body
    central_body_vertices = create_cuboid([0, 0, 0], 2, 2, 2)

    # Draw the central body
    draw_cuboid(ax, central_body_vertices)
    
    # Arms extending from each face of the central body
    arm_length = 3
    arm_width = 0.5
    
    arm_offsets = [
        [-1.5, -1, 0],   # Top face
        [2, 0, 0],  # Bottom face

        [-1.5, 1, 0],  # Bottom face

    ]
    
    for offset in arm_offsets:
        arm_center = np.array(offset) * (arm_length / 2)
        arm_vertices = create_cuboid(arm_center, arm_length, arm_width, arm_width)
        draw_cuboid(ax, arm_vertices)

def get_drone(ax):
    central_body_vertices = create_cuboid([0, 0, 0], 2, 2, 2)

    # Draw the central body
    draw_cuboid(ax, central_body_vertices)
    
    # Arms extending from each face of the central body
    arm_length = 8
    arm_width = 2
    
    arm_offsets = [
        [-1.5, -1, 0],   # Top face
      #  [6, 0, 0],  # Bottom face

        [-1.5, 1, 0],  # Bottom face

    ]
    arm_vertices_list = []
    for offset in arm_offsets:
        arm_center = np.array(offset) * (arm_length / 2)
        arm_vertices = create_cuboid(arm_center, arm_length, arm_width, arm_width)
        arm_vertices_list.append(arm_vertices)
    return arm_vertices_list

def main():
    # Plotting the drone
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    draw_drone(ax)

    # Set the limits and labels
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

if __name__ == "__main__":
    main()
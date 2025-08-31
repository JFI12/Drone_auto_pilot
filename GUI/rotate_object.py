import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotation_matrix_x(angle):
    """Create a rotation matrix for a rotation around the x-axis."""
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rotation_matrix_y(angle):
    """Create a rotation matrix for a rotation around the y-axis."""
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rotation_matrix_z(angle):
    """Create a rotation matrix for a rotation around the z-axis."""
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

def rotate_cube(cube, angles):
    """Rotate the cube by the specified angles."""
    Rx = rotation_matrix_x(angles[0])
    Ry = rotation_matrix_y(angles[1])
    Rz = rotation_matrix_z(angles[2])
    
    rotation_matrix = Rz @ Ry @ Rx  # Combined rotation matrix
    return cube @ rotation_matrix.T

def create_cuboid(x_size, y_size, z_size):
    """Create vertices for a cuboid with the given dimensions."""
    return np.array([
        [-x_size / 2, -y_size / 2, -z_size / 2],
        [ x_size / 2, -y_size / 2, -z_size / 2],
        [ x_size / 2,  y_size / 2, -z_size / 2],
        [-x_size / 2,  y_size / 2, -z_size / 2],
        [-x_size / 2, -y_size / 2,  z_size / 2],
        [ x_size / 2, -y_size / 2,  z_size / 2],
        [ x_size / 2,  y_size / 2,  z_size / 2],
        [-x_size / 2,  y_size / 2,  z_size / 2]
    ])

def draw_cube(ax, vertices):
    """Draw the edges of the cube given its vertices."""
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom square
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top square
        [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
    ]
    
    for edge in edges:
        points = vertices[edge, :]
        ax.plot(points[:, 0], points[:, 1], points[:, 2], 'k')
        
cuboid_vertices = create_cuboid(10, 4, 1)

    
var1 = 10
def main():
    # Define the vertices of a unit cube centered at the origin



    # Input rotation angles in radians for x, y, and z axes
    x_angle = float(input("Enter rotation angle around x-axis (in radians): "))
    y_angle = float(input("Enter rotation angle around y-axis (in radians): "))
    z_angle = float(input("Enter rotation angle around z-axis (in radians): "))

    # Rotate the cube
    rotated_vertices = rotate_cube(cuboid_vertices, [x_angle, y_angle, z_angle])

    # Plot the cube
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    draw_cube(ax, rotated_vertices)

    # Set the limits of the plot
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

if __name__ == "__main__":
    main()
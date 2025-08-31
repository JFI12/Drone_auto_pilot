import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Rodrigues' rotation formula implementation
def rotation_matrix(k, theta):
    k = k / np.linalg.norm(k)  # Ensure k is a unit vector
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    I = np.eye(3)
    R = I + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
    print(K*np.sin(theta))
    print(np.dot(K, K))
    return R

# Apply rotation to vector
def rotate_vector(vector, axis, theta):

    R = rotation_matrix(axis, theta)
    print(R)
    return np.dot(R, vector)

# Example vector and axis
vector = np.array([0,1, 1])
axis = np.array([0, 1, 1])  # Arbitrary axis, you can change this
theta = np.radians(90)  # Rotation angle in radians

# Rotate the vector
rotated_vector = rotate_vector(vector, axis, theta)

# Display results
print(f"Original Vector: {vector}")
print(f"Rotated Vector: {rotated_vector}")

# Plot the original and rotated vectors
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot original vector
ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='r', label='Original Vector')

# Plot rotated vector
ax.quiver(0, 0, 0, rotated_vector[0], rotated_vector[1], rotated_vector[2], color='b', label='Rotated Vector')

# Plot rotation axis
ax.quiver(0, 0, 0, axis[0], axis[1], axis[2], color='g', label='Rotation Axis')

# Set labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-3, 3])
ax.legend()

plt.show()
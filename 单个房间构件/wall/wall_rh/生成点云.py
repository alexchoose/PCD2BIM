import numpy as np


def generate_uniform_point_cloud(x_range, y_range, z_range, num_points):
    """
    Generate a uniform 3D point cloud within specified x, y, z ranges.

    Parameters:
    x_range (tuple): The (min, max) range for the x coordinates.
    y_range (tuple): The (min, max) range for the y coordinates.
    z_range (tuple): The (min, max) range for the z coordinates.
    num_points (int): The total number of points to generate.

    Returns:
    np.ndarray: An array of shape (num_points, 3) containing the generated 3D points.
    """
    # Calculate the number of points along each dimension
    num_points_per_dim = int(np.ceil(num_points ** (1 / 3)))

    # Generate evenly spaced points along each dimension
    x = np.linspace(x_range[0], x_range[1], num_points_per_dim)
    y = np.linspace(y_range[0], y_range[1], num_points_per_dim)
    z = np.linspace(z_range[0], z_range[1], num_points_per_dim)

    # Create a 3D grid of points
    X, Y, Z = np.meshgrid(x, y, z)

    # Flatten the grid to create a point cloud
    points = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

    # If there are too many points, randomly sample to get the exact number needed
    if points.shape[0] > num_points:
        indices = np.random.choice(points.shape[0], num_points, replace=False)
        points = points[indices]

    return points


# Example usage
x_range = (5, 10)
y_range = (1, 5)
z_range = (1, 3)
num_points = 1000

point_cloud = generate_uniform_point_cloud(x_range, y_range, z_range, num_points)
np.savetxt('1.txt', point_cloud)

from matplotlib.pyplot import draw
from mpl_toolkits.mplot3d import Axes3D
from numpy import ndarray, array


def plot_transformation(transformation: ndarray, axis: Axes3D):

    # Define the origin for ease of use
    origin = transformation[0:3, 3]

    # Plot the origin
    axis.scatter3D(origin[0], origin[1], origin[2],
                   s=35, color='black')

    # For each axis of the origin
    for i, color in zip(range(3), ['red', 'green', 'blue']):

        # Define the vector that describes the axis
        axis_vector = array([origin, origin + transformation[0:3, i] * .001])

        # Plot the vector
        axis.plot(axis_vector[:, 0],
                  axis_vector[:, 1],
                  axis_vector[:, 2],
                  color=color)

        draw()

# Import standard packages
from matplotlib.pyplot import draw
from mpl_toolkits.mplot3d import Axes3D
from numpy import array


def plot_triangles(triangles_to_plot: list, progress_plot: Axes3D, line_color: str) -> None:
    """
    Interactively plots triangles on a 3D graph as 3 individual lines.

    Parameters
    ----------
    triangles_to_plot :
        A list of triangles to add to the plot given as a list of tuples. A single triangle is given as a tuple of the
        three vertices. Each vertex of the triangle is given as a tuple.
    progress_plot :
        The axes object on which to plot the triangles.
    line_color :
        The line color to use to plot the triangles.
    """

    # Define a list of lines to plot
    lines_to_plot = []

    # For each triangle in the list of triangles to plot
    for triangle in triangles_to_plot:

        # Create a list of indices to iterate through
        indices = [0, 1, 2, 0]

        # For each vertex in the triangle
        for i in range(3):

            # Define variables to store the two vertexes making up each side of the triangle
            vertex_a = triangle[indices[i]]
            vertex_b = triangle[indices[i + 1]]

            # If the line between these two vertices is not already in the list of lines to plot,
            # Add it to the list of lines to plot
            if not (vertex_a, vertex_b) in lines_to_plot or not (vertex_b, vertex_a) in lines_to_plot:
                lines_to_plot.append((vertex_a, vertex_b))

    # For each line to plot
    for line in lines_to_plot:
        # Create an array from the line
        line_array = array(line)

        # Plot the line
        progress_plot.plot(line_array[:, 0],
                           line_array[:, 1],
                           line_array[:, 2],
                           color=line_color)

        # Update the plot
        draw()

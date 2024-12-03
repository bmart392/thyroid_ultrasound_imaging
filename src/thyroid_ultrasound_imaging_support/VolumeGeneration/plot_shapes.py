# Import standard packages
from matplotlib.pyplot import draw
from mpl_toolkits.mplot3d import Axes3D
from numpy import array


def plot_shapes(shapes_to_plot: list, progress_plot: Axes3D, line_color: str) -> None:
    """
    Interactively plots shapes on a 3D graph as individual lines.

    Parameters
    ----------
    shapes_to_plot :
        A list of shapes to add to the plot given as a list of tuples. A single shape is given as a tuple of the
        vertices contained within the shape. Each vertex of the shape is given as a tuple. Shapes must have at least
        two vertices.
    progress_plot :
        The axes object on which to plot the triangles.
    line_color :
        The line color to use to plot the triangles.
    """

    # Define a list of lines to plot
    lines_to_plot = []

    # For each shape in the list of shapes to plot
    for shape in shapes_to_plot:

        # Calculate the number of vertices in the shape
        num_vertices = len(shape)

        # Create a range from the number of vertices
        vertex_range = range(num_vertices)

        # Create a list of indices to iterate through
        indices = list(vertex_range)

        # If the shape has more than 2 vertices, ensure that the closing line is drawn
        if num_vertices > 2:
            indices = indices + [0]

        elif num_vertices == 2:
            vertex_range = range(1)
            indices = [0, 1]

        # Otherwise, ensure that at least two vertices are given
        elif num_vertices < 2:
            raise Exception('Shape must have at least 2 vertices.')

        # For each vertex in the shape
        for i in vertex_range:

            # Define variables to store the two vertexes making up each side of the shape
            vertex_a = shape[indices[i]]
            vertex_b = shape[indices[i + 1]]

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

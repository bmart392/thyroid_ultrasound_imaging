"""
File containing the InteractiveProgressPlot3D class definition.
"""

# Import standard python packages
from matplotlib.pyplot import figure, ion, draw
from numpy import array, ndarray

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.CrossContourLine import CrossContourLine

# Define color constants
POINT_COLOR: str = 'black'  # Point colors used to color the points on each contour
EXTERNAL_POINT_COLOR: str = 'red'  # Point color used to color the external points on each contour
CONTOUR_LINE: str = 'black'  # Line color used to color the lines that exist on contours
END_FACE_LINE_COLOR: str = 'purple'
COMMON_CLOSEST_POINT_COLOR: str = 'green'
QUADRILATERAL_CLOSING_LINE_COLOR: str = 'orange'
GAP_FILLING_LINE_COLOR: str = 'blue'

# Define point size constants
POINT_SIZE: int = 10
EXTERNAL_POINT_SIZE: int = 35


class InteractiveProgressPlot3D:

    def __init__(self, create_empty_object: bool = False):
        if create_empty_object:
            self.axis_object = None
        else:
            ion()
            fig = figure()
            self.axis_object = fig.add_subplot(projection='3d')
            self.axis_object.set_xlabel('X (m)')
            self.axis_object.set_ylabel('Y (m)')
            self.axis_object.set_zlabel('Z (m)')
            self.axis_object.set_proj_type('ortho')

    def plot_transformation(self, transformation: ndarray):
        if self.axis_object is not None:
            # Define the origin for ease of use
            origin = transformation[0:3, 3]

            # Plot the origin
            self.axis_object.scatter3D(origin[0], origin[1], origin[2],
                                       s=35, color='black')

            # For each axis of the origin
            for i, color in zip(range(3), ['red', 'green', 'blue']):
                # Define the vector that describes the axis
                axis_vector = array([origin, origin + transformation[0:3, i] * .001])

                # Plot the vector
                self.axis_object.plot(axis_vector[:, 0],
                                      axis_vector[:, 1],
                                      axis_vector[:, 2],
                                      color=color)

                draw()

    def plot_internal_and_external_points(self, internal_points: list = None, external_points: list = None):
        if self.axis_object is not None:
            if internal_points is not None and len(internal_points) > 0:
                internal_points_array = array(internal_points)
                self.axis_object.scatter3D(array(internal_points_array)[:, 0],
                                           array(internal_points_array)[:, 1],
                                           array(internal_points_array)[:, 2],
                                           c='black', s=10)

            if external_points is not None and len(external_points) > 0:
                external_points_array = array(external_points)
                self.axis_object.scatter3D(array(external_points_array)[:, 0],
                                           array(external_points_array)[:, 1],
                                           array(external_points_array)[:, 2],
                                           c='red', s=25)

    def plot_centroids(self, transformed_centroids: list):
        if self.axis_object is not None:
            self.axis_object.scatter3D(array(transformed_centroids)[0, :, 0],
                                       array(transformed_centroids)[0, :, 1],
                                       array(transformed_centroids)[0, :, 2],
                                       c='black', s=75
                                       )

    def plot_contour_points(self, contour_points: list, point_color: str = POINT_COLOR,
                            external_point_indices: list = None):
        # Plot the first slice if necessary
        if self.axis_object is not None:
            if external_point_indices is not None:
                color_to_use = [POINT_COLOR] * len(contour_points)
                size_to_use = [POINT_SIZE] * len(contour_points)
                for index in external_point_indices:
                    color_to_use[index] = EXTERNAL_POINT_COLOR
                    size_to_use[index] = EXTERNAL_POINT_SIZE
            else:
                color_to_use = point_color
                size_to_use = POINT_SIZE
            array_contour = array(contour_points)
            self.axis_object.scatter(array_contour[:, 0],
                                     array_contour[:, 1],
                                     array_contour[:, 2],
                                     color=color_to_use,
                                     s=size_to_use)

    def plot_single_line(self, line: CrossContourLine, lines_that_have_been_plotted: list):
        if self.axis_object is not None:
            # If the line has not already been plotted,
            if line not in lines_that_have_been_plotted:

                # Add the new common pair to the list of shapes that has already been plotted
                lines_that_have_been_plotted.append(line)

                # Plot the line
                if self.axis_object is not None:
                    self.plot_shapes(line.to_shape(), line.color)

    def plot_shapes(self, shapes_to_plot: list, line_color: str) -> None:
        """
        Interactively plots shapes on a 3D graph as individual lines.

        Parameters
        ----------
        shapes_to_plot :
            A list of shapes to add to the plot given as a list of tuples. A single shape is given as a tuple of the
            vertices contained within the shape. Each vertex of the shape is given as a tuple. Shapes must have at least
            two vertices.
        line_color :
            The line color to use to plot the triangles.
        """
        if self.axis_object is not None:

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
                self.axis_object.plot(line_array[:, 0],
                                      line_array[:, 1],
                                      line_array[:, 2],
                                      color=line_color)

                # Update the plot
                draw()


if __name__ == '__main__':
    # Create an instance of the object for testing
    test_object = InteractiveProgressPlot3D()

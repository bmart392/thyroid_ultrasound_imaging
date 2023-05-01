"""
Contains create_convex_triangles_from_points function.
"""
# Import standard packages
from panda3d.core import Triangulator


def create_convex_triangles_from_points(list_of_points: list):
    """
    Creates a set of convex, non-overlapping triangles, from a set of points forming a polygon.
    Each triangle is returned as a list of three (x, y) coordinates.

    Parameters
    ----------
    list_of_points
        a list of (x, y) coordinates defining a closed polygon.
    """
    # Create a triangulator object
    triangulator = Triangulator()

    # Iterate through each point in the list
    for point in list_of_points:
        # Add each point to the pool of vertices and then to the current polygon
        triangulator.addPolygonVertex(triangulator.addVertex(x=point[0], y=point[1]))

    # Create the triangles of the polygons
    triangulator.triangulate(None)

    # Define list to store polygons
    list_of_triangles = []

    # Add the indices for each triangle to the list of triangle indices
    for i in range(triangulator.getNumTriangles()):
        list_of_triangles.append([triangulator.getTriangleV0(i),
                                  triangulator.getTriangleV1(i),
                                  triangulator.getTriangleV2(i)])
        for j in range(len(list_of_triangles[-1])):
            vertex = triangulator.getVertex(list_of_triangles[-1][j])
            list_of_triangles[-1][j] = [int(vertex.x), int(vertex.y)]

    # Replace the triangle indices with the points themselves
    return list_of_triangles

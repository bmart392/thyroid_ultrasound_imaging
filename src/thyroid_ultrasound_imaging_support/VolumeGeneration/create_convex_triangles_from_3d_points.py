# Import standard packages
from panda3d.core import Triangulator3


# noinspection PyArgumentList
def create_convex_triangles_from_3d_points(list_of_points: list) -> list:
    """
    Creates a set of convex, non-overlapping triangles, from a set of points forming a polygon.
    Each triangle is returned as a tuple of three (x, y, z) coordinates.

    Parameters
    ----------
    list_of_points
        a list of (x, y, z) coordinates defining a closed polygon.
    """
    # Create a triangulator object
    triangulator = Triangulator3()

    # Iterate through each point in the list
    for point in list_of_points:
        # Add each point to the pool of vertices and then to the current polygon
        triangulator.addPolygonVertex(triangulator.addVertex(x=point[0], y=point[1], z=point[2]))

    # Create the triangles of the polygons
    triangulator.triangulate()

    # Define list to store polygons
    list_of_triangles = []

    # Add the indices for each triangle to the list of triangle indices
    for i in range(triangulator.getNumTriangles()):
        list_of_triangles.append([triangulator.getTriangleV0(i),
                                  triangulator.getTriangleV1(i),
                                  triangulator.getTriangleV2(i)])
        for j in range(len(list_of_triangles[-1])):
            vertex = triangulator.getVertex(list_of_triangles[-1][j])
            list_of_triangles[-1][j] = (vertex.x, vertex.y, vertex.z)
        list_of_triangles[-1] = tuple(list_of_triangles[-1])

    # Replace the triangle indices with the points themselves
    return list_of_triangles

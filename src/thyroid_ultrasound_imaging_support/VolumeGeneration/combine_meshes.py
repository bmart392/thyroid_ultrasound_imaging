"""
File containing the combine_meshes function definition.
"""


# Import standard python packages
from stl.mesh import Mesh as stl_Mesh
from meshlib.mrmeshpy import loadMesh, boolean, BooleanOperation, saveMesh
from meshlib.mrmeshpy import Mesh as meshlib_Mesh
from meshlib.mrmeshnumpy import meshFromFacesVerts
from numpy import array, zeros

# Import custom python packages
from thyroid_ultrasound_imaging_support.VolumeGeneration.display_mesh_information import display_mesh_information


def combine_meshes(mesh_1, mesh_2,
                   save_result_mesh: str = '',
                   display_mesh_data: str = '') -> stl_Mesh:
    """
    Combines two meshes and can then save the result and display the data about the mesh if desired.

    Parameters
    ----------
    mesh_1 : str or stl_Mesh or meshlib_Mesh
        Either a path to a saved mesh or an existing mesh.
    mesh_2 : str or stl_Mesh or meshlib_Mesh
        Either a path to a saved mesh or an existing mesh.
    save_result_mesh :
        The path in which the combined mesh should be saved.
    display_mesh_data :
        The name to use when displaying the information about the combined mesh.

    Returns
    -------
    Mesh :
        The combined mesh.
    """

    # Load the meshes that will be combined
    if type(mesh_1) == str:
        working_mesh_1 = loadMesh(mesh_1)
    elif type(mesh_1) == stl_Mesh:
        working_mesh_1 = convert_stl_mesh_to_meshlib_mesh(mesh_1)
    elif type(mesh_1) == meshlib_Mesh:
        working_mesh_1 = mesh_1
    else:
        raise TypeError('mesh_1: ' + str(type(mesh_1)) + ' is not accepted for this function')
    if type(mesh_2) == str:
        working_mesh_2 = loadMesh(mesh_2)
    elif type(mesh_2) == stl_Mesh:
        working_mesh_2 = convert_stl_mesh_to_meshlib_mesh(mesh_2)
    elif type(mesh_2) == meshlib_Mesh:
        working_mesh_2 = mesh_2
    else:
        raise TypeError('mesh_2: ' + str(type(mesh_2)) + ' is not accepted for this function')

    # Combine the meshes into one mesh
    combined_mesh = boolean(working_mesh_1, working_mesh_2, BooleanOperation.Union)

    # Save the mesh data if a path is given
    if len(save_result_mesh) > 0:
        saveMesh(combined_mesh.mesh, save_result_mesh)

    # Convert the combined mesh back to a stl_mesh
    combined_mesh = convert_meshlib_mesh_to_stl_mesh(combined_mesh.mesh)

    # Display mesh data if a name is given
    if len(display_mesh_data) > 0:
        display_mesh_information(display_mesh_data, existing_mesh=combined_mesh)

    # Return the combined mesh
    return combined_mesh


def convert_stl_mesh_to_meshlib_mesh(stl_mesh: stl_Mesh) -> meshlib_Mesh:
    """
    Converts a Mesh object of the stl library to a Mesh object of the meshlib library.

    Parameters
    ----------
    stl_mesh :
        A Mesh object of the stl library type.

    Returns
    -------
    meshlib_Mesh :
        A Mesh object of the meshlib library type.
    """

    # Capture all the vertices that make up the mesh
    all_vertices = []
    for face in stl_mesh.vectors:
        for vertex in face:
            all_vertices.append(tuple(vertex))

    # Remove all duplicates and convert each vertex to a list
    all_vertices = [list(x) for x in set(all_vertices)]

    # Capture all the faces that make up the mesh where each vertex is identified by index
    all_faces = []
    for face in stl_mesh.vectors:
        this_face = []
        for vertex in face:
            this_face.append(all_vertices.index(list(vertex)))
        all_faces.append(this_face)

    # Create the mesh object and return it
    return meshFromFacesVerts(array(all_faces), array(all_vertices))


def convert_meshlib_mesh_to_stl_mesh(meshlib_mesh: meshlib_Mesh) -> stl_Mesh:
    """
    Converts Mesh object of the meshlib library a to a Mesh object of the stl library.

    Parameters
    ----------
    meshlib_mesh :
        A Mesh object of the meshlib library type.

    Returns
    -------
    stl_mesh :
        A Mesh object of the stl library type.

    """

    # Capture all the vertices that make up the mesh
    all_vertices = []
    for point in meshlib_mesh.points.vec:
        all_vertices.append([point.x, point.y, point.z])

    # Capture all the faces that make up the mesh where each vertex is identified by index
    all_faces = []
    for face in meshlib_mesh.topology.getAllTriVerts():
        this_face = []
        for vertex_id in face:
            this_face.append(vertex_id.get())
        all_faces.append(this_face)

    # Convert vertices and faces to appropriate numpy arrays
    vertices_np = array(all_vertices)
    faces_np = array(all_faces)

    # Create the stl mesh object using numpy-stl
    result_mesh = stl_Mesh(zeros(faces_np.shape[0], dtype=stl_Mesh.dtype))

    # Populate the mesh object with the correct data
    for i, f in enumerate(faces_np):
        for j in range(3):  # each face has 3 vertices
            result_mesh.vectors[i][j] = vertices_np[f[j]]

    # Return the resulting mesh
    return result_mesh


if __name__ == '__main__':

    # Display the information about the individual meshes
    rod_mesh = display_mesh_information('Rod', path_to_file='/home/ben/Desktop/test_rod.stl')
    square_mesh = display_mesh_information('Square', path_to_file='/home/ben/Desktop/test_square.stl')

    # Combine the meshes
    combine_meshes(loadMesh('/home/ben/Desktop/test_rod.stl'), square_mesh,
                   save_result_mesh='/home/ben/Desktop/test_rod_cube_combined_repaired.stl',
                   display_mesh_data='Rod + Square')

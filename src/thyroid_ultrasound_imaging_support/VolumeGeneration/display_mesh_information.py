"""
File containing the display_mesh_information function definition.
"""


# Import standard python packages
from stl.mesh import Mesh

# Import custom python packages


def display_mesh_information(name: str, path_to_file: str = None, existing_mesh: Mesh = None) -> Mesh:
    """
    Displays the volume information about a mesh file in the terminal console.

    Parameters
    ----------
    name :
        The name of the mesh to display.
    path_to_file :
        The absolute path to the mesh file that should be displayed.
    existing_mesh :
        An existing mesh object that should be displayed.
    Returns
    -------
    Mesh :
        The mesh object that was displayed.

    """
    # Load the mesh at the given file location
    if path_to_file is not None and existing_mesh is None:
        temp_mesh = Mesh.from_file(path_to_file)
    # Or analyze the given mesh
    elif path_to_file is None and existing_mesh is not None:
        temp_mesh = existing_mesh
    # Or raise an exception if no mesh was given
    else:
        raise Exception('No mesh was supplied.')

    # Analyze the mesh
    temp_volume, temp_cog, temp_inertia = temp_mesh.get_mass_properties()

    # Display the information in an easy-to-read format
    print('=' * 25)
    print(name + ' Volume')
    print('-' * 25)
    print("Volume (mm^3) = {0}".format(round(temp_volume / 10 ** -9, 1)))
    print("Volume (mL)   = {0}".format(round(temp_volume * 10 ** 6, 4)))

    # Return the mesh that was analyzed
    return temp_mesh


if __name__ == '__main__':
    pass

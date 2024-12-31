def construct_faces_from_prism_vertices(vertices):
    """Constructs the faces of a prism from all of its vertices. Used for converting gamegine's 3D shapes to PyBullet-Compatible meshes.

    :param vertices: The vertices of the prism.
    :type vertices: list
    :return: The faces of the prism.
    :rtype: list
    """
    base_length = len(vertices) // 2

    faces = []

    # Create the faces for the sides of the prism
    for i in range(base_length - 1):
        index = i
        faces.append((index, index + 1, index + base_length + 1, index + base_length))

    faces.append((base_length - 1, 0, base_length, 2 * base_length - 1))

    # Create the faces for the top and bottom of the prism
    faces.append(tuple(range(base_length)))

    faces.append(tuple(range(base_length, 2 * base_length)))

    return faces

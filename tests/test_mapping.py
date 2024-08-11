def test_create_map():
    from gamegine.analysis.meshing import Map

    map = Map("Test Map")
    assert map.name == "Test Map"
    assert map.nodes == {}
    assert map.get_all_nodes() == []
    assert map.get_all_unique_connections() == []


def test_add_node():
    from gamegine.analysis.meshing import Map
    from gamegine.utils.NCIM.ncim import Feet

    map = Map("Test Map")
    map.add_node(Feet(1), Feet(2))
    assert len(map.get_all_nodes()) == 1
    assert map.get_all_unique_connections() == []
    assert map.encode_coordinates(Feet(1), Feet(2)) == 0


def test_add_edge():
    from gamegine.analysis.meshing import Map
    from gamegine.utils.NCIM.ncim import Feet

    map = Map("Test Map")
    map.add_node(Feet(1), Feet(2))
    map.add_node(Feet(3), Feet(4))
    map.add_edge((Feet(1), Feet(2)), (Feet(3), Feet(4)), Feet(3))
    assert len(map.get_neighbours(0)) == 1
    assert len(map.get_neighbours(1)) == 1


def test_add_edges():
    from gamegine.analysis.meshing import Map
    from gamegine.utils.NCIM.ncim import Feet

    map = Map("Test Map")
    map.add_node(Feet(1), Feet(2))
    map.add_node(Feet(3), Feet(4))
    map.add_edges(
        [
            ((Feet(1), Feet(2)), (Feet(3), Feet(4)), Feet(3)),
            ((Feet(1), Feet(2)), (Feet(4), Feet(5)), Feet(4)),
        ]
    )
    assert len(map.get_neighbours(0)) == 2
    assert len(map.get_neighbours(1)) == 1


def test_add_edges_no_distance():
    from gamegine.analysis.meshing import Map
    from gamegine.utils.NCIM.ncim import Feet

    map = Map("Test Map")
    map.add_node(Feet(1), Feet(2))
    map.add_node(Feet(3), Feet(4))
    map.add_edges(
        [
            ((Feet(1), Feet(2)), (Feet(3), Feet(4)), None),
            ((Feet(1), Feet(2)), (Feet(4), Feet(5)), None),
        ]
    )
    assert len(map.get_neighbours(0)) == 2
    assert len(map.get_neighbours(1)) == 1


def test_add_edges_no_node_initialization():
    from gamegine.analysis.meshing import Map
    from gamegine.utils.NCIM.ncim import Feet

    map = Map("Test Map")
    map.add_edges(
        [
            ((Feet(1), Feet(2)), (Feet(3), Feet(4)), Feet(3)),
            ((Feet(1), Feet(2)), (Feet(4), Feet(5)), Feet(4)),
            ((Feet(3), Feet(4)), (Feet(4), Feet(5)), Feet(5)),
            ((Feet(1), Feet(2)), (Feet(6), Feet(7)), Feet(4)),
        ]
    )
    assert len(map.get_neighbours(0)) == 3
    assert len(map.get_neighbours(1)) == 2
    assert len(map.get_neighbours(2)) == 2
    assert len(map.get_neighbours(3)) == 1
    assert len(map.get_all_unique_connections()) == 4
    assert len(map.get_all_nodes()) == 4

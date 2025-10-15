"""Graph construction helpers for navigation meshes used by the path planner.

The :class:`Map` class stores a thin wrapper around a visibility/triangulated graph
backed by NCIM aware coordinates.  Each node stores its original coordinate in meters and
the adjacency list with travel distances.  The module also exposes factory helpers for
building visibility graphs and triangular lattices that respect field obstacles.

The data structure intentionally focuses on expressiveness rather than raw performance.
The navigation subsystem only operates on a handful of nodes at a time, so the overhead of
keeping rich metadata and validation hooks is acceptable and makes debugging far easier.
"""

import math
from typing import List, Set, Tuple
from enum import Enum

import pygame

from gamegine.render.drawable import Drawable
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import (
    Boundary,
    BoundedObject,
    DiscreteBoundary,
    LineIntersectsAnyBound,
)
from gamegine.utils.NCIM.Dimensions import spatial
from gamegine.utils.logging import Debug
from gamegine.utils.matematika import CoordinateInRectangle, GetDistanceBetween
from gamegine.utils.NCIM.ncim import (
    Centimeter,
    Inch,
    RatioOf,
    SpatialMeasurement,
)


class ConnectionStrategy(Enum):
    Nearest = 1
    LineOfSight = 2


# TODO: Add display for one-directional edges and colors depending on weight
class Map(Drawable):
    """Mutable graph describing the walkable field surface.

    The structure keeps a mapping between NCIM coordinates and integer identifiers and
    supplies convenience accessors for rendering and nearest-node queries.  All methods
    accept NCIM measurements so downstream consumers do not have to reason about raw
    floats.
    """

    def __init__(self, name: str = "Map") -> None:
        """Initialise an empty graph with a human friendly name."""
        self.name = name
        self.nodes = {}
        self.encoding = {}
        self.id = 0

        # Rendering Caches
        self.nodes_cache = self.get_all_nodes()
        self.connections_cache = self.get_all_unique_connections()
        self.cache_up_to_date = True

    def add_node(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Map":
        """Register a node at the given coordinates.

        The node id is generated automatically.  Adding a duplicate coordinate raises an
        exception to avoid ambiguous routing behaviour.
        """

        coord = (float(x), float(y))
        self.nodes[self.id] = (coord, {})
        if coord in self.encoding:
            raise Exception(f"Node at ({x}, {y}) already exists.")
        self.encoding[coord] = self.id
        self.id += 1
        self.__cache_outdated()
        return self

    def add_edge(
        self,
        node1: Tuple[SpatialMeasurement, SpatialMeasurement],
        node2: Tuple[SpatialMeasurement, SpatialMeasurement],
        weight: float = None,
        bidirectional: bool = True,
        weight_backward: float = None,
    ) -> "Map":
        """Create an edge (optionally bidirectional) between two coordinates.

        Distances default to straight-line distance if not provided.  Missing nodes are
        created on the fly so caller code can treat the map as append-only.
        """

        if node1 not in self.encoding:
            Debug(f"Node at {node1} does not exist. Attempting to add node.")
            self.add_node(node1[0], node1[1])
        if node2 not in self.encoding:
            Debug(f"Node at {node2} does not exist. Attempting to add node.")
            self.add_node(node2[0], node2[1])

        node1_id = self.encoding[node1]
        node2_id = self.encoding[node2]
        if node1_id == node2_id:
            raise Exception("Cannot add edge between the same node.")

        if weight is None:
            weight = GetDistanceBetween(node1, node2)

        self.nodes[node1_id][1][node2_id] = weight

        if bidirectional:
            if weight_backward is None:
                weight_backward = weight
            self.nodes[node2_id][1][node1_id] = weight_backward
        self.__cache_outdated()
        return self

    def add_one_way_edge(
        self,
        node1: Tuple[SpatialMeasurement, SpatialMeasurement],
        node2: Tuple[SpatialMeasurement, SpatialMeasurement],
        weight: float,
    ) -> "Map":
        """Convenience wrapper for inserting a directed edge only."""
        return self.add_edge(node1, node2, weight, False)

    def add_edges(
        self,
        edges: List[
            Tuple[
                Tuple[SpatialMeasurement, SpatialMeasurement],
                Tuple[SpatialMeasurement, SpatialMeasurement],
                SpatialMeasurement,
            ]
        ],
    ) -> "Map":
        """Bulk insert helper for iterables of ``(start, end, weight)`` tuples."""
        for edge in edges:
            self.add_edge(edge[0], edge[1], edge[2])
        return self

    def get_node(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> Tuple[int, Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Return the internal node id for the provided coordinate."""
        coord = (x, y)
        if coord not in self.encoding:
            raise Exception(f"Node at ({x}, {y}) does not exist.")
        return (self.encoding[coord], coord)

    def get_neighbours(self, node_id: int) -> List[Tuple[int, SpatialMeasurement]]:
        """Return the neighbour id/weight pairs for the selected node."""
        if node_id not in self.nodes:
            raise Exception(f"Node with id {node_id} does not exist.")
        return [
            (neighbour_id, distance)
            for neighbour_id, distance in self.nodes[node_id][1].items()
        ]

    def get_all_nodes(
        self,
    ) -> List[Tuple[int, Tuple[SpatialMeasurement, SpatialMeasurement]]]:
        """Expose node ids with their decoded coordinates."""
        return [(node_id, node[0]) for node_id, node in self.nodes.items()]

    def encode_coordinates(self, x: SpatialMeasurement, y: SpatialMeasurement) -> int:
        """Return the integer id associated with the coordinate."""
        coord = (x, y)
        if coord not in self.encoding:
            raise Exception(f"Node at ({x}, {y}) does not exist.")
        return self.encoding[coord]

    def decode_coordinates(
        self, node_id: int
    ) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Convert a node id back to absolute NCIM coordinates."""
        if node_id not in self.nodes:
            raise Exception(f"Node with id {node_id} does not exist.")
        raw = self.nodes[node_id][0]
        return (
            SpatialMeasurement(0, spatial.Meter, base_magnitude=raw[0]),
            SpatialMeasurement(0, spatial.Meter, base_magnitude=raw[1]),
        )

    def get_all_unique_connections(
        self,
    ) -> List[Tuple[Tuple[int, int], Tuple[float], bool]]:
        """Generate a unique edge list for rendering and introspection."""
        connections = {}
        for node_id, node in self.nodes.items():
            for id, weight in node[1].items():
                if (id, node_id) in connections:
                    connections[(id, node_id)] = (connections[(id, node_id)], weight)
                else:
                    connections[(node_id, id)] = weight

        return [
            (connection, weights, len(weights) == 2)
            for connection, weights in connections.items()
        ]

    def connect_all_nodes(self) -> "Map":
        """Fully connect the graph (mostly useful while prototyping)."""
        items = list(self.nodes.items())
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                node1_id, node1 = items[i]
                node2_id, node2 = items[j]
                self.add_edge(node1[0], node2[0])
        return self

    def connect_all_points(
        self, points: List[Tuple[SpatialMeasurement, SpatialMeasurement]]
    ) -> "Map":
        """Insert nodes for the supplied coordinates and fully connect them."""
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                self.add_edge(points[i], points[j])
        return self

    def add_connected_node(
        self, x: SpatialMeasurement, y: SpatialMeasurement, strategy: ConnectionStrategy
    ) -> "Map":
        # TODO: Implement connection strategy
        return self

    def get_closest_node(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> Tuple[int, Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Return the nearest node id/coordinate pair to the supplied point."""
        coord = (x, y)
        if coord in self.encoding:
            return self.get_node(x, y)
        return self.get_node(
            *min(
                self.nodes.items(),
                key=lambda node: GetDistanceBetween(node[1][0], coord),
            )[1][0]
        )

    def __cache_outdated(self):
        self.cache_up_to_date = False

    def __update_nodes_cache(self) -> None:
        self.nodes_cache = self.get_all_nodes()

    def __update_connections_cache(self) -> None:
        self.connections_cache = self.get_all_unique_connections()

    def z_index(self) -> int:
        return 0

    def draw(self, render_scale: SpatialMeasurement) -> None:
        if not self.cache_up_to_date:
            self.__update_nodes_cache()
            self.__update_connections_cache()
            self.cache_up_to_date = True

        for node in self.nodes_cache:
            pygame.draw.circle(
                pygame.display.get_surface(),
                (0, 255, 0),
                (Renderer.to_pixels(node[1][0]), Renderer.to_pixels(node[1][1])),
                5,
            )

        for connection in self.connections_cache:
            one, two = connection[0][0], connection[0][1]
            node1 = self.decode_coordinates(one)
            node2 = self.decode_coordinates(two)
            color = (0, 255, 0) if connection[2] else (255, 0, 0)
            pygame.draw.line(
                pygame.display.get_surface(),
                color,
                (Renderer.to_pixels(node1[0]), Renderer.to_pixels(node1[1])),
                (Renderer.to_pixels(node2[0]), Renderer.to_pixels(node2[1])),
                width=1,
            )


def VisibilityGraph(
    obstacles: List[Boundary],
    required_points: List[Tuple[SpatialMeasurement, SpatialMeasurement]] = [],
    clip_to: Tuple[SpatialMeasurement, SpatialMeasurement] = None,
    discretization_quality: int = 4,
) -> Map:
    """Create a visibility graph that links mutually visible obstacle vertices.

    The routine discretises each obstacle boundary (respecting ``discretization_quality``)
    and connects any pair of points that can see each other without intersecting an
    obstacle.  Additional waypoints can be injected through ``required_points`` and the
    search space can optionally be clipped to a bounding rectangle.
    """
    map = Map("Visibility Graph")
    discrete_bounds = [
        obstacle.discretized(discretization_quality) for obstacle in obstacles
    ]
    points = [
        point for bounds in discrete_bounds for point in bounds.get_vertices()
    ]  # This is cooked...also flattening this array and adding necessary points
    points.extend(required_points)

    for i, point in enumerate(points):
        for j in range(i + 1, len(points)):
            if clip_to is not None:
                if not (
                    CoordinateInRectangle(point, [0, 0, *clip_to])
                    and CoordinateInRectangle(points[j], [0, 0, *clip_to])
                ):
                    continue

            # Move points on line along slope slightly to avoid intersection with the edge
            # Warning: Super Jank Solution                             / \
            # TODO: Fix this jank solution...later...possibly...plz \_(._.)_/
            intersection_line = (point[0], point[1], points[j][0], points[j][1])
            slope = (intersection_line[3] - intersection_line[1]) / (
                intersection_line[2] - intersection_line[0] + Centimeter(1e-6)
            )
            step = Centimeter(0.01)
            intersection_line = (
                intersection_line[0] + step,
                intersection_line[1] + step * slope,
                intersection_line[2] - step,
                intersection_line[3] - step * slope,
            )

            if not LineIntersectsAnyBound(discrete_bounds, *intersection_line):
                map.add_edge(point, points[j])
    return map


def TriangulatedGraph(
    obstacles: List[Boundary],
    triangle_size: SpatialMeasurement,
    field_bounds: Tuple[SpatialMeasurement, SpatialMeasurement],
    discretization_quality: int = 4,
) -> Map:
    """Generate a regular triangular lattice that excludes blocked cells.

    Nodes whose centres intersect an obstacle or fall outside the field bounds are
    skipped.  Remaining nodes are fully connected within each triangle in order to
    preserve symmetry.
    """
    map = Map("Triangulated Graph")
    discrete_bounds = [
        obstacle.discretized(discretization_quality) for obstacle in obstacles
    ]
    altitude = math.sqrt(3) * triangle_size * 0.5
    for row in range(math.ceil(RatioOf(field_bounds[1], altitude))):
        x_offset = row % 2 * (0.5 * triangle_size)
        for column in range(math.ceil(RatioOf(field_bounds[0], triangle_size))):
            points = [
                (x_offset + column * triangle_size, (row + 1) * altitude),
                (x_offset + (column + 1) * triangle_size, (row + 1) * altitude),
                (x_offset + (column + 0.5) * triangle_size, (row) * altitude),
            ]

            good_points = []
            for point in points:
                eliminate = False
                for bound in discrete_bounds:
                    if bound.contains_point(*point) or not CoordinateInRectangle(
                        point, [0, 0, *field_bounds]
                    ):
                        eliminate = True
                        break
                if not eliminate:
                    good_points.append(point)

            map.connect_all_points(good_points)

    return map

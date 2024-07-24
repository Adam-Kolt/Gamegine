import math
from queue import PriorityQueue
from typing import Callable, Dict, List, Tuple
from pint import Quantity
from enum import Enum
from abc import ABC, abstractmethod

import pygame

from gamegine.analysis.meshing import Map
from gamegine.render.drawable import Drawable
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import Boundary, DiscreteBoundary
from gamegine.utils.logging import Debug
from gamegine.utils.matematika import AngleBetweenVectors, GetDistanceBetween
from gamegine.utils.unit import StdMag, StdMagTuple


class InitialConnectionPolicy(Enum):
    ConnectToClosest = 0
    SnapToClosest = 1
    VisibilityConnect = 2


class Path(Drawable):  # TODO: Use in codebase
    def __init__(self, path: List[Tuple[Quantity, Quantity]]) -> None:
        self.path = path

    def set_path(self, path: List[Tuple[Quantity, Quantity]]) -> None:
        self.path = path

    def add_point(self, point: Tuple[Quantity, Quantity]) -> None:
        self.path.append(point)

    def get_points(self) -> List[Tuple[Quantity, Quantity]]:
        return self.path

    def shortcut(self, discrete_obstacles: List[DiscreteBoundary]) -> "Path":
        if len(self.path) <= 2:
            return self.path
        out = [self.path[0]]
        current = self.path[0]

        i = 2
        while i < len(self.path) - 1:
            if any(
                obstacle.intersects_line(*current, *self.path[i])
                for obstacle in discrete_obstacles
            ):
                Debug(
                    f"Path segment {current} -> {self.path[i]} intersects obstacle. Skipping."
                )
                out.append(self.path[i - 1])
                current = self.path[i - 1]
            i += 1
        out.append(self.path[-1])
        self.path = out
        return self

    def shortcutted(self, discrete_obstacles: List[DiscreteBoundary]) -> "Path":
        return Path(shortcut_tuple_path(discrete_obstacles, self.path))

    def z_index(self) -> int:
        return 1

    def draw(self, render_scale: Quantity) -> None:
        for i in range(len(self.path) - 1):
            node1 = self.path[i]
            node2 = self.path[i + 1]
            Debug(f"Drawing line from {node1} to {node2}")
            pygame.draw.line(
                pygame.display.get_surface(),
                (255, 0, 0),
                (Renderer.to_pixels(node1[0]), Renderer.to_pixels(node1[1])),
                (Renderer.to_pixels(node2[0]), Renderer.to_pixels(node2[1])),
                width=2,
            )


class Pathfinder(ABC):
    @abstractmethod
    def calculate_path(map: Map, start: int, end: int) -> List[int]:
        pass


class Heuristics(object):
    @staticmethod
    def EuclideanHeuristic(map: Map, start: int, end: int, current: int, next: int):
        end_coords = StdMagTuple(map.decode_coordinates(end))
        next_coords = StdMagTuple(map.decode_coordinates(next))
        return GetDistanceBetween(end_coords, next_coords)

    @staticmethod
    def DirectedEuclideanHeuristic(
        map: Map, start: int, end: int, current: int, next: int
    ):
        end_coords = StdMagTuple(map.decode_coordinates(end))
        next_coords = StdMagTuple(map.decode_coordinates(next))
        current_coords = StdMagTuple(map.decode_coordinates(current))
        direction_weight = 0.00
        vector_to_next = (
            next_coords[0] - current_coords[0],
            next_coords[1] - current_coords[1],
        )
        vector_to_end = (end_coords[0] - next_coords[0], end_coords[1] - next_coords[1])
        return (
            GetDistanceBetween(end_coords, next_coords)
            + AngleBetweenVectors(vector_to_next, vector_to_end) * direction_weight
        )


class AStar(Pathfinder):
    class Node:
        def __init__(
            self, id: int, parent: int, g_score: float, f_score: float
        ) -> None:
            self.id = id
            self.parent = parent
            self.g_score = g_score
            self.f_score = f_score

    @staticmethod
    def __retrace_steps(nodes: Dict[int, Node], end_node: Node) -> List[int]:
        out = []
        parent_id = end_node.id
        while parent_id != None:
            current_node = nodes[parent_id]
            out.append(current_node.id)
            parent_id = current_node.parent
        return out[::-1]

    @staticmethod
    def calculate_path(
        map: Map,
        start: int,
        end: int,
        heuristic: Callable[
            [Map, int, int, int, int], float
        ] = Heuristics.EuclideanHeuristic,
    ) -> List[int]:
        open_set = PriorityQueue()
        nodes = {}

        # Initialize start node
        nodes[start] = AStar.Node(start, None, 0, 0)
        open_set.put((nodes[start].f_score, start))

        while not open_set.empty():
            current_node: AStar.Node = nodes[open_set.get()[1]]

            if current_node.id == end:  # Found path
                return AStar.__retrace_steps(nodes, current_node)

            neighbors = map.get_neighbours(current_node.id)
            for neighbor in neighbors:
                g_score = StdMag(neighbor[1]) + current_node.g_score
                f_score = g_score + heuristic(
                    map, start, end, current_node.id, neighbor[0]
                )
                if not neighbor[0] in nodes or f_score < nodes[neighbor[0]].f_score:
                    nodes[neighbor[0]] = AStar.Node(
                        neighbor[0], current_node.id, g_score, f_score
                    )
                    open_set.put((f_score, neighbor[0]))

        return []  # No path could be found


class DirectedAStar(AStar):
    @staticmethod
    def calculate_path(map: Map, start: int, end: int) -> List[int]:
        return super(DirectedAStar, DirectedAStar).calculate_path(
            map, start, end, Heuristics.DirectedEuclideanHeuristic
        )


def shortcut_tuple_path(
    discrete_obstacles: List[DiscreteBoundary], path: List[Tuple[Quantity, Quantity]]
) -> List[Tuple[Quantity, Quantity]]:
    if len(path) <= 2:
        return path
    out = [path[0]]
    current = path[0]

    i = 2
    while i < len(path) - 1:
        if any(
            obstacle.intersects_line(*current, *path[i])
            for obstacle in discrete_obstacles
        ):
            Debug(f"Path segment {current} -> {path[i]} intersects obstacle. Skipping.")
            out.append(path[i - 1])
            current = path[i - 1]
        i += 1
    out.append(path[-1])
    return out


def findPath(
    map: Map,
    start: Tuple[Quantity, Quantity],
    end: Tuple[Quantity, Quantity],
    pathfinder: Pathfinder = DirectedAStar,
    initial_connection_policy: InitialConnectionPolicy = InitialConnectionPolicy.ConnectToClosest,
) -> Path:
    match initial_connection_policy:
        case InitialConnectionPolicy.ConnectToClosest:
            connect_start = map.get_closest_node(*start)
            if StdMagTuple(start) != StdMagTuple(connect_start[1]):
                map.add_edge(start, connect_start[1])
                start_node = map.encode_coordinates(*start)
            else:
                start_node = connect_start[0]

            connect_end = map.get_closest_node(*end)
            if StdMagTuple(end) != StdMagTuple(connect_end[1]):
                map.add_edge(end, connect_end[1])
                end_node = map.encode_coordinates(*end)
            else:
                end_node = connect_end[0]
        case InitialConnectionPolicy.SnapToClosest:
            start_node = map.get_closest_node(*start)[0]
            end_node = map.get_closest_node(*end)[0]
        case InitialConnectionPolicy.VisibilityConnect:
            raise NotImplementedError(
                "Visibility Connect not implemented yet."
            )  # TODO: Implement visibility connect ¯\_(ツ)_/¯

    path = pathfinder.calculate_path(map, start_node, end_node)

    return Path([map.decode_coordinates(node) for node in path])

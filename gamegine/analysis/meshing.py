
# {node_id : ((x, y), [(neighbour_id, distance), ...]), ...}
from typing import List, Set, Tuple
import pint
from enum import Enum

from gamegine.representation.bounds import BoundedObject, LineIntersectsAnyBound
from gamegine.utils.matematika import CoordinateInRectangle, GetDistanceBetween
from gamegine import ureg
from gamegine.utils.unit import Centimeter, StdMag, StdMagTuple, ToStd, Tuple2Std



class ConnectionStrategy(Enum):
    Nearest = 1
    LineOfSight = 2



class Map(object):
    def __init__(self, name: str = "Map") -> None:
        self.name = name
        self.nodes = {}
        self.encoding = {}
        self.id = 0

    
    def add_node(self, x: pint.Quantity, y: pint.Quantity) -> 'Map':
        if type(x) != float:
            x = StdMag(x)
        if type(y) != float:
            y = StdMag(y)
        coord = (x,y)
        self.nodes[self.id] = (coord, {})
        if coord in self.encoding:
            raise Exception(f"Node at ({x}, {y}) already exists.")
        self.encoding[coord] = self.id
        self.id += 1
        return self

    def add_edge(self, node1: Tuple[pint.Quantity, pint.Quantity], node2: Tuple[pint.Quantity, pint.Quantity], distance: pint.Quantity = None) -> 'Map':
        node1, node2 = StdMagTuple(node1), StdMagTuple(node2)
            
        if node1 not in self.encoding:
            # TODO: Add logging stuff
            # print(f"Node at {node1} does not exist. Attempting to add node.")
            self.add_node(node1[0], node1[1])
        if node2 not in self.encoding:
            # print(f"Node at {node2} does not exist. Attempting to add node.")
            self.add_node(node2[0], node2[1])

        node1_id = self.encoding[node1]
        node2_id = self.encoding[node2]
        if node1_id == node2_id:
            raise Exception("Cannot add edge between the same node.")
        
        if distance is None:
            distance = GetDistanceBetween(node1, node2)
        else: 
            distance = StdMag(distance)
        
        self.nodes[node1_id][1][node2_id] = distance
        self.nodes[node2_id][1][node1_id] = distance
        return self
    
    def add_edges(self, edges: List[Tuple[Tuple[pint.Quantity, pint.Quantity], Tuple[pint.Quantity, pint.Quantity], pint.Quantity]]) -> 'Map':
        for edge in edges:
            self.add_edge(edge[0], edge[1], edge[2])
        return self
    
    def get_node(self, x: pint.Quantity, y: pint.Quantity) -> Tuple[int, Tuple[pint.Quantity, pint.Quantity]]:
        coord = StdMagTuple((x, y))
        if coord not in self.encoding:
            raise Exception(f"Node at ({x}, {y}) does not exist.")
        return (self.encoding[coord], coord)
    
    def get_neighbours(self, node_id: int) -> List[Tuple[int, pint.Quantity]]:
        if node_id not in self.nodes:
            raise Exception(f"Node with id {node_id} does not exist.")
        return [(neighbour_id, ToStd(distance)) for neighbour_id, distance in self.nodes[node_id][1].items()]
    
    def get_all_nodes(self) -> List[Tuple[int, Tuple[pint.Quantity, pint.Quantity]]]:
        return [(node_id, node[0]) for node_id, node in self.nodes.items()]
    
    def encode_coordinates(self, x: pint.Quantity, y: pint.Quantity) -> int:
        coord = StdMagTuple((x, y))
        if coord not in self.encoding:
            raise Exception(f"Node at ({x}, {y}) does not exist.")
        return self.encoding[(x, y)]
    
    def decode_coordinates(self, node_id: int) -> Tuple[pint.Quantity, pint.Quantity]:
        if node_id not in self.nodes:
            raise Exception(f"Node with id {node_id} does not exist.")
        return self.nodes[node_id][0]
    
    def get_all_unique_connections(self) -> List[Set[Tuple[int, int]]]:
        connections = []
        for node_id, node in self.nodes.items():
            for id, distance in node[1].items():
                node_pair = {node_id, id}
                if node_pair not in connections:
                    connections.append(node_pair)
        return connections

    def connect_all_nodes(self) -> 'Map':
        items = list(self.nodes.items())
        for i in range(len(items)):
            for j in range(i+1, len(items)):
                node1_id, node1 = items[i]
                node2_id, node2 = items[j]
                self.add_edge(Tuple2Std(node1[0]), Tuple2Std(node2[0]))
        return self
    
    def add_connected_node(self, x: pint.Quantity, y: pint.Quantity, strategy: ConnectionStrategy) -> 'Map':
        # TODO: Implement connection strategy
        return self

def VisibilityGraph(obstacles: List[BoundedObject], required_points: List[Tuple[pint.Quantity, pint.Quantity]]=[], clip_to: Tuple[pint.Quantity, pint.Quantity] = None, discretization_quality: int = 4) -> Map:
    map = Map("Visibility Graph")
    discrete_bounds = [obstacle.bounds.discretized(discretization_quality) for obstacle in obstacles]
    points = [point for bounds in discrete_bounds for point in bounds.get_vertices()] # This is cooked...also flattening this array and adding necessary points
    points.extend(required_points)
    
    for i, point in enumerate(points):
        for j in range(i+1, len(points)):
            if clip_to is not None:
                if not (CoordinateInRectangle(point, [0, 0, *clip_to]) and CoordinateInRectangle(points[j], [0, 0, *clip_to])):
                    continue

            # Move points on line along slope slightly to avoid intersection with the edge
            # Warning: Super Jank Solution                             / \
            # TODO: Fix this jank solution...later...possibly...plz \_(._.)_/
            intersection_line = (point[0], point[1], points[j][0], points[j][1])
            slope = (intersection_line[3] - intersection_line[1]) / (intersection_line[2] - intersection_line[0] + Centimeter(1e-6))
            step = Centimeter(0.01)
            intersection_line = (intersection_line[0] + step, intersection_line[1] + step*slope, intersection_line[2] - step, intersection_line[3] - step*slope)
            
            if not LineIntersectsAnyBound(discrete_bounds, *intersection_line):
                map.add_edge(point, points[j])
    return map








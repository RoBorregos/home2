#!/usr/bin/env python3

"""
Exploration Planner modulle for EGPSR task.
"""

import json
import math
import heapq
from typing import List, Tuple

class ExplorationPlanner:
    def __init__(self, areas_json_path: str):
        """Initialize the exploration planner with areas data"""
        with open(areas_json_path, "r") as file:
            self.areas = json.load(file)

        # Extract valid exploration areas (exclude start_area and entrance)
        self.exploration_areas = {
            name: data
            for name, data in self.areas.items()
            if name not in ["start_area", "entrance"]
            and "safe_place" in data
            and data["safe_place"]
        }

        self.distances = {}
        self._calculate_distances()

    def _calculate_distances(self):
        """Calculate Euclidean distances between all areas"""
        area_names = list(self.exploration_areas.keys())

        for i, area1 in enumerate(area_names):
            for j, area2 in enumerate(area_names):
                if i != j:
                    dist = self._euclidean_distance(area1, area2)
                    self.distances[(area1, area2)] = dist
                else:
                    self.distances[(area1, area2)] = 0

    def _euclidean_distance(self, area1: str, area2: str) -> float:
        """Calculate Euclidean distance between two areas"""
        pos1 = self.exploration_areas[area1]["safe_place"]
        pos2 = self.exploration_areas[area2]["safe_place"]

        # Extract x, y coordinates
        x1, y1 = pos1[0], pos1[1]
        x2, y2 = pos2[0], pos2[1]

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def dijkstra_shortest_path(self, start: str, end: str) -> Tuple[List[str], float]:
        """Find shortest path between two areas using Dijkstra's algorithm"""
        if start not in self.exploration_areas or end not in self.exploration_areas:
            return [], float("inf")

        # Priority queue: (distance, node)
        pq = [(0, start)]
        distances = {area: float("inf") for area in self.exploration_areas}
        distances[start] = 0
        previous = {}
        visited = set()

        while pq:
            current_dist, current = heapq.heappop(pq)

            if current in visited:
                continue

            visited.add(current)

            if current == end:
                break

            for neighbor in self.exploration_areas:
                if neighbor not in visited:
                    distance = current_dist + self.distances[(current, neighbor)]

                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous[neighbor] = current
                        heapq.heappush(pq, (distance, neighbor))

        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()

        return path, distances[end]

    def plan_exploration_order(self, start_area: str = "start_area") -> List[str]:
        """Plan optimal exploration order using nearest neighbor heuristic"""
        if start_area not in self.areas:
            start_area = "start_area"

        # Get starting position
        start_pos = self.areas[start_area]["safe_place"]

        # Find nearest exploration area to start
        unvisited = set(self.exploration_areas.keys())
        current_pos = start_pos
        exploration_order = []

        # Find closest area to start position
        min_dist = float("inf")
        next_area = None

        for area in unvisited:
            area_pos = self.exploration_areas[area]["safe_place"]
            dist = math.sqrt(
                (area_pos[0] - current_pos[0]) ** 2 + (area_pos[1] - current_pos[1]) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                next_area = area

        if next_area:
            exploration_order.append(next_area)
            unvisited.remove(next_area)
            current_area = next_area

        # Continue with nearest neighbor
        while unvisited:
            min_dist = float("inf")
            next_area = None

            for area in unvisited:
                dist = self.distances[(current_area, area)]
                if dist < min_dist:
                    min_dist = dist
                    next_area = area

            if next_area:
                exploration_order.append(next_area)
                unvisited.remove(next_area)
                current_area = next_area

        return exploration_order

    def get_total_exploration_distance(self, order: List[str]) -> float:
        """Calculate total distance for exploration order"""
        if not order:
            return 0

        total_dist = 0

        # Distance from start to first area
        start_pos = self.areas["start_area"]["safe_place"]
        first_pos = self.exploration_areas[order[0]]["safe_place"]
        total_dist += math.sqrt(
            (first_pos[0] - start_pos[0]) ** 2 + (first_pos[1] - start_pos[1]) ** 2
        )

        # Distance between consecutive areas
        for i in range(len(order) - 1):
            total_dist += self.distances[(order[i], order[i + 1])]

        return total_dist

    def get_distance_between_area_subarea(self, area1: str, subarea1: str, area2: str, subarea2: str) -> float:
        """
        Get the Euclidean distance between [area1][subarea1] and [area2][subarea2].
        Returns float('inf') if any area or subarea is missing.
        """
        try:
            pos1 = self.areas[area1][subarea1]
            pos2 = self.areas[area2][subarea2]
            x1, y1 = pos1[0], pos1[1]
            x2, y2 = pos2[0], pos2[1]
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        except Exception:
            return float('inf')
        
    def get_nearest_trashbin(self) -> dict:
        """
        Get the nearest trash bin location for all locations.
        Returns a dictionary with the nearest trash bin for each location.
        """
        nearest_trashbin = {}

        for current_location in self.areas.keys():
            # If current location has a trashbin, use it directly
            if "trashbin" in self.areas[current_location]:
                nearest_trashbin[current_location] = current_location
                continue

            min_distance = float('inf')
            nearest_area = None
            for area, subarea in self.areas.items():
                if "trashbin" in subarea:
                    distance = self.get_distance_between_area_subarea(
                    current_location, "safe_place", area, "trashbin"
                    )
                    if distance < min_distance:
                        min_distance = distance
                        nearest_area = area
                nearest_trashbin[current_location] = nearest_area

        return nearest_trashbin
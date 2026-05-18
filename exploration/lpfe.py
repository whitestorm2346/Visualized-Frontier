import math
from collections import deque

from exploration.base import ExplorationPolicy
from config import UNKNOWN, FREE


class LPFEPolicy(ExplorationPolicy):
    def __init__(self):
        self.target = None
        self.blacklist = set()
        self.info_gain_weight = 0.8

    def get_action(self, robot, occupancy_map, frontiers, world=None):
        if not frontiers:
            self.target = None
            return 0, 0

        # target 不存在或 target 已經不是 frontier 時，重新選
        valid_frontiers = [f for f in frontiers if f not in self.blacklist]

        if not valid_frontiers:
            self.blacklist.clear()
            valid_frontiers = list(frontiers)

        if self.target is None or self.target not in valid_frontiers:
            distance_map = self._build_wavefront_map(robot, occupancy_map)

            self.target = self._select_best_frontier(
                occupancy_map,
                valid_frontiers,
                distance_map
            )

        if self.target is None:
            return 0, 0

        return {
            "type": "target",
            "target": self.target
        }
    

    def on_stuck(self, target):
        if target is not None:
            self.blacklist.add(target)

        self.target = None

    def _build_wavefront_map(self, robot, occupancy_map):
        start_x, start_y = robot.grid_pos()

        distance_map = [
            [float("inf") for _ in range(occupancy_map.width)]
            for _ in range(occupancy_map.height)
        ]

        queue = deque()
        queue.append((start_x, start_y))

        distance_map[start_y][start_x] = 0

        while queue:
            x, y = queue.popleft()

            for nx, ny in self._get_neighbors_4(x, y):
                if not occupancy_map.is_inside(nx, ny):
                    continue

                if occupancy_map.get_cell(nx, ny) != FREE:
                    continue

                if distance_map[ny][nx] != float("inf"):
                    continue

                distance_map[ny][nx] = distance_map[y][x] + 1
                queue.append((nx, ny))

        return distance_map

    def _select_best_frontier(self, occupancy_map, frontiers, distance_map):
        best_frontier = None
        best_cost = float("inf")

        for fx, fy in frontiers:
            distance_cost = distance_map[fy][fx]

            if distance_cost == float("inf"):
                continue

            information_gain = self._information_gain(occupancy_map, fx, fy)

            cost = distance_cost - self.info_gain_weight * information_gain

            if cost < best_cost:
                best_cost = cost
                best_frontier = (fx, fy)

        return best_frontier

    def _information_gain(self, occupancy_map, fx, fy, radius=4):
        gain = 0

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx = fx + dx
                ny = fy + dy

                if not occupancy_map.is_inside(nx, ny):
                    continue

                if math.sqrt(dx * dx + dy * dy) > radius:
                    continue

                if occupancy_map.get_cell(nx, ny) == UNKNOWN:
                    gain += 1

        return gain

    def _get_neighbors_4(self, x, y):
        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]
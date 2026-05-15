import math
from collections import deque

from exploration.base import ExplorationPolicy
from config import UNKNOWN, FREE


class LPFEPolicy(ExplorationPolicy):
    def __init__(self):
        self.target = None
        self.path = None
        self.waypoint = None
        self.mode = "ALIGN_CENTER"

        self.arrive_threshold = 0.08
        self.info_gain_weight = 0.8

    def get_action(self, robot, occupancy_map, frontiers, world=None):
        if not frontiers:
            self.target = None
            self.path = None
            self.waypoint = None
            return 0, 0

        # 只有 target 不存在或失效時，才重新做 wavefront
        if self.target is None or self.target not in frontiers or self.path is None:
            distance_map, parent_map = self._build_wavefront_map(robot, occupancy_map)

            self.target = self._select_best_frontier(
                occupancy_map,
                frontiers,
                distance_map
            )

            if self.target is None:
                self.path = None
                self.waypoint = None
                return 0, 0

            self.path = self._reconstruct_path(parent_map, self.target)
            self.waypoint = None

        if self.path is None or len(self.path) < 2:
            self.target = None
            self.path = None
            self.waypoint = None
            self.mode = "ALIGN_CENTER"
            return 0, 0

        if self.mode == "ALIGN_CENTER":
            current_cell = robot.grid_pos()
            cx = current_cell[0] + 0.5
            cy = current_cell[1] + 0.5

            dx = cx - robot.x
            dy = cy - robot.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > 0.05:
                return dx / dist, dy / dist

            self.mode = "MOVE_TO_NEXT"
            self.waypoint = None

        if self.mode == "MOVE_TO_NEXT":
            if self.waypoint is None:
                next_cell = self.path[1]
                self.waypoint = (next_cell[0] + 0.5, next_cell[1] + 0.5)

            wx, wy = self.waypoint
            dx = wx - robot.x
            dy = wy - robot.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.arrive_threshold:
                self.path.pop(0)
                self.waypoint = None
                self.mode = "ALIGN_CENTER"
                return 0, 0

            return dx / dist, dy / dist

    def _build_wavefront_map(self, robot, occupancy_map):
        start_x, start_y = robot.grid_pos()

        distance_map = [
            [float("inf") for _ in range(occupancy_map.width)]
            for _ in range(occupancy_map.height)
        ]

        parent_map = {}

        queue = deque()
        queue.append((start_x, start_y))

        distance_map[start_y][start_x] = 0
        parent_map[(start_x, start_y)] = None

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
                parent_map[(nx, ny)] = (x, y)
                queue.append((nx, ny))

        return distance_map, parent_map

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

    def _reconstruct_path(self, parent_map, goal):
        if goal not in parent_map:
            return None

        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = parent_map[current]

        path.reverse()
        return path

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
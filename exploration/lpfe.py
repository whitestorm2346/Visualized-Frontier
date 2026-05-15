import math

from collections import deque
from exploration.base import ExplorationPolicy
from config import UNKNOWN, FREE, OBSTACLE


class LPFEPolicy(ExplorationPolicy):
    def __init__(self):
        self.target = None
        self.waypoint = None
        self.arrive_threshold = 0.08
        self.path = None

        self.blacklist = set()

    def get_action(self, robot, occupancy_map, frontiers, world=None):
        if not frontiers:
            return 0, 0

        # blacklist 過濾
        valid_frontiers = [f for f in frontiers if f not in self.blacklist]

        if not valid_frontiers:
            self.blacklist.clear()
            valid_frontiers = list(frontiers)

        # target 不存在或失效時重新選 frontier
        if self.target is None or self.target not in valid_frontiers:
            self.target = self._select_best_frontier(
                robot,
                occupancy_map,
                valid_frontiers
            )
            
            self.waypoint = None

        path = self._wavefront_path(robot, occupancy_map, self.target[0], self.target[1])

        if path is None or len(path) < 2:
            self.target = None
            self.waypoint = None
            return 0, 0

        # 如果目前沒有 waypoint，或已經到達 waypoint，才換下一個 waypoint
        if self.waypoint is None:
            next_cell = path[1]
            self.waypoint = (next_cell[0] + 0.5, next_cell[1] + 0.5)

        wx, wy = self.waypoint
        dx = wx - robot.x
        dy = wy - robot.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.arrive_threshold:
            self.waypoint = None
            return 0, 0

        return dx / dist, dy / dist

    def _select_best_frontier(self, robot, occupancy_map, frontiers):
        best_frontier = None
        best_cost = float("inf")

        for frontier in frontiers:
            cost = self._calculate_cost(robot, occupancy_map, frontier)

            if cost < best_cost:
                best_cost = cost
                best_frontier = frontier

        return best_frontier

    def _calculate_cost(self, robot, occupancy_map, frontier):
        fx, fy = frontier

        path = self._wavefront_path(robot, occupancy_map, fx, fy)

        if path is None:
            return float("inf")

        distance_cost = len(path)
        information_gain = self._information_gain(occupancy_map, fx, fy)

        return distance_cost - 0.8 * information_gain

    def _distance_cost(self, robot, fx, fy):
        target_x = fx + 0.5
        target_y = fy + 0.5

        dx = target_x - robot.x
        dy = target_y - robot.y

        return math.sqrt(dx * dx + dy * dy)
    
    def _wavefront_path(self, robot, occupancy_map, goal_x, goal_y):
        start_x, start_y = robot.grid_pos()

        queue = deque()
        visited = set()
        parent = {}

        queue.append((start_x, start_y))
        visited.add((start_x, start_y))
        parent[(start_x, start_y)] = None

        while queue:
            x, y = queue.popleft()

            if (x, y) == (goal_x, goal_y):
                return self._reconstruct_path(parent, (goal_x, goal_y))

            for nx, ny in self._get_neighbors_4(x, y):
                if not occupancy_map.is_inside(nx, ny):
                    continue

                if (nx, ny) in visited:
                    continue

                if occupancy_map.get_cell(nx, ny) != FREE:
                    continue

                visited.add((nx, ny))
                parent[(nx, ny)] = (x, y)
                queue.append((nx, ny))

        return None
    
    def _reconstruct_path(self, parent, goal):
        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = parent[current]

        path.reverse()
        return path
    
    def _get_neighbors_4(self, x, y):
        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]

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
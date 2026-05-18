import math
from collections import deque

from config import FREE, OBSTACLE


class Navigator:
    def __init__(self):
        self.path = None
        self.waypoint = None

        self.arrive_threshold = 0.08

        # stuck detection
        self.last_x = None
        self.last_y = None
        self.stuck_counter = 0
        self.stuck_threshold = 30
        self.move_epsilon = 0.02

    def reset(self):
        self.path = None
        self.waypoint = None
        self.last_x = None
        self.last_y = None
        self.stuck_counter = 0

    def navigate_to(self, robot, occupancy_map, target):
        """
        target: (x, y) grid cell
        return: dx, dy, stuck
        """
        if target is None:
            return 0, 0, False

        self._update_stuck_state(robot)

        if self.stuck_counter >= self.stuck_threshold:
            self.reset()
            return 0, 0, True

        if self.path is None or not self._is_path_valid(occupancy_map):
            self.path = self._build_path(robot, occupancy_map, target)
            self.waypoint = None

        if self.path is None or len(self.path) < 2:
            self.reset()
            return 0, 0, True

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
            return 0, 0, False

        return dx / dist, dy / dist, False

    def _update_stuck_state(self, robot):
        if self.last_x is None:
            self.last_x = robot.x
            self.last_y = robot.y
            return

        moved = math.sqrt(
            (robot.x - self.last_x) ** 2 +
            (robot.y - self.last_y) ** 2
        )

        if moved < self.move_epsilon:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        self.last_x = robot.x
        self.last_y = robot.y

    def _build_path(self, robot, occupancy_map, target):
        start = robot.grid_pos()
        goal = target

        queue = deque([start])
        parent = {start: None}

        while queue:
            x, y = queue.popleft()

            if (x, y) == goal:
                return self._reconstruct_path(parent, goal)

            for nx, ny in self._neighbors_4(x, y):
                if not occupancy_map.is_inside(nx, ny):
                    continue

                if (nx, ny) in parent:
                    continue

                if not self._is_safe_cell(occupancy_map, nx, ny):
                    continue

                parent[(nx, ny)] = (x, y)
                queue.append((nx, ny))

        return None

    def _is_path_valid(self, occupancy_map):
        if self.path is None:
            return False

        for x, y in self.path:
            if not self._is_safe_cell(occupancy_map, x, y):
                return False

        return True

    def _is_safe_cell(self, occupancy_map, x, y, clearance=1):
        if not occupancy_map.is_inside(x, y):
            return False

        if occupancy_map.get_cell(x, y) != FREE:
            return False

        for dy in range(-clearance, clearance + 1):
            for dx in range(-clearance, clearance + 1):
                nx = x + dx
                ny = y + dy

                if not occupancy_map.is_inside(nx, ny):
                    return False

                # 只避開 obstacle，不把 unknown 當成牆
                if occupancy_map.get_cell(nx, ny) == OBSTACLE:
                    return False

        return True

    def _reconstruct_path(self, parent, goal):
        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = parent[current]

        path.reverse()
        return path

    def _neighbors_4(self, x, y):
        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]
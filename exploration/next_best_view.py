import math
from collections import deque

from exploration.base import ExplorationPolicy
from config import FREE, UNKNOWN, OBSTACLE, SENSOR_RADIUS


class NextBestViewPolicy(ExplorationPolicy):
    def __init__(self):
        self.target = None
        self.blacklist = set()

        self.recent_targets = []
        self.recent_target_limit = 8

        self.info_gain_weight = 1.0
        self.distance_weight = 1.0

        self.reached_target_threshold = 0.8

        self.min_unknown_to_continue = 20
        self.min_gain = 8

    def get_action(self, robot, occupancy_map, frontiers, world=None):
        # 如果目前還有 target，就繼續交給 navigator 走
        if self.target is not None:
            tx, ty = self.target

            dist_to_target = math.sqrt(
                (robot.x - (tx + 0.5)) ** 2 +
                (robot.y - (ty + 0.5)) ** 2
            )

            if dist_to_target > self.reached_target_threshold:
                return {
                    "type": "target",
                    "target": self.target
                }

            self.target = None

        if self._unknown_count(occupancy_map) < self.min_unknown_to_continue:
            self.target = None
            return 0, 0

        # 只有需要新 target 時，才做 wavefront + NBV 評估
        distance_map = self._build_wavefront_map(robot, occupancy_map)

        self.target = self._select_best_viewpoint(
            occupancy_map,
            distance_map
        )

        if self.target is not None:
            self.recent_targets.append(self.target)

            if len(self.recent_targets) > self.recent_target_limit:
                self.recent_targets.pop(0)

        if self.target is not None:
            tx, ty = self.target

            distance_map = self._build_wavefront_map(robot, occupancy_map)

            if distance_map[ty][tx] == float("inf"):
                self.blacklist.add(self.target)
                self.target = None

            else:
                dist_to_target = math.sqrt(
                    (robot.x - (tx + 0.5)) ** 2 +
                    (robot.y - (ty + 0.5)) ** 2
                )

                if dist_to_target > self.reached_target_threshold:
                    return {
                        "type": "target",
                        "target": self.target
                    }

                self.target = None

        return {
            "type": "target",
            "target": self.target
        }

    def on_stuck(self, target):
        if target is not None:
            self.blacklist.add(target)

        self.target = None

    def _unknown_count(self, occupancy_map):
        count = 0
        for y in range(occupancy_map.height):
            for x in range(occupancy_map.width):
                if occupancy_map.get_cell(x, y) == UNKNOWN:
                    count += 1
        return count

    def _select_best_viewpoint(self, occupancy_map, distance_map):
        best_cell = None
        best_score = float("-inf")

        for y in range(occupancy_map.height):
            for x in range(occupancy_map.width):
                if (x, y) in self.blacklist:
                    continue

                if (x, y) in self.recent_targets:
                    continue

                if not self._is_safe_cell(occupancy_map, x, y):
                    continue

                distance = distance_map[y][x]

                if distance == float("inf"):
                    continue

                gain = self._estimate_visible_unknown(occupancy_map, x, y)

                if gain < self.min_gain:
                    continue

                score = (
                    self.info_gain_weight * gain
                    - self.distance_weight * distance
                )

                if score > best_score:
                    best_score = score
                    best_cell = (x, y)

        if best_cell is None:
            self.blacklist.clear()

        return best_cell

    def _estimate_visible_unknown(self, occupancy_map, x, y):
        gain = 0

        for dy in range(-SENSOR_RADIUS, SENSOR_RADIUS + 1):
            for dx in range(-SENSOR_RADIUS, SENSOR_RADIUS + 1):
                nx = x + dx
                ny = y + dy

                if not occupancy_map.is_inside(nx, ny):
                    continue

                if math.sqrt(dx * dx + dy * dy) > SENSOR_RADIUS:
                    continue

                if occupancy_map.get_cell(nx, ny) == UNKNOWN:
                    gain += 1

        return gain

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

                # clearance 只避開障礙物，不把 unknown 當成牆
                if occupancy_map.get_cell(nx, ny) == OBSTACLE:
                    return False

        return True

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

                if not self._is_safe_cell(occupancy_map, nx, ny):
                    continue

                if distance_map[ny][nx] != float("inf"):
                    continue

                distance_map[ny][nx] = distance_map[y][x] + 1
                queue.append((nx, ny))

        return distance_map

    def _get_neighbors_4(self, x, y):
        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]
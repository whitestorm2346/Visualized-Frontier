import math
from config import ROBOT_START_X, ROBOT_START_Y, ROBOT_SPEED, ROBOT_RADIUS


class Robot:
    def __init__(self, x=ROBOT_START_X, y=ROBOT_START_Y):
        self.start_x = float(x) + 0.5
        self.start_y = float(y) + 0.5
        self.x = float(x) + 0.5
        self.y = float(y) + 0.5

    def reset(self):
        self.x = self.start_x
        self.y = self.start_y

    def grid_pos(self):
        return int(self.x), int(self.y)

    def try_move(self, dx, dy, world, dt):
        length = math.sqrt(dx * dx + dy * dy)
        if length == 0:
            return

        dx /= length
        dy /= length

        new_x = self.x + dx * ROBOT_SPEED * dt
        new_y = self.y + dy * ROBOT_SPEED * dt

        if not self._collides_with_obstacle(new_x, new_y, world):
            self.x = new_x
            self.y = new_y

    def _collides_with_obstacle(self, x, y, world):
        left = int(math.floor(x - ROBOT_RADIUS))
        right = int(math.floor(x + ROBOT_RADIUS))
        top = int(math.floor(y - ROBOT_RADIUS))
        bottom = int(math.floor(y + ROBOT_RADIUS))

        for cy in range(top, bottom + 1):
            for cx in range(left, right + 1):
                if world.is_obstacle(cx, cy):
                    closest_x = max(cx, min(x, cx + 1))
                    closest_y = max(cy, min(y, cy + 1))

                    dist_x = x - closest_x
                    dist_y = y - closest_y

                    if dist_x * dist_x + dist_y * dist_y < ROBOT_RADIUS * ROBOT_RADIUS:
                        return True

        return False
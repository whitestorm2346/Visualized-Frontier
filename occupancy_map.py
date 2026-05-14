import math
from config import UNKNOWN, FREE, OBSTACLE, SENSOR_RADIUS


class OccupancyMap:
    """Robot 目前知道的地圖。

    UNKNOWN: 還沒看過
    FREE: 已確認可走
    OBSTACLE: 已確認障礙物
    """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[UNKNOWN for _ in range(width)] for _ in range(height)]

    def reset(self):
        self.grid = [[UNKNOWN for _ in range(self.width)] for _ in range(self.height)]

    def is_inside(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def get_cell(self, x, y):
        if not self.is_inside(x, y):
            return OBSTACLE
        return self.grid[y][x]

    def set_cell(self, x, y, value):
        if self.is_inside(x, y):
            self.grid[y][x] = value

    def update_by_sensor(self, world, robot_x, robot_y, radius=SENSOR_RADIUS):
        """用簡化版圓形感測範圍更新 occupancy map。

        這裡沒有做 ray casting，所以可以理解成：
        robot 在半徑範圍內可以直接觀察到真實格子狀態。
        如果你之後想更接近 LiDAR，可以把這裡改成 ray casting。
        """
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx = robot_x + dx
                ny = robot_y + dy

                if not self.is_inside(nx, ny):
                    continue

                if math.sqrt(dx * dx + dy * dy) <= radius:
                    self.grid[ny][nx] = world.get_cell(nx, ny)

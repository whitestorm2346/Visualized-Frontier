import math
from config import UNKNOWN, FREE, OBSTACLE, SENSOR_RADIUS, LIDAR_RAY_COUNT, LIDAR_STEP_SIZE


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
        # robot_x / robot_y 可以是格子座標或連續座標
        origin_x = robot_x
        origin_y = robot_y

        for i in range(LIDAR_RAY_COUNT):
            angle = 2 * math.pi * i / LIDAR_RAY_COUNT
            dx = math.cos(angle)
            dy = math.sin(angle)

            dist = 0

            while dist <= radius:
                x = origin_x + dx * dist
                y = origin_y + dy * dist

                cell_x = int(x)
                cell_y = int(y)

                if not self.is_inside(cell_x, cell_y):
                    break

                world_cell = world.get_cell(cell_x, cell_y)
                self.grid[cell_y][cell_x] = world_cell

                # 打到牆就停止，不繼續看牆後面
                if world.is_obstacle(cell_x, cell_y):
                    break

                dist += LIDAR_STEP_SIZE

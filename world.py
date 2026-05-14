import random
import time
from config import WORLD_WIDTH, WORLD_HEIGHT, FREE, OBSTACLE


class World:
    """真實世界地圖。

    注意：
    這張地圖是固定的 ground truth。
    Robot 不會直接看見整張圖，只能透過感測範圍更新 occupancy map。
    """

    def __init__(self, width=WORLD_WIDTH, height=WORLD_HEIGHT, seed=None):
        self.width = width
        self.height = height
        self.seed = seed
        self.grid = [[FREE for _ in range(width)] for _ in range(height)]
        self._generate_map()

    def regenerate(self):
        self.seed = time.time()
        self.grid = [[FREE for _ in range(self.width)] for _ in range(self.height)]
        self._generate_map()

    def _generate_map(self):
        if self.seed is not None:
            random.seed(self.seed)

        # 地圖邊界全部設為牆壁
        for y in range(self.height):
            for x in range(self.width):
                if x == 0 or y == 0 or x == self.width - 1 or y == self.height - 1:
                    self.grid[y][x] = OBSTACLE

        # 手刻一些牆，讓 frontier 比較明顯
        self._add_rect(18, 8, 4, 22)
        self._add_rect(35, 0, 4, 18)
        self._add_rect(50, 22, 4, 28)
        self._add_rect(10, 38, 30, 4)
        self._add_rect(55, 10, 15, 4)
        self._add_rect(5, 20, 12, 3)

        # 隨機障礙物
        for _ in range(80):
            x = random.randint(2, self.width - 3)
            y = random.randint(2, self.height - 3)
            if (x, y) not in [(8, 8), (9, 8), (8, 9)]:
                self.grid[y][x] = OBSTACLE

    def _add_rect(self, start_x, start_y, w, h):
        for y in range(start_y, min(start_y + h, self.height)):
            for x in range(start_x, min(start_x + w, self.width)):
                self.grid[y][x] = OBSTACLE

    def is_inside(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def is_obstacle(self, x, y):
        if not self.is_inside(x, y):
            return True
        return self.grid[y][x] == OBSTACLE

    def get_cell(self, x, y):
        if not self.is_inside(x, y):
            return OBSTACLE
        return self.grid[y][x]

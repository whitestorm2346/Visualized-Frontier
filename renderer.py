import pygame
from config import (
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    TILE_SIZE,
    UNKNOWN,
    FREE,
    OBSTACLE,
    COLOR_UNKNOWN,
    COLOR_FREE,
    COLOR_OBSTACLE,
    COLOR_FRONTIER,
    COLOR_GRID,
    COLOR_ROBOT,
    COLOR_SENSOR,
    COLOR_TEXT,
    COLOR_BG,
    SENSOR_RADIUS,
)


class Renderer:
    def __init__(self, screen):
        self.screen = screen
        self.font = pygame.font.SysFont("consolas", 18)

    def draw(self, occupancy_map, frontiers, robot):
        self.screen.fill(COLOR_BG)

        camera_x, camera_y = self._get_camera_position(occupancy_map, robot)
        visible_cols = SCREEN_WIDTH // TILE_SIZE + 2
        visible_rows = SCREEN_HEIGHT // TILE_SIZE + 2

        start_x = max(0, camera_x)
        start_y = max(0, camera_y)
        end_x = min(occupancy_map.width, start_x + visible_cols)
        end_y = min(occupancy_map.height, start_y + visible_rows)

        for y in range(start_y, end_y):
            for x in range(start_x, end_x):
                screen_x = (x - camera_x) * TILE_SIZE
                screen_y = (y - camera_y) * TILE_SIZE

                cell = occupancy_map.get_cell(x, y)
                color = self._cell_to_color(cell)

                if (x, y) in frontiers:
                    color = COLOR_FRONTIER

                rect = pygame.Rect(screen_x, screen_y, TILE_SIZE, TILE_SIZE)
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, COLOR_GRID, rect, 1)

        self._draw_sensor_range(robot, camera_x, camera_y)
        self._draw_robot(robot, camera_x, camera_y)
        self._draw_ui(robot, len(frontiers))

        pygame.display.flip()

    def _cell_to_color(self, cell):
        if cell == UNKNOWN:
            return COLOR_UNKNOWN
        if cell == FREE:
            return COLOR_FREE
        if cell == OBSTACLE:
            return COLOR_OBSTACLE
        return COLOR_UNKNOWN

    def _get_camera_position(self, occupancy_map, robot):
        """讓 robot 盡量保持在視窗中心，除非接近地圖邊緣。"""
        visible_cols = SCREEN_WIDTH // TILE_SIZE
        visible_rows = SCREEN_HEIGHT // TILE_SIZE

        camera_x = robot.x - visible_cols // 2
        camera_y = robot.y - visible_rows // 2

        max_camera_x = max(0, occupancy_map.width - visible_cols)
        max_camera_y = max(0, occupancy_map.height - visible_rows)

        camera_x = max(0, min(camera_x, max_camera_x))
        camera_y = max(0, min(camera_y, max_camera_y))

        return camera_x, camera_y

    def _draw_robot(self, robot, camera_x, camera_y):
        center_x = int((robot.x - camera_x + 0.5) * TILE_SIZE)
        center_y = int((robot.y - camera_y + 0.5) * TILE_SIZE)
        radius = TILE_SIZE // 2

        pygame.draw.circle(self.screen, COLOR_ROBOT, (center_x, center_y), radius)

    def _draw_sensor_range(self, robot, camera_x, camera_y):
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)

        center_x = int((robot.x - camera_x + 0.5) * TILE_SIZE)
        center_y = int((robot.y - camera_y + 0.5) * TILE_SIZE)
        radius = SENSOR_RADIUS * TILE_SIZE

        pygame.draw.circle(overlay, COLOR_SENSOR, (center_x, center_y), radius)
        self.screen.blit(overlay, (0, 0))

    def _draw_ui(self, robot, frontier_count):
        lines = [
            f"Robot: ({robot.x}, {robot.y})",
            f"Frontiers: {frontier_count}",
            "Move: WASD / Arrow Keys | Reset: R | Quit: ESC",
        ]

        y = 10
        for line in lines:
            text_surface = self.font.render(line, True, COLOR_TEXT)
            self.screen.blit(text_surface, (10, y))
            y += 22

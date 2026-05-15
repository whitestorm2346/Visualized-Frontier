import pygame
import math
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
    ROBOT_RADIUS,
    LIDAR_RAY_COUNT,
    LIDAR_STEP_SIZE
)


class Renderer:
    def __init__(self, screen):
        self.screen = screen
        self.font = pygame.font.SysFont("consolas", 18)

    def draw(self, occupancy_map, frontiers, robot, world):
        self.screen.fill(COLOR_BG)

        camera_x, camera_y = self._get_camera_position(occupancy_map, robot)
        visible_cols = SCREEN_WIDTH // TILE_SIZE + 2
        visible_rows = SCREEN_HEIGHT // TILE_SIZE + 2

        start_x = max(0, int(camera_x))
        start_y = max(0, int(camera_y))
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

        self._draw_sensor_range(robot, camera_x, camera_y, world)
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
        visible_cols = SCREEN_WIDTH // TILE_SIZE
        visible_rows = SCREEN_HEIGHT // TILE_SIZE

        camera_x = robot.x - visible_cols / 2
        camera_y = robot.y - visible_rows / 2

        max_camera_x = max(0, occupancy_map.width - visible_cols)
        max_camera_y = max(0, occupancy_map.height - visible_rows)

        camera_x = max(0, min(camera_x, max_camera_x))
        camera_y = max(0, min(camera_y, max_camera_y))

        return camera_x, camera_y

    def _draw_robot(self, robot, camera_x, camera_y):
        center_x = int((robot.x - camera_x) * TILE_SIZE)
        center_y = int((robot.y - camera_y) * TILE_SIZE)
        radius = int(ROBOT_RADIUS * TILE_SIZE)

        pygame.draw.circle(self.screen, COLOR_ROBOT, (center_x, center_y), radius)

    def _draw_sensor_range(self, robot, camera_x, camera_y, world):
        origin_x = robot.x
        origin_y = robot.y

        screen_origin_x = int((origin_x - camera_x) * TILE_SIZE)
        screen_origin_y = int((origin_y - camera_y) * TILE_SIZE)

        for i in range(LIDAR_RAY_COUNT):
            angle = 2 * math.pi * i / LIDAR_RAY_COUNT
            dx = math.cos(angle)
            dy = math.sin(angle)

            dist = 0
            end_x = origin_x
            end_y = origin_y

            while dist <= SENSOR_RADIUS:
                x = origin_x + dx * dist
                y = origin_y + dy * dist

                cell_x = int(x)
                cell_y = int(y)

                if not world.is_inside(cell_x, cell_y):
                    break

                end_x = x
                end_y = y

                if world.is_obstacle(cell_x, cell_y):
                    break

                dist += LIDAR_STEP_SIZE

            screen_end_x = int((end_x - camera_x) * TILE_SIZE)
            screen_end_y = int((end_y - camera_y) * TILE_SIZE)

            pygame.draw.line(
                self.screen,
                (255, 60, 60),
                (screen_origin_x, screen_origin_y),
                (screen_end_x, screen_end_y),
                2
            )

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

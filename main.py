import pygame

from config import SCREEN_WIDTH, SCREEN_HEIGHT, FPS
from world import World
from occupancy_map import OccupancyMap
from robot import Robot
from renderer import Renderer
from frontier import detect_frontiers


def handle_movement(keys, robot, world):
    dx, dy = 0, 0

    if keys[pygame.K_w] or keys[pygame.K_UP]:
        dy = -1
    elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
        dy = 1
    elif keys[pygame.K_a] or keys[pygame.K_LEFT]:
        dx = -1
    elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        dx = 1

    if dx != 0 or dy != 0:
        robot.try_move(dx, dy, world)


def reset_simulation(robot, occupancy_map, world):
    world.regenerate()
    robot.reset()

    # 避免新地圖剛好把起點變成障礙物
    while world.is_obstacle(robot.x, robot.y):
        world.regenerate()

    occupancy_map.reset()
    occupancy_map.update_by_sensor(world, robot.x, robot.y)


def main():
    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Frontier-based Occupancy Map Demo")

    clock = pygame.time.Clock()

    world = World()
    occupancy_map = OccupancyMap(world.width, world.height)
    robot = Robot()
    renderer = Renderer(screen)

    occupancy_map.update_by_sensor(world, robot.x, robot.y)

    running = True
    move_cooldown = 0

    while running:
        dt = clock.tick(FPS)
        move_cooldown -= dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    reset_simulation(robot, occupancy_map, world)

        keys = pygame.key.get_pressed()

        # 避免按住按鍵時移動太快
        if move_cooldown <= 0:
            old_pos = (robot.x, robot.y)
            handle_movement(keys, robot, world)

            if (robot.x, robot.y) != old_pos:
                occupancy_map.update_by_sensor(world, robot.x, robot.y)

            move_cooldown = 90

        frontiers = detect_frontiers(occupancy_map)
        renderer.draw(occupancy_map, frontiers, robot)

    pygame.quit()


if __name__ == "__main__":
    main()

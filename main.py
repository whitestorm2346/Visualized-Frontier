import pygame

from config import SCREEN_WIDTH, SCREEN_HEIGHT, FPS
from world import World
from occupancy_map import OccupancyMap
from robot import Robot
from renderer import Renderer
from frontier import detect_frontiers


def handle_movement(keys, robot, world, dt):
    dx, dy = 0, 0

    if keys[pygame.K_w] or keys[pygame.K_UP]:
        dy -= 1
    if keys[pygame.K_s] or keys[pygame.K_DOWN]:
        dy += 1
    if keys[pygame.K_a] or keys[pygame.K_LEFT]:
        dx -= 1
    if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        dx += 1

    robot.try_move(dx, dy, world, dt)


def reset_simulation(robot, occupancy_map, world):
    world.regenerate()
    robot.reset()

    rx, ry = robot.grid_pos()

    while world.is_obstacle(rx, ry):
        world.regenerate()
        rx, ry = robot.grid_pos()

    occupancy_map.reset()
    occupancy_map.update_by_sensor(world, rx, ry)


def main():
    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Frontier-based Occupancy Map Demo")

    clock = pygame.time.Clock()

    world = World()
    occupancy_map = OccupancyMap(world.width, world.height)
    robot = Robot()
    renderer = Renderer(screen)

    rx, ry = robot.grid_pos()
    occupancy_map.update_by_sensor(world, rx, ry)

    running = True

    while running:
        dt = clock.tick(FPS) / 1000

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    reset_simulation(robot, occupancy_map, world)

        keys = pygame.key.get_pressed()

        old_pos = robot.grid_pos()

        handle_movement(keys, robot, world, dt)

        new_pos = robot.grid_pos()
        if new_pos != old_pos:
            occupancy_map.update_by_sensor(world, new_pos[0], new_pos[1])

        frontiers = detect_frontiers(occupancy_map)
        renderer.draw(occupancy_map, frontiers, robot)

    pygame.quit()


if __name__ == "__main__":
    main()

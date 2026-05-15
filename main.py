import pygame
import argparse

from config import SCREEN_WIDTH, SCREEN_HEIGHT, FPS
from exploration.lpfe import LPFEPolicy
from world import World
from occupancy_map import OccupancyMap
from robot import Robot
from renderer import Renderer
from frontier import detect_frontiers

from exploration.manual import ManualPolicy
from exploration.random_walk import RandomWalkPolicy


def reset_simulation(robot, occupancy_map, world):
    world.regenerate()
    robot.reset()

    rx, ry = robot.grid_pos()

    while world.is_obstacle(rx, ry):
        world.regenerate()
        rx, ry = robot.grid_pos()

    occupancy_map.reset()
    occupancy_map.update_by_sensor(world, robot.x, robot.y)


def create_policy(policy_name):
    if policy_name == "manual":
        return ManualPolicy()

    if policy_name == "random":
        return RandomWalkPolicy()
    
    if policy_name == "lpfe":
        return LPFEPolicy()

    raise ValueError(f"Unknown policy: {policy_name}")


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-p",
        "--policy",
        choices=["manual", "random", "lpfe"],
        default="manual",
        help="Choose exploration policy"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Frontier-based Occupancy Map Demo")

    clock = pygame.time.Clock()

    world = World()
    occupancy_map = OccupancyMap(world.width, world.height)
    robot = Robot()
    renderer = Renderer(screen)

    policy = create_policy(args.policy)

    occupancy_map.update_by_sensor(world, robot.x, robot.y)

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
        frontiers = detect_frontiers(occupancy_map)

        if hasattr(policy, "update_keys"):
            policy.update_keys(keys)

        dx, dy = policy.get_action(robot, occupancy_map, frontiers, world)

        old_pos = robot.grid_pos()
        robot.try_move(dx, dy, world, dt)

        new_pos = robot.grid_pos()
        if new_pos != old_pos:
            occupancy_map.update_by_sensor(world, robot.x, robot.y)

        renderer.draw(occupancy_map, frontiers, robot, world)

    pygame.quit()


if __name__ == "__main__":
    main()

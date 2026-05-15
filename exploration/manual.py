import pygame
from exploration.base import ExplorationPolicy


class ManualPolicy(ExplorationPolicy):
    def __init__(self):
        self.keys = None

    def update_keys(self, keys):
        self.keys = keys

    def get_action(self, robot, occupancy_map, frontiers, world=None):
        dx, dy = 0, 0

        if self.keys is None:
            return dx, dy

        if self.keys[pygame.K_w] or self.keys[pygame.K_UP]:
            dy -= 1
        if self.keys[pygame.K_s] or self.keys[pygame.K_DOWN]:
            dy += 1
        if self.keys[pygame.K_a] or self.keys[pygame.K_LEFT]:
            dx -= 1
        if self.keys[pygame.K_d] or self.keys[pygame.K_RIGHT]:
            dx += 1

        return dx, dy
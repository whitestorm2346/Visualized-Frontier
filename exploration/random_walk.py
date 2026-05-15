import random

from exploration.base import ExplorationPolicy


class RandomWalkPolicy(ExplorationPolicy):
    def __init__(self):
        self.dx = 0
        self.dy = 0
        self.timer = 0

    def get_action(self, robot, occupancy_map, frontiers, world=None):
        self.timer -= 1

        if self.timer <= 0:
            self.dx, self.dy = random.choice([
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ])

            self.timer = 40

        return self.dx, self.dy
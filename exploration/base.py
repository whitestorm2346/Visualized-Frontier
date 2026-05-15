from abc import ABC, abstractmethod


class ExplorationPolicy(ABC):
    @abstractmethod
    def get_action(self, robot, occupancy_map, frontiers, world=None):
        pass
from config import ROBOT_START_X, ROBOT_START_Y


class Robot:
    def __init__(self, x=ROBOT_START_X, y=ROBOT_START_Y):
        self.start_x = x
        self.start_y = y
        self.x = x
        self.y = y

    def reset(self):
        self.x = self.start_x
        self.y = self.start_y

    def try_move(self, dx, dy, world):
        new_x = self.x + dx
        new_y = self.y + dy

        # robot 移動判斷可以使用真實地圖
        # 因為這代表物理世界中真的撞到牆就不能過去
        if not world.is_obstacle(new_x, new_y):
            self.x = new_x
            self.y = new_y

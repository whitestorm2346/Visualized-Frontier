from config import UNKNOWN, FREE


def detect_frontiers(occupancy_map):
    """找出 frontier cells。

    Frontier 定義：
    一個已知 free cell，只要它的上下左右鄰居至少有一格是 unknown，
    就把它視為 frontier。

    這代表 robot 已經知道自己可以站在這裡，
    而這裡旁邊還有尚未探索區域。
    """
    frontiers = set()

    for y in range(occupancy_map.height):
        for x in range(occupancy_map.width):
            if occupancy_map.get_cell(x, y) != FREE:
                continue

            for nx, ny in get_4_neighbors(x, y):
                if occupancy_map.is_inside(nx, ny) and occupancy_map.get_cell(nx, ny) == UNKNOWN:
                    frontiers.add((x, y))
                    break

    return frontiers


def get_4_neighbors(x, y):
    return [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),
    ]

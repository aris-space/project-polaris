from pathlib import Path
import math
import yaml
from grid_generation import generate_grid

PARAMS_FILE = Path(__file__).parent / "params.yaml"

with open(PARAMS_FILE) as f:
    params = yaml.safe_load(f)

GRID_STARTING_POSITION = tuple(params["grid"]["starting_position"])


def dist(current_point, next_point):
    """Euclidean distance between two (x, y) points."""
    dx = next_point[0] - current_point[0]
    dy = next_point[1] - current_point[1]
    return math.hypot(dx, dy)


grid_points_list = generate_grid()
waypoints_list = [GRID_STARTING_POSITION] + grid_points_list

def traversal_order_nearest_n(points,start):
    grid_traversal_order, current = [], start
    remaining = set(range(len(points)))

    while remaining:
        nxt = min(remaining, key=lambda i: dist(current, points[i]))
        grid_traversal_order.append(nxt); remaining.remove(nxt); current = points[nxt]

    return grid_traversal_order


if __name__ == "__main__":
    print(waypoints_list)
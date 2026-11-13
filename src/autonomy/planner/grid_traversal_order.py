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

def generate_order_nearest_n(points,start):
    grid_traversal_order, current = [], start
    remaining = set(range(len(points)))

    while remaining:
        nxt = min(remaining, key=lambda i: dist(current, points[i]))
        grid_traversal_order.append(nxt); remaining.remove(nxt); current = points[nxt]

    return grid_traversal_order

def generate_order_two_opt(points, start, eps=1e-9):
    """Improve the nearest neighbour order with 2-opt.

    Repeatedly reverses a section of the path whenever that makes the path
    shorter, until no reversal helps anymore. The start never moves, the
    end of the path is free (there is no fixed end point).
    """
    # Path as indices into points, with None standing for the start
    path = [None] + generate_order_nearest_n(points, start)
    pos = lambda k: start if k is None else points[k]
    n = len(path)

    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                A, B = pos(path[i - 1]), pos(path[i])
                C = pos(path[j])
                if j + 1 < n:
                    D = pos(path[j + 1])
                    delta = dist(A, C) + dist(B, D) - dist(A, B) - dist(C, D)
                else:
                    # Reversing up to the end: only the edge A-B changes to A-C
                    delta = dist(A, C) - dist(A, B)

                if delta < -eps:  # eps avoids looping on float noise
                    path[i:j + 1] = reversed(path[i:j + 1])
                    improved = True

    return path[1:]


if __name__ == "__main__":
    print(waypoints_list)
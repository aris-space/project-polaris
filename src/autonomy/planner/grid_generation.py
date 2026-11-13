from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon
import numpy as np
import yaml


PARAMS_FILE = Path(__file__).parent / "params.yaml"

with open(PARAMS_FILE) as f:
    params = yaml.safe_load(f)

GRID_SIZE = params["grid"]["size"]
GRID_RESOLUTION = params["grid"]["resolution"]
GRID_ROTATION = np.deg2rad(params["grid"]["rotation"])
GRID_BOUNDARY_COORDINATES = [tuple(c) for c in params["grid"]["boundary_coordinates"]]
GRID_STARTING_POSITION = tuple(params["grid"]["starting_position"])
GRID_START_OFFSET = params["grid"]["start_offset"]
DRIVEOUT_DISTANCE = params["driveout"]["distance"]
DRIVEOUT_DIRECTION = np.deg2rad(params["driveout"]["direction"])


RESTRICTED_COLOR = (1, 0, 0, 0.2)
PATH_COLOR = "tab:green"


def create_map(grid_list=None, title="Survey Grid"):
    """Draw only the map: boundary, restricted area, start and (optional) grid points."""
    fig, ax = plt.subplots(figsize=(9, 6))

    # Area outside the boundary: transparent red background
    ax.set_facecolor(RESTRICTED_COLOR)

    # Area inside the boundary: white, with a solid red line at the interface
    boundary = Polygon(GRID_BOUNDARY_COORDINATES, closed=True,
                       facecolor="white", edgecolor="red", linewidth=2)
    ax.add_patch(boundary)

    # Grid points (optional)
    if grid_list:
        xs, ys = zip(*grid_list)
        ax.scatter(xs, ys, s=10, color="black", zorder=2, label="Grid points")

    # Starting position as cross
    ax.plot(*GRID_STARTING_POSITION, marker="x", color="blue",
            markersize=12, markeredgewidth=2, linestyle="none", zorder=3, label="Start")

    # Leave a margin around the boundary so the outside region is visible
    xs_b, ys_b = zip(*GRID_BOUNDARY_COORDINATES)
    margin = 0.1 * max(max(xs_b) - min(xs_b), max(ys_b) - min(ys_b))
    ax.set_xlim(min(xs_b) - margin, max(xs_b) + margin)
    ax.set_ylim(min(ys_b) - margin, max(ys_b) + margin)

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)

    _add_legend(fig, ax)

    return fig, ax


def create_map_with_path(grid_list, traversal_order,
                         title="Grid Traversal with Nearest Neighbour Algorithm"):
    """Draw the map plus the path through grid_list in traversal_order, as arrows."""
    fig, ax = create_map(grid_list, title=title)

    # One arrow per segment
    path = generate_path(grid_list, traversal_order)
    for start, end in zip(path[:-1], path[1:]):
        ax.annotate("", xy=end, xytext=start, zorder=1,
                    arrowprops=dict(arrowstyle="->", color=PATH_COLOR, lw=1.5))

    # Arrows have no legend entry by themselves, so add a proxy line
    _add_legend(fig, ax, [Line2D([], [], color=PATH_COLOR, lw=1.5, marker=">", label="Path")])

    return fig, ax


def _add_legend(fig, ax, extra_handles=()):
    # Background colour has no legend entry by itself, so add a proxy patch
    handles, _ = ax.get_legend_handles_labels()
    handles += list(extra_handles)
    handles.append(Patch(facecolor=RESTRICTED_COLOR, edgecolor="red", label="Restricted area"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1))
    fig.tight_layout()


def generate_grid(starting_position=GRID_STARTING_POSITION,
                  driveout_distance=DRIVEOUT_DISTANCE,
                  driveout_direction=DRIVEOUT_DIRECTION,
                  grid_size=GRID_SIZE,
                  grid_resolution=GRID_RESOLUTION,
                  grid_rotation=GRID_ROTATION,
                  start_offset=GRID_START_OFFSET):
    """Return a list of (x, y) grid points.

    The grid lies in front of starting_position, in driveout_direction.
    It is placed so that the closest grid point is exactly driveout_distance
    away from the start (for a 45 deg rotated grid that is its near corner).
    grid_rotation rotates the grid around its own center, measured from the
    x-axis. start_offset (-10 to 10) sets where the start sits sideways,
    seen along the driveout direction: -10 = in line with the leftmost grid
    point, 0 = in line with the center, 10 = in line with the rightmost one.
    Both angles are in radians, counter-clockwise.
    """
    if not -10 <= start_offset <= 10:
        raise ValueError(f"start_offset must be between -10 and 10, got {start_offset}")

    # Driveout direction (forward) and the direction to its right
    forward = np.array([np.cos(driveout_direction), np.sin(driveout_direction)])
    right = np.array([np.sin(driveout_direction), -np.cos(driveout_direction)])

    # Grid axes, rotated by grid_rotation
    x_axis = np.array([np.cos(grid_rotation), np.sin(grid_rotation)])
    y_axis = np.array([-np.sin(grid_rotation), np.cos(grid_rotation)])

    # Grid points relative to the grid center
    n_points = int(grid_size // grid_resolution) + 1
    offsets = np.arange(n_points) * grid_resolution - grid_size / 2
    local_points = [u * x_axis + v * y_axis for v in offsets for u in offsets]

    # Each point's position relative to the center: ahead (a) and to the right (b)
    a = np.array([p @ forward for p in local_points])
    b = np.array([p @ right for p in local_points])

    # Sideways: put the center so that the start sits at start_offset
    half_width = np.max(np.abs(b))
    center_right = -start_offset / 10 * half_width

    # Forward: smallest center distance for which every grid point is at least
    # driveout_distance from the start, so the closest one is exactly that far
    lateral = center_right + b
    reachable = np.abs(lateral) <= driveout_distance
    if reachable.any():
        center_ahead = np.max(-a[reachable] + np.sqrt(driveout_distance**2 - lateral[reachable]**2))
    else:
        # All points are further sideways than driveout_distance: start the grid level with the start
        center_ahead = np.max(-a)

    center = np.array(starting_position) + center_ahead * forward + center_right * right

    grid_list = []
    for p in local_points:
        x, y = center + p
        grid_list.append((float(x), float(y)))

    return grid_list


def generate_path(grid_list, traversal_order, starting_position=GRID_STARTING_POSITION):
    """Return the path as a list of (x, y) points: the starting position
    followed by the grid points in traversal order (indices into grid_list)."""
    return [starting_position] + [grid_list[i] for i in traversal_order]


if __name__ == "__main__":
    from grid_traversal_order import generate_order_nearest_n, generate_order_two_opt

    grid_list = generate_grid()
    traversal_order = generate_order_two_opt(grid_list, GRID_STARTING_POSITION)

    create_map_with_path(grid_list, traversal_order)
    plt.show()

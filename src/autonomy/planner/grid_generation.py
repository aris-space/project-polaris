import matplotlib.pyplot as plt
from matplotlib.patches import Patch, Polygon
import numpy as np



GRID_SIZE = 100
GRID_RESOLUTION = 10
GRID_BOUNDARY_COORDINATES = [(0,0), (0,200), (200,200), (200,0)]
GRID_STARTING_POSITION = (100,50)
DRIVEOUT_DISTANCE = 20
DRIVEOUT_DIRECTION = np.pi/2


def create_map(grid_list=None, title="Survey Grid"):
    fig, ax = plt.subplots(figsize=(9, 6))

    # Area outside the boundary: transparent red background
    restricted_color = (1, 0, 0, 0.2)
    ax.set_facecolor(restricted_color)

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

    # Background colour has no legend entry by itself, so add a proxy patch
    handles, _ = ax.get_legend_handles_labels()
    handles.append(Patch(facecolor=restricted_color, edgecolor="red", label="Restricted area"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1))
    fig.tight_layout()

    return fig, ax

def generate_grid(starting_position=GRID_STARTING_POSITION,
                  driveout_distance=DRIVEOUT_DISTANCE,
                  driveout_direction=DRIVEOUT_DIRECTION,
                  grid_size=GRID_SIZE,
                  grid_resolution=GRID_RESOLUTION):
    """Return a list of (x, y) grid points.

    The grid starts at the end of the driveout (starting_position moved by
    driveout_distance in driveout_direction) and is aligned with the driveout
    direction: it extends grid_size forward and is centered left/right.
    driveout_direction is in radians, measured from the x-axis.
    """
    # End point of the driveout = grid origin
    origin_x = starting_position[0] + driveout_distance * np.cos(driveout_direction)
    origin_y = starting_position[1] + driveout_distance * np.sin(driveout_direction)

    # Unit vectors along the driveout direction (forward) and perpendicular to it (lateral)
    forward = np.array([np.cos(driveout_direction), np.sin(driveout_direction)])
    lateral = np.array([-np.sin(driveout_direction), np.cos(driveout_direction)])

    # Local grid coordinates
    n_points = int(grid_size // grid_resolution) + 1
    along = np.arange(n_points) * grid_resolution
    across = np.arange(n_points) * grid_resolution - grid_size / 2

    grid_list = []
    for a in along:
        for c in across:
            x, y = np.array([origin_x, origin_y]) + a * forward + c * lateral
            grid_list.append((float(x), float(y)))

    return grid_list


if __name__ == "__main__":
    create_map(generate_grid())
    plt.show()

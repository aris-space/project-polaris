import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
from matplotlib.patches import FancyArrowPatch, Circle
from matplotlib.lines import Line2D

WAYPOINTS = [(11.5, 0.0), (11.5, -11.5), (0.0, -11.5), (0.0, 0.0)]
WAYPOINT_RADIUS = 0.8
WAYPOINT_COLOR = 'orange'


def draw_waypoint_markers(ax):
    for (x, y) in WAYPOINTS:
        ax.plot(x, y, marker='+', color=WAYPOINT_COLOR, markersize=14,
                markeredgewidth=2.5, linestyle='None', zorder=7)
        ax.add_patch(Circle((x, y), WAYPOINT_RADIUS, fill=False,
                            edgecolor=WAYPOINT_COLOR, linewidth=1.8, zorder=7))

# 1. Matplotlib Configuration (Standard Fonts, No LaTeX)
plt.rcParams.update({
    "text.usetex": False,
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial", "DejaVu Sans"],
    "axes.titlesize": 14,
    "axes.labelsize": 12,
    "font.size": 12,
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10
})


def load_dataset(csv_file_path, lookahead_radius, title):
    df = pd.read_csv(csv_file_path)
    df.ffill(inplace=True)
    df.fillna(0, inplace=True)

    t0 = df['time_sec'].iloc[0]
    t = (df['time_sec'] - t0).to_numpy()

    rx = df['robot_x_m'].to_numpy()
    ry = df['robot_y_m'].to_numpy()

    qx = df['robot_qx'].to_numpy()
    qy = df['robot_qy'].to_numpy()
    qz = df['robot_qz'].to_numpy()
    qw = df['robot_qw'].to_numpy()
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))

    path_xy = df[['closest_x_m', 'closest_y_m']]
    path_mask = (path_xy != path_xy.shift()).any(axis=1)
    path_xy = path_xy[path_mask]
    path_x = path_xy['closest_x_m'].to_numpy()
    path_y = path_xy['closest_y_m'].to_numpy()
    path_diff = np.hypot(np.diff(path_x), np.diff(path_y))
    path_s = np.concatenate([[0.0], np.cumsum(path_diff)])

    return {
        'title': title,
        'lookahead': lookahead_radius,
        'csv_path': csv_file_path,
        't': t,
        'rx': rx, 'ry': ry, 'yaw': yaw,
        'path_x': path_x, 'path_y': path_y, 'path_s': path_s,
    }


def plot_static_xy(datasets, save_path, t_max_global, x_lim, y_lim):
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[20, 2],
                          hspace=0.4, wspace=0.25)
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])]
    ax_legend = fig.add_subplot(gs[1, :])
    ax_legend.axis('off')

    for ax, d in zip(axes, datasets):
        ax.plot(d['path_x'], d['path_y'], '--', color='black',
                linewidth=1.5, label='Reference path')

        rx, ry, t = d['rx'], d['ry'], d['t']
        points = np.array([rx, ry]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap='viridis',
                            norm=plt.Normalize(0, t_max_global), linewidth=2)
        lc.set_array(t[:-1])
        ax.add_collection(lc)

        ax.plot(rx[0], ry[0], 'o', color='green', markersize=10,
                markeredgecolor='black', label='Start', zorder=5)
        ax.plot(rx[-1], ry[-1], 's', color='red', markersize=10,
                markeredgecolor='black', label='End', zorder=5)

        draw_waypoint_markers(ax)

        ax.set_xlabel('East / X [m]')
        ax.set_ylabel('North / Y [m]')
        ax.set_title(d['title'])
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(*x_lim)
        ax.set_ylim(*y_lim)
        ax.xaxis.set_major_locator(plt.MultipleLocator(2))
        ax.yaxis.set_major_locator(plt.MultipleLocator(2))
        ax.grid(True, linestyle='--', alpha=0.6)

    wp_proxy = Line2D([0], [0], marker='+', color=WAYPOINT_COLOR,
                      markersize=10, markeredgewidth=2.5,
                      linestyle='None', label='Predefined waypoints')
    handles, labels = axes[0].get_legend_handles_labels()
    ax_legend.legend(handles + [wp_proxy], labels + ['Predefined waypoints'],
                     loc='center', ncol=len(labels) + 1,
                     fontsize=14, markerscale=1.4,
                     handlelength=2.5, columnspacing=1.8,
                     frameon=True)

    fig.suptitle('X-Y Plane View (ROS ENU) — No current vs. With current (0.08 m/s Southwest, σ=0.1 m/s)',
                 fontsize=15)
    fig.savefig(save_path, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {save_path}")


def animate_xy(datasets, save_path, t_max_global, x_lim, y_lim,
               n_frames=500, target_duration_s=50.0):
    frame_times = np.linspace(0, t_max_global, n_frames)

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[20, 2],
                          hspace=0.4, wspace=0.25)
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])]
    ax_legend = fig.add_subplot(gs[1, :])
    ax_legend.axis('off')

    norm = plt.Normalize(0, t_max_global)

    panel_artists = []
    for ax, d in zip(axes, datasets):
        leg_end_idx = []
        for (wx, wy) in WAYPOINTS:
            d2_wp = (d['path_x'] - wx) ** 2 + (d['path_y'] - wy) ** 2
            leg_end_idx.append(int(np.argmin(d2_wp)))
        leg_end_idx = sorted(set(leg_end_idx))
        if leg_end_idx[-1] < len(d['path_x']) - 1:
            leg_end_idx.append(len(d['path_x']) - 1)
        d['leg_end_idx'] = leg_end_idx

        ref_line, = ax.plot([], [], '--', color='black',
                            linewidth=1.5, label='Reference path')
        ax.plot(d['rx'][0], d['ry'][0], 'o', color='green', markersize=10,
                markeredgecolor='black', label='Start', zorder=5)
        ax.plot(d['rx'][-1], d['ry'][-1], 's', color='red', markersize=10,
                markeredgecolor='black', label='End', zorder=5)

        draw_waypoint_markers(ax)

        ax.set_xlabel('East / X [m]')
        ax.set_ylabel('North / Y [m]')
        ax.set_title(d['title'])
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(*x_lim)
        ax.set_ylim(*y_lim)
        ax.xaxis.set_major_locator(plt.MultipleLocator(2))
        ax.yaxis.set_major_locator(plt.MultipleLocator(2))
        ax.grid(True, linestyle='--', alpha=0.6)

        trail = LineCollection([], cmap='viridis', norm=norm, linewidth=2)
        ax.add_collection(trail)
        robot_dot, = ax.plot([], [], 'o', color='white',
                             markeredgecolor='black', markersize=9, zorder=6)
        yaw_stick, = ax.plot([], [], '-', color='#ff6f00', linewidth=3.0,
                             zorder=6, label='Heading (yaw)')
        lookahead_arrow = FancyArrowPatch((0, 0), (0, 0),
                                          arrowstyle='-|>',
                                          mutation_scale=8,
                                          color='#444444', linewidth=1.4,
                                          zorder=5)
        ax.add_patch(lookahead_arrow)

        panel_artists.append({
            'ref_line': ref_line,
            'trail': trail,
            'robot_dot': robot_dot,
            'yaw_stick': yaw_stick,
            'lookahead_arrow': lookahead_arrow,
            'stick_len': 1.15,
        })

    arrow_proxy = Line2D([0], [0], color='#444444', linewidth=1.4,
                         marker='>', markersize=7,
                         label='Lookahead point')
    wp_proxy = Line2D([0], [0], marker='+', color=WAYPOINT_COLOR,
                      markersize=10, markeredgewidth=2.5,
                      linestyle='None', label='Predefined waypoints')
    handles, labels = axes[0].get_legend_handles_labels()
    extra = [arrow_proxy, wp_proxy]
    extra_labels = ['Lookahead point', 'Predefined waypoints']
    ax_legend.legend(handles + extra, labels + extra_labels,
                     loc='center', ncol=len(labels) + len(extra),
                     fontsize=14, markerscale=1.4,
                     handlelength=2.5, columnspacing=1.8,
                     frameon=True)


    def update(frame_i):
        try:
            cur_t = frame_times[frame_i]
            out = []
            for d, pa in zip(datasets, panel_artists):
                t = d['t']
                cur_t_d = min(cur_t, t[-1])
                cur_end = int(np.searchsorted(t, cur_t_d, side='right'))
                cur_end = max(cur_end, 1)

                pts = np.array([d['rx'][:cur_end],
                                d['ry'][:cur_end]]).T.reshape(-1, 1, 2)
                if len(pts) >= 2:
                    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
                    pa['trail'].set_segments(segs)
                    pa['trail'].set_array(t[:cur_end - 1])

                i_now = cur_end - 1
                x, y, psi = d['rx'][i_now], d['ry'][i_now], d['yaw'][i_now]
                pa['robot_dot'].set_data([x], [y])
                sl = pa['stick_len']
                pa['yaw_stick'].set_data([x, x + sl * np.cos(psi)],
                                         [y, y + sl * np.sin(psi)])

                d2 = (d['path_x'] - x) ** 2 + (d['path_y'] - y) ** 2
                i_near = int(np.argmin(d2))
                leg_end = next((e for e in d['leg_end_idx'] if e >= i_near),
                               d['leg_end_idx'][-1])
                target_s = d['path_s'][i_near] + d['lookahead']
                i_la = int(np.searchsorted(d['path_s'], target_s))
                i_la = min(i_la, leg_end)
                lx, ly = d['path_x'][i_la], d['path_y'][i_la]
                pa['lookahead_arrow'].set_positions((x, y), (lx, ly))

                ref_stop = leg_end + 1
                pa['ref_line'].set_data(d['path_x'][:ref_stop],
                                        d['path_y'][:ref_stop])

                out.extend([pa['ref_line'], pa['trail'], pa['robot_dot'],
                            pa['yaw_stick'], pa['lookahead_arrow']])

            return out
        except Exception as e:
            import traceback
            print(f"[update frame {frame_i}] ERROR: {type(e).__name__}: {e}")
            traceback.print_exc()
            raise

    fig.suptitle('X-Y Plane Animation (ROS ENU) — No current vs. With current (0.08 m/s SW, σ=0.1 m/s)',
                 fontsize=15)

    anim = FuncAnimation(fig, update, frames=n_frames,
                         interval=50, blit=False)
    fps = max(1, round(n_frames / target_duration_s))

    last_err = None
    for writer_name, make_writer, ext in [
        ('PillowWriter', lambda: PillowWriter(fps=fps), '.gif'),
        ('FFMpegWriter', lambda: FFMpegWriter(fps=fps), '.mp4'),
    ]:
        try:
            target = os.path.splitext(save_path)[0] + ext
            print(f"[animate_xy] trying {writer_name} -> {target}")
            anim.save(target, writer=make_writer())
            print(f"Animation saved: {target}")
            last_err = None
            break
        except Exception as e:
            import traceback
            print(f"[animate_xy] {writer_name} failed: {type(e).__name__}: {e}")
            traceback.print_exc()
            last_err = e

    plt.close(fig)
    if last_err is not None:
        raise last_err


if __name__ == "__main__":
    datasets = [
        load_dataset(
            r"C:\Users\pzzsc\Documents\FP_official_Programming\Other things\Autonomy_analyzation\Autonomy_analyzation\orca_tracking_csv_export_Autonomy_grid_no_current_01_very_good\tracking_errors_wide.csv",
            #"/home/polaris_pz/Autonomy_analyzation/orca_tracking_csv_export_Autonomy_grid_no_current_01_very_good/tracking_errors_wide.csv",
            lookahead_radius=2.5,
            title="No current",
        ),
        load_dataset(
            r"C:\Users\pzzsc\Documents\FP_official_Programming\Other things\Autonomy_analyzation\Autonomy_analyzation\orca_tracking_csv_export_Autonomy_grid_yes_current_very_good\tracking_errors_wide.csv",
            #"/home/polaris_pz/Autonomy_analyzation/orca_tracking_csv_export_Autonomy_grid_yes_current_very_good/tracking_errors_wide.csv",
            lookahead_radius=2.5,
            title="With current",
        ),
    ]

    t_max_global = max(d['t'][-1] for d in datasets)
    x_min = min(min(d['rx'].min(), d['path_x'].min()) for d in datasets) - 1
    x_max = max(max(d['rx'].max(), d['path_x'].max()) for d in datasets) + 1
    y_min = min(min(d['ry'].min(), d['path_y'].min()) for d in datasets) - 1
    y_max = max(max(d['ry'].max(), d['path_y'].max()) for d in datasets) + 1
    x_lim = (x_min, x_max)
    y_lim = (y_min, y_max)

    out_dir = r"C:\Users\pzzsc\Documents\FP_official_Programming\Other things\Autonomy_analyzation\Autonomy_analyzation"
    plot_static_xy(datasets,
                   os.path.join(out_dir, 'xy_view_square_two_plots.png'),
                   t_max_global, x_lim, y_lim)
    animate_xy(datasets,
               os.path.join(out_dir, 'xy_animation_square_two_plots.gif'),
               t_max_global, x_lim, y_lim)

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 1. Matplotlib Configuration (LaTeX rendering, serif/Palatino via mathpazo)
# Requires a working LaTeX install. On Linux:
#   sudo apt install texlive-latex-extra texlive-fonts-recommended dvipng cm-super
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "text.latex.preamble": r"\usepackage{mathpazo}\usepackage{amsmath}",
    "axes.titlesize": 17,
    "axes.labelsize": 16,
    "font.size": 16,
    "legend.fontsize": 13,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10
})

def plot_data(csv_file_path, start_time=None, end_time=None, foxglove_offset=0.0):
    """
    start_time, end_time: elapsed seconds (relative to first CSV row) to crop the data.
        Pass None to use the natural start/end.
    foxglove_offset: seconds between Foxglove's "Elapsed" (bag start) and the CSV's
        first sample. If your Foxglove markers are e.g. 6.57s and 42.81s and the CSV
        starts 0.3s after the bag, set foxglove_offset=0.3 and pass start_time=6.57,
        end_time=42.81 in Foxglove units.
    """
    out_dir = os.path.dirname(os.path.abspath(csv_file_path))

    # 2. Data Loading
    df = pd.read_csv(csv_file_path)

    # Fill missing values to ensure continuous lines
    df.ffill(inplace=True)
    df.fillna(0, inplace=True)

    t0 = df['time_sec'].iloc[0]
    df = df.assign(_elapsed=df['time_sec'] - t0)

    # Crop to [start_time, end_time] in CSV-elapsed seconds. Foxglove markers can
    # be passed directly by giving foxglove_offset = (csv_first_sample - bag_start).
    if start_time is not None:
        df = df[df['_elapsed'] >= (start_time - foxglove_offset)]
    if end_time is not None:
        df = df[df['_elapsed'] <= (end_time - foxglove_offset)]
    df = df.reset_index(drop=True)
    if df.empty:
        raise ValueError(f"No rows in [{start_time}, {end_time}] s — check window/offset.")

    # Re-base time so plots start at 0 within the cropped window.
    t0 = df['time_sec'].iloc[0]
    time = df['time_sec'] - t0

    # # -------------------------------------------------------------------
    # # GROUP 1: TRACKING ERRORS
    # # Using exact names: cross_track_xy_m, vertical_error_m, yaw_error_rad
    # # -------------------------------------------------------------------
    # fig1, axes1 = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
    
    # axes1[0].plot(time, df['cross_track_xy_m'], color='#1f77b4', label=r'Cross-track error')
    # axes1[0].set_ylabel(r'XY error [\textrm{m}]')
    # axes1[0].set_title(r'\textbf{Controller tracking errors}')

    # axes1[1].plot(time, df['vertical_error_m'], color='#2ca02c', label=r'Vertical error')
    # axes1[1].set_ylabel(r'Vertical error [\textrm{m}]')

    # axes1[2].plot(time, df['yaw_error_rad'], color='#d62728', label=r'Yaw error')
    # axes1[2].set_ylabel(r'Yaw error [\textrm{rad}]')
    # axes1[2].set_xlabel(r'Time [\textrm{s}]')

    # for ax in axes1:
    #     ax.grid(True, linestyle='--', alpha=0.6)
    #     ax.legend(loc='upper right')
    
    # fig1.tight_layout()
    # fig1.savefig(os.path.join(out_dir, 'tracking_errors.png'))

    # # -------------------------------------------------------------------
    # # GROUP 2: POSE COMPARISON (Robot vs. Path)
    # # Using exact names: robot_x_m, closest_x_m, etc.
    # # -------------------------------------------------------------------
    # fig2, axes2 = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
    
    # # X Position
    # axes2[0].plot(time, df['robot_x_m'], label=r'Robot $x$')
    # axes2[0].plot(time, df['closest_x_m'], '--', label=r'Path $x$')
    # axes2[0].set_ylabel(r'$X$ [\textrm{m}]')
    # axes2[0].set_title(r'\textbf{Robot position vs. closest path point}')

    # # Y Position
    # axes2[1].plot(time, df['robot_y_m'], color='orange', label=r'Robot $y$')
    # axes2[1].plot(time, df['closest_y_m'], '--', color='brown', label=r'Path $y$')
    # axes2[1].set_ylabel(r'$Y$ [\textrm{m}]')

    # # Z Position
    # axes2[2].plot(time, df['robot_z_m'], color='purple', label=r'Robot $z$')
    # axes2[2].plot(time, df['closest_z_m'], '--', color='black', label=r'Path $z$')
    # axes2[2].set_ylabel(r'$Z$ [\textrm{m}]')
    # axes2[2].set_xlabel(r'Time [\textrm{s}]')

    # for ax in axes2:
    #     ax.grid(True, linestyle='--', alpha=0.6)
    #     ax.legend(loc='upper right')

    # fig2.tight_layout()
    # fig2.savefig(os.path.join(out_dir, 'pose_comparison.png'))

    # # -------------------------------------------------------------------
    # # GROUP 3: ROBOT TWISTS (Velocities)
    # # Using exact names: twist_linear_x, twist_angular_z, etc.
    # # -------------------------------------------------------------------
    # fig3, axes3 = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    
    # axes3[0].plot(time, df['twist_linear_x'], label=r'$v_x$ (forward)')
    # axes3[0].plot(time, df['twist_linear_y'], label=r'$v_y$ (strafe)')
    # axes3[0].plot(time, df['twist_linear_z'], label=r'$v_z$ (vertical)')
    # axes3[0].set_ylabel(r'Linear vel.\ [\textrm{m/s}]')
    # axes3[0].set_title(r'\textbf{Robot body twists}')

    # axes3[1].plot(time, df['twist_angular_z'], color='red', label=r'$\omega_z$ (yaw)')
    # axes3[1].set_ylabel(r'Angular vel.\ [\textrm{rad/s}]')
    # axes3[1].set_xlabel(r'Time [\textrm{s}]')

    # for ax in axes3:
    #     ax.grid(True, linestyle='--', alpha=0.6)
    #     ax.legend(loc='upper right')

    # fig3.tight_layout()
    # fig3.savefig(os.path.join(out_dir, 'robot_twists.png'))

    # print(f"Plots saved in {out_dir}: tracking_errors.png, pose_comparison.png, robot_twists.png")

    # -------------------------------------------------------------------
    # GROUP 3b: estimated vs COMMANDED VELOCITIES (single plot)
    # vx, commanded vx, yaw rate, commanded yaw rate all on one axes.
    # Commanded columns come from /pixhawk/cmd_vel (added to the CSV export).
    # -------------------------------------------------------------------
    if {'cmd_vel_linear_x', 'cmd_vel_angular_z'}.issubset(df.columns):
        fig3b, ax3b = plt.subplots(figsize=(9, 5))

        ax3b.plot(time, df['twist_linear_x'], color='#1f77b4',
                  label=r'$v_x$ estimated [\textrm{m/s}]')
        ax3b.plot(time, df['cmd_vel_linear_x'], '--', color='#2ca02c',
                  label=r'$\bar{v}_x$ commanded [\textrm{m/s}]')
        #ax3b.plot(time, df['twist_angular_z'], color='#ffbf00',
        #          label=r'$\omega_z$ estimated [\textrm{rad/s}]')
        #ax3b.plot(time, df['cmd_vel_angular_z'], '--', color='#9467bd',
        #          label=r'$\bar{\omega}_z$ commanded [\textrm{rad/s}]')

        ax3b.set_xlabel(r'Time [\textrm{s}]')   
        ax3b.set_ylabel(r'Surge Velocity [\textrm{m/s}]')
        ax3b.set_ylim(-0.2, 0.5)  # zoom in on surge velocity
        #ax3b.set_title(r'\textbf{estimated vs.\ commanded velocities, Only surge}')
        ax3b.grid(True, linestyle='--', alpha=0.6)
        ax3b.legend(loc='upper right')  

        fig3b.tight_layout()
        fig3b.savefig(os.path.join(out_dir, 'velocity_cmd_vs_measured.png'))
        print(f"Plot saved in {out_dir}: velocity_cmd_vs_measured.png")
    else:
        print("cmd_vel columns not in CSV — skipping velocity_cmd_vs_measured.png "
              "(re-run export_tracking_bag_csv.py to include /pixhawk/cmd_vel).")
        
    plt.show()

    # # -------------------------------------------------------------------
    # # GROUP 4: XY VIEW (ROS ENU frame) — reference path + robot trajectory
    # # -------------------------------------------------------------------
    # from matplotlib.collections import LineCollection
    # from matplotlib.patches import Circle
    # from matplotlib.lines import Line2D

    # WAYPOINT_RADIUS = 0.8
    # WAYPOINT_COLOR = 'orange'

    # fig4, ax4 = plt.subplots(figsize=(9, 9))

    # # Deduplicated reference path (consecutive duplicates dropped)
    # path_xy = df[['closest_x_m', 'closest_y_m']]
    # path_mask = (path_xy != path_xy.shift()).any(axis=1)
    # path_xy = path_xy[path_mask]
    # ax4.plot(path_xy['closest_x_m'], path_xy['closest_y_m'],
    #          '--', color='black', linewidth=1.5, label=r'Reference path')

    # # Predefined waypoint: orange '+' plus a circle of radius WAYPOINT_RADIUS.
    # waypoints = [(6.25, -1.5)]
    # for (wx, wy) in waypoints:
    #     ax4.plot(wx, wy, marker='+', color=WAYPOINT_COLOR, markersize=14,
    #              markeredgewidth=2.5, linestyle='None', zorder=6)
    #     ax4.add_patch(Circle((wx, wy), WAYPOINT_RADIUS, fill=False,
    #                          edgecolor=WAYPOINT_COLOR, linewidth=1.8, zorder=6))

    # # Robot trajectory colored by absolute cross-track error
    # rx = df['robot_x_m'].to_numpy()
    # ry = df['robot_y_m'].to_numpy()
    # err = np.abs(df['cross_track_xy_m'].to_numpy())

    # # RMSE of the cross-track error + completion time over the cropped window.
    # t = time.to_numpy()
    # title = os.path.basename(os.path.dirname(out_dir)) or out_dir
    # rmse_err = float(np.sqrt(np.mean(err ** 2)))
    # duration_sec = float(t[-1])
    # minutes = int(duration_sec // 60)
    # seconds = duration_sec - minutes * 60
    # print(f"[{title}] RMSE(e_track) = {rmse_err:.4f} m | "
    #       f"completion time = {minutes} min {seconds:.1f} s")

    # # Per-segment color = mean error of its two endpoints (length = N-1).
    # err_seg = 0.5 * (err[:-1] + err[1:])
    # points = np.array([rx, ry]).T.reshape(-1, 1, 2)
    # segments = np.concatenate([points[:-1], points[1:]], axis=1)
    # lc = LineCollection(segments, cmap='viridis',
    #                     norm=plt.Normalize(0.0, err.max()), linewidth=2)
    # lc.set_array(err_seg)
    # ax4.add_collection(lc)
    # cbar = fig4.colorbar(lc, ax=ax4, orientation='horizontal',
    #                      shrink=0.85, pad=0.1)
    # cbar.set_label(r'$|e_{xy}|$ \,---\, cross-track error [\textrm{m}]')

    # # Start / end markers
    # ax4.plot(rx[0], ry[0], 'o', color='green', markersize=10,
    #          markeredgecolor='black', label=r'Start', zorder=5)
    # ax4.plot(rx[-1], ry[-1], 's', color='red', markersize=10,
    #          markeredgecolor='black', label=r'End', zorder=5)

    # ax4.set_xlabel(r'East / $X$ [\textrm{m}]')
    # ax4.set_ylabel(r'North / $Y$ [\textrm{m}]')
    # ax4.set_title(r'\textbf{$X$--$Y$ plane view (ROS ENU) --- robot trajectory vs.\ reference path}')
    # ax4.set_aspect('equal', adjustable='box')
    # ax4.set_ylim(-4, 1)
    # ax4.xaxis.set_major_locator(plt.MultipleLocator(0.5))
    # ax4.yaxis.set_major_locator(plt.MultipleLocator(0.5))
    # ax4.grid(True, linestyle='--', alpha=0.6)

    # wp_proxy = Line2D([0], [0], marker='+', color=WAYPOINT_COLOR,
    #                   markersize=14, markeredgewidth=2.5,
    #                   linestyle='None', label=r'Predefined waypoints')
    # handles, labels = ax4.get_legend_handles_labels()
    # ax4.legend(handles + [wp_proxy], labels + [r'Predefined waypoints'],
    #            loc='best')

    # fig4.tight_layout()
    # fig4.savefig(os.path.join(out_dir, 'xy_view_latex.png'))
    # fig4.savefig(os.path.join(out_dir, 'xy_view_latex.pdf'), bbox_inches='tight')

    # print(f"Plots saved in {out_dir}: tracking_errors.png, pose_comparison.png, xy_view.png")

    # # Single blocking show so all four figures stay on screen until you close them.
    # plt.show()

    # -------------------------------------------------------------------
    # GROUP 5: XY ANIMATION (ROS ENU) — moving robot + yaw stick + time bar
    # -------------------------------------------------------------------
    # from matplotlib.animation import FuncAnimation, PillowWriter

    # # Yaw from quaternion (ENU map frame)
    # qx = df['robot_qx'].to_numpy()
    # qy = df['robot_qy'].to_numpy()
    # qz = df['robot_qz'].to_numpy()
    # qw = df['robot_qw'].to_numpy()
    # yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
    #                  1.0 - 2.0 * (qy * qy + qz * qz))

    # # Subsample to ~200 frames
    # n_frames = 200
    # n = len(t)
    # idx = np.linspace(0, n - 1, min(n_frames, n)).astype(int)
    # rx_a, ry_a, yaw_a, t_a = rx[idx], ry[idx], yaw[idx], t[idx]

    # norm = plt.Normalize(t.min(), t.max())

    # fig5 = plt.figure(figsize=(9, 10))
    # gs = fig5.add_gridspec(2, 1, height_ratios=[20, 1], hspace=0.15)
    # ax5 = fig5.add_subplot(gs[0])
    # ax_bar = fig5.add_subplot(gs[1])

    # # Static background: reference path, start, end
    # ax5.plot(path_xy['closest_x_m'], path_xy['closest_y_m'],
    #          '--', color='black', linewidth=1.5, label='Reference path')
    # ax5.plot(rx[0], ry[0], 'o', color='green', markersize=10,
    #          markeredgecolor='black', label='Start', zorder=5)
    # ax5.plot(rx[-1], ry[-1], 's', color='red', markersize=10,
    #          markeredgecolor='black', label='End', zorder=5)

    # ax5.set_xlabel('East / X [m]')
    # ax5.set_ylabel('North / Y [m]')
    # ax5.set_title('X-Y Plane Animation (ROS ENU) — Robot + Yaw')
    # ax5.set_aspect('equal', adjustable='box')
    # ax5.set_xlim(rx.min() - 1, rx.max() + 1)
    # ax5.set_ylim(-5, 5)
    # ax5.yaxis.set_major_locator(plt.MultipleLocator(1))
    # ax5.grid(True, linestyle='--', alpha=0.6)

    # # Dynamic artists
    # trail = LineCollection([], cmap='viridis', norm=norm, linewidth=2)
    # ax5.add_collection(trail)
    # robot_dot, = ax5.plot([], [], 'o', color='white',
    #                       markeredgecolor='black', markersize=9, zorder=6)
    # stick_len = 1.15
    # yaw_stick, = ax5.plot([], [], '-', color='#ff6f00', linewidth=3.0,
    #                       zorder=6, label='Heading (yaw)')
    # lookahead_radius = 2.5

    # # Precompute path arrays + cumulative arc length for lookahead lookup
    # path_x_arr = path_xy['closest_x_m'].to_numpy()
    # path_y_arr = path_xy['closest_y_m'].to_numpy()
    # path_diff = np.hypot(np.diff(path_x_arr), np.diff(path_y_arr))
    # path_s = np.concatenate([[0.0], np.cumsum(path_diff)])

    # from matplotlib.patches import FancyArrowPatch
    # from matplotlib.lines import Line2D
    # lookahead_arrow = FancyArrowPatch((0, 0), (0, 0),
    #                                   arrowstyle='-|>', mutation_scale=8,
    #                                   color='#444444', linewidth=1.4,
    #                                   zorder=5)
    # ax5.add_patch(lookahead_arrow)
    # arrow_proxy = Line2D([0], [0], color='#444444', linewidth=1.4,
    #                      marker='>', markersize=7,
    #                      label='Lookahead point')

    # handles, labels = ax5.get_legend_handles_labels()
    # ax5.legend(handles + [arrow_proxy], labels + ['Lookahead point'],
    #            loc='upper right')

    # # Time-bar axes: full viridis strip + a white overlay that shrinks as time advances
    # gradient = np.linspace(0, 1, 256).reshape(1, -1)
    # ax_bar.imshow(gradient, aspect='auto', cmap='viridis',
    #               extent=[t.min(), t.max(), 0, 1])
    # cover = ax_bar.axvspan(t.min(), t.max(), color='white', alpha=0.85)
    # ax_bar.set_xlim(t.min(), t.max())
    # ax_bar.set_ylim(0, 1)
    # ax_bar.set_yticks([])
    # ax_bar.set_xlabel('Elapsed time [s]')

    # def update(i):
    #     # Trail up to current frame
    #     cur_end = idx[i] + 1
    #     pts = np.array([rx[:cur_end], ry[:cur_end]]).T.reshape(-1, 1, 2)
    #     if len(pts) >= 2:
    #         segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    #         trail.set_segments(segs)
    #         trail.set_array(t[:cur_end - 1])
    #     # Robot + yaw stick
    #     x, y, psi = rx_a[i], ry_a[i], yaw_a[i]
    #     robot_dot.set_data([x], [y])
    #     yaw_stick.set_data([x, x + stick_len * np.cos(psi)],
    #                        [y, y + stick_len * np.sin(psi)])
    #     # Lookahead/carrot point: walk forward along path arc length by lookahead_radius
    #     d2 = (path_x_arr - x) ** 2 + (path_y_arr - y) ** 2
    #     i_near = int(np.argmin(d2))
    #     target_s = path_s[i_near] + lookahead_radius
    #     i_la = int(np.searchsorted(path_s, target_s))
    #     i_la = min(i_la, len(path_x_arr) - 1)
    #     lx, ly = path_x_arr[i_la], path_y_arr[i_la]
    #     lookahead_arrow.set_positions((x, y), (lx, ly))
    #     # Time-bar cover: shrink from the right as time advances
    #     cur_t = t_a[i]
    #     cover.set_xy([[cur_t, 0], [cur_t, 1],
    #                   [t.max(), 1], [t.max(), 0], [cur_t, 0]])
    #     return (trail, robot_dot, yaw_stick, lookahead_arrow, cover)

    # anim = FuncAnimation(fig5, update, frames=len(idx),
    #                      interval=50, blit=False)
    # gif_path = os.path.join(out_dir, 'xy_animation_bad_yaw.gif')
    # target_duration_s = 9.57 + 5.0
    # fps = max(1, round(len(idx) / target_duration_s))
    # anim.save(gif_path, writer=PillowWriter(fps=fps))
    # plt.close(fig5)
    # print(f"Animation saved: {gif_path}")

if __name__ == "__main__":
    csv_path = "/home/polaris_pz/Downloads/recordings_final_autonomy_lake4/aut_no_z_below_rectangle_04_2026_06_03-13_05_50/csv_export/velocity_cmd_vs_measured.csv"
    # Foxglove markers (elapsed seconds since bag start):
    #   start = 6.569712344 s
    #   end   = 42.806242311 s
    # foxglove_offset = (CSV first-sample time) - (bag start time). Leave 0 if unsure;
    # the cropped window will then be in CSV-elapsed seconds instead.
    plot_data(
        csv_path,
        start_time=220,
        end_time=300,
        foxglove_offset=0.0,
    )
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
    "legend.fontsize": 10,
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
    # GROUP 3b: MEASURED vs COMMANDED VELOCITIES (single plot)
    # vx, commanded vx, yaw rate, commanded yaw rate all on one axes.
    # Commanded columns come from /pixhawk/cmd_vel (added to the CSV export).
    # -------------------------------------------------------------------
    if {'cmd_vel_linear_x', 'cmd_vel_angular_z'}.issubset(df.columns):
        from matplotlib.collections import LineCollection
        from matplotlib.lines import Line2D

        fig3b, ax3b = plt.subplots(figsize=(9, 5))

        # Grey highlight band over t in [6, 15] s, full height.
        ax3b.axvspan(6, 15, color='gray', alpha=0.2, zorder=0)

        # vx estimated drawn as colored segments: red where it leaves the
        # [VX_LO, VX_HI] band OR during a localization glitch (a physically
        # impossible position jump between consecutive samples), blue otherwise.
        VX_LO, VX_HI = -0.2, 0.5
        GLITCH_STEP_M = 2.0
        tt = time.to_numpy()
        vx = df['twist_linear_x'].to_numpy()
        rx = df['robot_x_m'].to_numpy()
        ry = df['robot_y_m'].to_numpy()
        glitch_seg = np.hypot(np.diff(rx), np.diff(ry)) > GLITCH_STEP_M
        oor = (vx < VX_LO) | (vx > VX_HI)
        anom_seg = oor[:-1] | oor[1:] | glitch_seg   # per-segment (length N-1)

        pts = np.array([tt, vx]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        vx_colors = np.where(anom_seg, 'red', '#1f77b4')
        ax3b.add_collection(LineCollection(segs, colors=vx_colors, linewidth=1.5))

        ax3b.plot(time, df['cmd_vel_linear_x'], '--', color='#2ca02c',
                  label=r'$\bar{v}_x$ commanded [\textrm{m/s}]')
        ax3b.plot(time, df['twist_angular_z'], color='#ffbf00',
                  label=r'$\omega_z$ estimated [\textrm{rad/s}]')
        ax3b.plot(time, df['cmd_vel_angular_z'], '--', color='#9467bd',
                  label=r'$\bar{\omega}_z$ commanded [\textrm{rad/s}]')

        ax3b.set_xlabel(r'Time [\textrm{s}]')
        ax3b.set_ylabel(r'Twist [\textrm{m/s}, \textrm{rad/s}]')
        ax3b.set_ylim(-0.2, 0.5)
        #ax3b.set_title(r'\textbf{estimated vs.\ commanded velocities}')
        ax3b.grid(True, linestyle='--', alpha=0.6)

        # LineCollection isn't auto-legendable — add proxies for the vx line.
        vx_ok = Line2D([0], [0], color='#1f77b4', linewidth=1.5,
                       label=r'$v_x$ estimated [\textrm{m/s}]')
        vx_bad = Line2D([0], [0], color='red', linewidth=1.5,
                        label=r'$v_x$ glitched [\textrm{m/s}]')
        n_anom = int(anom_seg.sum())
        print(f"velocity plot: {n_anom} anomalous vx segments "
              f"(out of [{VX_LO}, {VX_HI}] m/s or step > {GLITCH_STEP_M:g} m)")
        handles, labels = ax3b.get_legend_handles_labels()
        ax3b.legend([vx_ok, vx_bad] + handles,
                    [vx_ok.get_label(), vx_bad.get_label()] + labels,
                    loc='upper right')

        fig3b.tight_layout()
        fig3b.savefig(os.path.join(out_dir, 'velocity_cmd_vs_measured.png'))
        print(f"Plot saved in {out_dir}: velocity_cmd_vs_measured.png")
    else:
        print("cmd_vel columns not in CSV — skipping velocity_cmd_vs_measured.png "
              "(re-run export_tracking_bag_csv.py to include /pixhawk/cmd_vel).")

    # -------------------------------------------------------------------
    # GROUP 3c: SCHMITT TRIGGER VIEW
    # Top:    yaw error vs time, with the hysteresis band drawn in.
    # Bottom: commanded yaw rate (the trigger output) vs the actual yaw
    #         rate (the plant response). The cmd snaps on/off as the error
    #         leaves/enters the band; the estimated rate lags and follows.
    # -------------------------------------------------------------------
    if {'yaw_error_rad', 'cmd_vel_angular_z'}.issubset(df.columns):
        # Hysteresis thresholds of the Schmitt trigger [rad]. Set to the
        # values used by the autonomy node; None = don't draw the band.
        YAW_ERR_HIGH = None   # e.g. 0.20 — error must exceed this to switch ON
        YAW_ERR_LOW = None    # e.g. 0.10 — error must drop below this to switch OFF

        fig3c, axes3c = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

        # Grey highlight band over t in [6, 15] s, full height (both subplots).
        for ax in axes3c:
            ax.axvspan(6, 15, color='gray', alpha=0.2, zorder=0)

        # --- Top: yaw error (converted rad -> deg) ---
        yaw_err_deg = np.degrees(df['yaw_error_rad'])
        axes3c[0].plot(time, yaw_err_deg, color='#d62728',
                       label=r'Yaw error $e_\psi$')
        if YAW_ERR_HIGH is not None:
            for sign in (1, -1):
                axes3c[0].axhline(sign * np.degrees(YAW_ERR_HIGH), color='gray',
                                  linestyle='--', linewidth=1.0)
        if YAW_ERR_LOW is not None:
            for sign in (1, -1):
                axes3c[0].axhline(sign * np.degrees(YAW_ERR_LOW), color='gray',
                                  linestyle=':', linewidth=1.0)
        if YAW_ERR_HIGH is not None and YAW_ERR_LOW is not None:
            axes3c[0].axhspan(np.degrees(YAW_ERR_LOW), np.degrees(YAW_ERR_HIGH),
                              color='gray', alpha=0.12)
            axes3c[0].axhspan(-np.degrees(YAW_ERR_HIGH), -np.degrees(YAW_ERR_LOW),
                              color='gray', alpha=0.12, label=r'Hysteresis band')
        axes3c[0].axhline(0.0, color='black', linewidth=0.6, alpha=0.5)
        axes3c[0].set_ylabel(r'Yaw error [\textrm{deg}]')
        #axes3c[0].set_title(r'\textbf{Schmitt trigger: yaw error and yaw-rate response}')

        # --- Bottom: commanded vs actual yaw rate ---
        axes3c[1].plot(time, df['cmd_vel_angular_z'], color='#9467bd',
                       label=r'$\bar{\omega}_z$ commanded')
        axes3c[1].plot(time, df['twist_angular_z'], color='#ffbf00',
                       label=r'$\omega_z$ estimated ')
        axes3c[1].axhline(0.0, color='black', linewidth=0.6, alpha=0.5)
        axes3c[1].set_ylabel(r'Yaw rate [\textrm{rad/s}]')
        axes3c[1].set_xlabel(r'Time [\textrm{s}]')

        for ax in axes3c:
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend(loc='upper right')

        fig3c.tight_layout()
        fig3c.savefig(os.path.join(out_dir, 'schmitt_trigger.png'))
        fig3c.savefig(os.path.join(out_dir, 'schmitt_trigger.pdf'), bbox_inches='tight')
        print(f"Plot saved in {out_dir}: schmitt_trigger.png")
    else:
        print("yaw_error_rad / cmd_vel_angular_z not in CSV — skipping schmitt_trigger.png.")

    plt.show()

    # -------------------------------------------------------------------
    # GROUP 4: XY VIEW (ROS ENU frame) — reference path + robot trajectory
    # -------------------------------------------------------------------
    # from matplotlib.collections import LineCollection
    # from matplotlib.patches import Circle
    # from matplotlib.lines import Line2D

    # WAYPOINT_RADIUS = 0.8   # = goal_checker xy_goal_tolerance in nav2_params.yaml
    # WAYPOINT_COLOR = 'orange'
    # # The controller's closest-point trace is NOT the reference path: it is the
    # # foot-point projection and jumps discontinuously at each leg handoff. Off by
    # # default; enable only as a tracking-debug overlay.
    # SHOW_FOOTPOINT = False

    # fig4, ax4 = plt.subplots(figsize=(9, 9))

    # # Reference path = the actual planned legs from /plan (planned_path.csv),
    # # written by export_tracking_bag_csv.py. One dashed line per leg.
    # plan_csv = os.path.join(out_dir, 'planned_path.csv')
    # if os.path.isfile(plan_csv):
    #     plan_df = pd.read_csv(plan_csv)
    #     first = True
    #     for _, leg in plan_df.groupby('leg_index'):
    #         ax4.plot(leg['plan_x_m'], leg['plan_y_m'], '--', color='black',
    #                  linewidth=1.5, zorder=3,
    #                  label=(r'Reference path (/plan)' if first else None))
    #         first = False
    # else:
    #     print(f'WARNING: {plan_csv} not found — re-run export_tracking_bag_csv.py '
    #           'for the truthful reference path. Skipping it.')

    # # Waypoints = the goals sent to Nav2 (mission_waypoints.csv) — never hardcoded.
    # wp_csv = os.path.join(out_dir, 'mission_waypoints.csv')
    # if os.path.isfile(wp_csv):
    #     wp_df = pd.read_csv(wp_csv)
    #     waypoints = list(zip(wp_df['wp_x_m'], wp_df['wp_y_m']))
    # else:
    #     waypoints = []
    #     print(f'WARNING: {wp_csv} not found — re-run export_tracking_bag_csv.py. '
    #           'No waypoint markers will be drawn.')
    # for (wx, wy) in waypoints:
    #     ax4.plot(wx, wy, marker='+', color=WAYPOINT_COLOR, markersize=14,
    #              markeredgewidth=2.5, linestyle='None', zorder=6)
    #     ax4.add_patch(Circle((wx, wy), WAYPOINT_RADIUS, fill=False,
    #                          edgecolor=WAYPOINT_COLOR, linewidth=1.8, zorder=6))

    # # Optional diagnostic: controller closest-point (foot-point) trace.
    # if SHOW_FOOTPOINT:
    #     foot = df[['closest_x_m', 'closest_y_m']]
    #     foot = foot[(foot != foot.shift()).any(axis=1)]
    #     ax4.plot(foot['closest_x_m'], foot['closest_y_m'], ':', color='gray',
    #              linewidth=1.0, alpha=0.7, zorder=2,
    #              label=r'Controller foot-point (diagnostic)')

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
    # # ax4.set_ylim(-4, 1)
    # ax4.xaxis.set_major_locator(plt.MultipleLocator(0.5))
    # ax4.yaxis.set_major_locator(plt.MultipleLocator(0.5))
    # ax4.grid(True, linestyle='--', alpha=0.6)

    # wp_proxy = Line2D([0], [0], marker='+', color=WAYPOINT_COLOR,
    #                   markersize=14, markeredgewidth=2.5,
    #                   linestyle='None', label=r'Mission waypoints (Nav2 goals)')
    # # Proxy for the robot's actual (driven) trajectory — the viridis-colored line.
    # # Uses a mid-colormap color since the real line is colored by cross-track error.
    # traj_proxy = Line2D([0], [0], color=plt.get_cmap('viridis')(0.5),
    #                     linewidth=2, label=r'Actual path (AUV)')
    # handles, labels = ax4.get_legend_handles_labels()
    # ax4.legend(handles + [traj_proxy, wp_proxy],
    #            labels + [r'Actual path (AUV)', r'Mission waypoints (Nav2 goals)'],
    #            loc='best')

    # fig4.tight_layout()
    # fig4.savefig(os.path.join(out_dir, 'xy_view_latex.png'))
    # fig4.savefig(os.path.join(out_dir, 'xy_view_latex.pdf'), bbox_inches='tight')

    # print(f"Plots saved in {out_dir}: tracking_errors.png, pose_comparison.png, xy_view.png")

    # # Single blocking show so all four figures stay on screen until you close them.
    # plt.show()


if __name__ == "__main__":
    csv_path = "/home/polaris_pz/Downloads/recordings_final_autonomy_lake4/aut_no_z_surface_grid_01_2026_06_03-13_23_08/csv_export/tracking_errors_wide.csv"
    # Foxglove markers (elapsed seconds since bag start):
    #   start = 6.569712344 s
    #   end   = 42.806242311 s
    # foxglove_offset = (CSV first-sample time) - (bag start time). Leave 0 if unsure;
    # the cropped window will then be in CSV-elapsed seconds instead.
    plot_data(
        csv_path,
        start_time=430,   # None = plot the entire recording (no cropping)
        end_time=490,
        foxglove_offset=0.0,  # = csv_first_sample - bag_start (from metadata.yaml)
    )
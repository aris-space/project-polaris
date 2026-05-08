"""
Thruster Odometry Analysis
==========================
Compares velocity and position estimation across four methods:

  - Static prior     : k=0.780, b=0.964 fixed, no learning
  - Surge-trained RLS: online RLS trained on Zürisee surge bags (surge_02–06)
  - Zermatt-trained  : online RLS trained on Zermatt grid survey (16 min)
  - IMU-only         : double-integration of /filter/free_acceleration

Ground truth
  - Velocity : /sensors/dvl/odometry_cov  (locked samples, cov < 1e-4)
  - Position : /odometry/filtered/global  (GPS-anchored full EKF, 30 Hz)

Evaluation bags
  - Velocity : surge_01  (held out from surge training)
  - Position : rectangle_02  (~88 s closed rectangle)

Usage (after sourcing the workspace):
  python3 /ros2_ws/analysis/thruster_odometry_analysis.py

Figures are saved to /ros2_ws/analysis/.
"""

import warnings
warnings.filterwarnings('ignore')

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ── numpy-only interpolation helpers ─────────────────────────────────────────

def _cumtrapz(y, x):
    """Cumulative trapezoidal integration, same shape as y, zero at index 0."""
    dt = np.diff(x)
    return np.concatenate([[0.0], np.cumsum(0.5 * (y[:-1] + y[1:]) * dt)])


def _zoh(t_query, t_data, y_data, fill=0.0):
    """Zero-order hold: return last y_data value at or before each t_query."""
    idx    = np.searchsorted(t_data, t_query, side='right') - 1
    result = np.full(len(t_query), fill)
    good   = (idx >= 0) & (idx < len(y_data))
    result[good] = y_data[idx[good]]
    return result


# ── Configuration ─────────────────────────────────────────────────────────────

REC      = Path('/ros2_ws/recordings')
ZUERISEE = REC / 'zuerisee_ekf_manoeuvres'
ZERMATT  = REC / 'zermatt_grid_01_2026_04_30-12_04_16'

SURGE_TRAIN_BAGS = [
    ZUERISEE / 'surge_02_2026_05_07-14_08_19',
    ZUERISEE / 'surge_03_2026_05_07-14_10_13',
    ZUERISEE / 'surge_05_2026_05_07-14_12_32',
    ZUERISEE / 'surge_06_2026_05_07-14_14_13',
]
EVAL_VEL = ZUERISEE / 'surge_01_2026_05_07-14_05_34'
EVAL_POS = ZUERISEE / 'rectangle_02_2026_05_07-12_06_37'

ANALYSIS_DIR = Path('/ros2_ws/analysis')

# Must match adaptive_thruster_estimator C++ node parameters
K_PRIOR      = 0.780
B_PRIOR      = 0.964
RLS_LAMBDA   = 0.995
U_MIN        = 0.10
ACCEL_THRESH = 0.05    # m/s²
DVL_COV_MAX  = 1e-4
PWM_NEUTRAL  = 1500
PWM_RANGE    = 500.0

COLORS = dict(
    gt      = '#1a1a1a',
    imu     = '#e74c3c',
    static  = '#3498db',
    surge   = '#e67e22',
    zermatt = '#27ae60',
)

plt.rcParams.update({
    'figure.dpi': 130,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.8,
    'font.size': 11,
})


# ── Bag reader ────────────────────────────────────────────────────────────────

def read_bag(bag_path, topics):
    """Read MCAP bag → {topic: [(t_sec, msg), ...]}."""
    bag_path = Path(bag_path)
    uri = str(bag_path)
    if bag_path.is_dir() and not (bag_path / 'metadata.yaml').exists():
        mcaps = sorted(bag_path.glob('*.mcap'))
        if not mcaps:
            raise FileNotFoundError(f'No .mcap in {bag_path}')
        uri = str(mcaps[0])

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_map = {i.name: i.type for i in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    data = {t: [] for t in topics}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in data and topic in type_map:
            data[topic].append(
                (t_ns * 1e-9, deserialize_message(raw, get_message(type_map[topic])))
            )
    return data


# ── RLS model (mirrors adaptive_thruster_estimator.cpp) ───────────────────────

class RLSModel:
    def __init__(self, k=K_PRIOR, b=B_PRIOR, P_init=1e4):
        self.theta   = np.array([np.log(k), b])
        self.P       = np.eye(2) * P_init
        self.history = []   # list of (t, k, b, trace_P)
        self._n      = 0

    @property
    def k(self): return float(np.exp(self.theta[0]))

    @property
    def b(self): return float(self.theta[1])

    def estimate(self, u):
        abs_u = abs(u)
        if abs_u < U_MIN:
            return 0.0
        return float(np.exp(self.theta[0]) * abs_u ** self.theta[1] * np.sign(u))

    def _rls_step(self, ln_vx, ln_u, t):
        phi        = np.array([1.0, ln_u])
        denom      = RLS_LAMBDA + phi @ self.P @ phi
        K          = self.P @ phi / denom
        self.theta += K * (ln_vx - phi @ self.theta)
        self.theta[0] = np.clip(self.theta[0], np.log(0.05), np.log(10.0))
        self.theta[1] = np.clip(self.theta[1], 0.1, 5.0)
        self.P     = (1 / RLS_LAMBDA) * (np.eye(2) - np.outer(K, phi)) @ self.P
        self.P     = 0.5 * (self.P + self.P.T)
        self._n   += 1
        self.history.append((t, self.k, self.b, float(np.trace(self.P))))

    def train(self, bag_data):
        """Run online RLS on bag data. Returns number of update steps."""
        servo = bag_data.get('/pixhawk/servo_output_raw', [])
        dvl   = bag_data.get('/sensors/dvl/odometry_cov', [])
        if not servo or not dvl:
            return 0

        st = np.array([t for t, _ in servo])
        su = np.array([(m.data[0] - PWM_NEUTRAL) / PWM_RANGE for _, m in servo])

        prev_vx = prev_t = None
        n_updates = 0
        for t, m in dvl:
            cov = m.twist.covariance[0]
            if not (0 < cov < DVL_COV_MAX):
                prev_vx = None
                continue
            vx = m.twist.twist.linear.x
            u  = su[int(np.clip(np.searchsorted(st, t), 0, len(su) - 1))]

            steady = True
            if prev_vx is not None and prev_t is not None:
                dt = t - prev_t
                if 0 < dt < 1.0:
                    steady = abs(vx - prev_vx) / dt < ACCEL_THRESH
            prev_vx, prev_t = vx, t

            if steady and abs(u) > U_MIN and abs(vx) > U_MIN and (u * vx > 0):
                self._rls_step(np.log(abs(vx)), np.log(abs(u)), t)
                n_updates += 1
        return n_updates


# ── Coordinate / dead-reckoning helpers ───────────────────────────────────────

def quat_to_yaw(msg):
    q = msg.quaternion
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def make_yaw_interp(quat_data):
    ts = np.array([t for t, _ in quat_data])
    ys = np.unwrap(np.array([quat_to_yaw(m) for _, m in quat_data]))
    return lambda t: np.interp(t, ts, ys)


def dead_reckon(times, vx_body, yaw_fn, x0=0.0, y0=0.0):
    yaws     = yaw_fn(times)
    dt       = np.diff(times, prepend=times[0]);  dt[0] = 0.0
    xs       = x0 + np.cumsum(vx_body * np.cos(yaws) * dt)
    ys       = y0 + np.cumsum(vx_body * np.sin(yaws) * dt)
    xs[0], ys[0] = x0, y0
    return xs, ys


def imu_dead_reckon(imu_t, a_east, a_north, ve0, vn0, x0, y0):
    vel_e = _cumtrapz(a_east,  imu_t) + ve0
    vel_n = _cumtrapz(a_north, imu_t) + vn0
    pos_e = x0 + _cumtrapz(vel_e, imu_t)
    pos_n = y0 + _cumtrapz(vel_n, imu_t)
    return pos_e, pos_n


def pos_error_at_gt(t_est, x_est, y_est, t_gt, x_gt, y_gt):
    xi = np.interp(t_gt, t_est, x_est)
    yi = np.interp(t_gt, t_est, y_est)
    return np.sqrt((xi - x_gt) ** 2 + (yi - y_gt) ** 2)


def rmse(e):
    return float(np.sqrt(np.mean(e ** 2)))


# ── Training ──────────────────────────────────────────────────────────────────

def train_models():
    TRAIN_TOPICS = ['/pixhawk/servo_output_raw', '/sensors/dvl/odometry_cov']

    print('Training surge model (Zürisee surge_02–06):')
    surge = RLSModel()
    bag_boundaries = [0]
    for bag in SURGE_TRAIN_BAGS:
        data = read_bag(bag, TRAIN_TOPICS)
        n    = surge.train(data)
        bag_boundaries.append(surge._n)
        print(f'  {bag.name}: {n:4d} updates  k={surge.k:.4f}  b={surge.b:.4f}')
    print(f'  Final: k={surge.k:.4f}  b={surge.b:.4f}  trace(P)={np.trace(surge.P):.3g}')

    print(f'\nTraining Zermatt model ({ZERMATT.name}):')
    zermatt = RLSModel()
    data    = read_bag(ZERMATT, TRAIN_TOPICS)
    n       = zermatt.train(data)
    print(f'  {n:4d} updates  k={zermatt.k:.4f}  b={zermatt.b:.4f}  trace(P)={np.trace(zermatt.P):.3g}')

    static = RLSModel()   # cold prior, never trained
    print(f'\nStatic prior: k={static.k:.4f}  b={static.b:.4f}')

    return static, surge, zermatt, bag_boundaries, TRAIN_TOPICS


# ── Velocity evaluation ───────────────────────────────────────────────────────

def load_velocity_eval(static, surge, zermatt):
    TRAIN_TOPICS = ['/pixhawk/servo_output_raw', '/sensors/dvl/odometry_cov']
    print(f'\nLoading velocity eval bag: {EVAL_VEL.name}')
    vd = read_bag(EVAL_VEL, TRAIN_TOPICS)

    srv_t = np.array([t for t, _ in vd['/pixhawk/servo_output_raw']])
    srv_u = np.array([(m.data[0] - PWM_NEUTRAL) / PWM_RANGE
                      for _, m in vd['/pixhawk/servo_output_raw']])

    vel_t, vel_dvl, vel_u = [], [], []
    for t, m in vd['/sensors/dvl/odometry_cov']:
        cov = m.twist.covariance[0]
        if 0 < cov < DVL_COV_MAX:
            vel_t.append(t)
            vel_dvl.append(m.twist.twist.linear.x)
            idx = int(np.clip(np.searchsorted(srv_t, t), 0, len(srv_u) - 1))
            vel_u.append(srv_u[idx])

    vel_t   = np.array(vel_t);   vel_t -= vel_t[0]
    vel_dvl = np.array(vel_dvl); vel_u  = np.array(vel_u)
    srv_t  -= srv_t[0]

    vel_static  = np.array([static.estimate(u)  for u in vel_u])
    vel_surge   = np.array([surge.estimate(u)   for u in vel_u])
    vel_zermatt = np.array([zermatt.estimate(u) for u in vel_u])

    r_static  = rmse(vel_static  - vel_dvl)
    r_surge   = rmse(vel_surge   - vel_dvl)
    r_zermatt = rmse(vel_zermatt - vel_dvl)

    print(f'  {len(vel_t)} locked DVL samples')
    print(f'  Velocity RMSE  static={r_static:.4f}  surge={r_surge:.4f}  zermatt={r_zermatt:.4f} m/s')

    return dict(
        vel_t=vel_t, vel_dvl=vel_dvl, vel_u=vel_u,
        srv_t=srv_t, srv_raw=vd['/pixhawk/servo_output_raw'],
        vel_static=vel_static, vel_surge=vel_surge, vel_zermatt=vel_zermatt,
        r_static=r_static, r_surge=r_surge, r_zermatt=r_zermatt,
    )


# ── Position evaluation ───────────────────────────────────────────────────────

def load_position_eval(static, surge, zermatt):
    POS_TOPICS = [
        '/pixhawk/servo_output_raw',
        '/sensors/dvl/odometry_cov',
        '/filter/free_acceleration',
        '/filter/quaternion',
        '/odometry/filtered/global',
    ]
    print(f'\nLoading position eval bag: {EVAL_POS.name}')
    pd_ = read_bag(EVAL_POS, POS_TOPICS)

    # Ground truth
    gt_msgs = pd_['/odometry/filtered/global']
    gt_t    = np.array([t for t, _ in gt_msgs])
    gt_x    = np.array([m.pose.pose.position.x for _, m in gt_msgs])
    gt_y    = np.array([m.pose.pose.position.y for _, m in gt_msgs])
    t0      = gt_t[0]
    gt_t   -= t0;  gt_x -= gt_x[0];  gt_y -= gt_y[0]

    yaw_fn = make_yaw_interp([(t - t0, m) for t, m in pd_['/filter/quaternion']])

    imu_msgs = pd_['/filter/free_acceleration']
    imu_t    = np.array([t - t0 for t, _ in imu_msgs])
    imu_ae   = np.array([m.vector.x for _, m in imu_msgs])
    imu_an   = np.array([m.vector.y for _, m in imu_msgs])

    ps_msgs  = pd_['/pixhawk/servo_output_raw']
    ps_t     = np.array([t - t0 for t, _ in ps_msgs])
    ps_u     = np.array([(m.data[0] - PWM_NEUTRAL) / PWM_RANGE for _, m in ps_msgs])
    eval_t, eval_dvl_vx = [], []
    for t, m in pd_['/sensors/dvl/odometry_cov']:
        if 0 < m.twist.covariance[0] < DVL_COV_MAX:
            eval_t.append(t - t0)
            eval_dvl_vx.append(m.twist.twist.linear.x)
    eval_t      = np.array(eval_t)
    eval_dvl_vx = np.array(eval_dvl_vx)
    eval_u      = _zoh(eval_t, ps_t, ps_u, fill=0.0)

    yaw0  = float(yaw_fn(eval_t[0]))
    ve0   = eval_dvl_vx[0] * np.cos(yaw0)
    vn0   = eval_dvl_vx[0] * np.sin(yaw0)
    x0, y0 = gt_x[0], gt_y[0]

    print(f'  GT:       {len(gt_t)} msgs @ {len(gt_t)/gt_t[-1]:.1f} Hz')
    print(f'  DVL lock: {len(eval_t)} msgs,  duration {eval_t[-1]:.1f} s')
    print(f'  IMU:      {len(imu_t)} msgs @ {len(imu_t)/imu_t[-1]:.1f} Hz')

    # Dead reckon all methods
    vx_static_p  = np.array([static.estimate(u)  for u in eval_u])
    vx_surge_p   = np.array([surge.estimate(u)   for u in eval_u])
    vx_zermatt_p = np.array([zermatt.estimate(u) for u in eval_u])

    x_static,  y_static  = dead_reckon(eval_t, vx_static_p,  yaw_fn, x0, y0)
    x_surge,   y_surge   = dead_reckon(eval_t, vx_surge_p,   yaw_fn, x0, y0)
    x_zermatt, y_zermatt = dead_reckon(eval_t, vx_zermatt_p, yaw_fn, x0, y0)
    x_dvl_dr,  y_dvl_dr  = dead_reckon(eval_t, eval_dvl_vx,  yaw_fn, x0, y0)
    x_imu,     y_imu     = imu_dead_reckon(imu_t, imu_ae, imu_an, ve0, vn0, x0, y0)

    e_imu     = pos_error_at_gt(imu_t,  x_imu,     y_imu,     gt_t, gt_x, gt_y)
    e_static  = pos_error_at_gt(eval_t, x_static,  y_static,  gt_t, gt_x, gt_y)
    e_surge   = pos_error_at_gt(eval_t, x_surge,   y_surge,   gt_t, gt_x, gt_y)
    e_zermatt = pos_error_at_gt(eval_t, x_zermatt, y_zermatt, gt_t, gt_x, gt_y)
    e_dvl_dr  = pos_error_at_gt(eval_t, x_dvl_dr,  y_dvl_dr,  gt_t, gt_x, gt_y)

    print('  Position RMSE (m):')
    print(f'    IMU-only:        {rmse(e_imu):.3f}')
    print(f'    Static prior:    {rmse(e_static):.3f}')
    print(f'    Surge-trained:   {rmse(e_surge):.3f}')
    print(f'    Zermatt-trained: {rmse(e_zermatt):.3f}')
    print(f'    DVL DR (ref):    {rmse(e_dvl_dr):.3f}')

    return dict(
        gt_t=gt_t, gt_x=gt_x, gt_y=gt_y,
        imu_t=imu_t, x_imu=x_imu, y_imu=y_imu,
        eval_t=eval_t,
        x_static=x_static,   y_static=y_static,
        x_surge=x_surge,     y_surge=y_surge,
        x_zermatt=x_zermatt, y_zermatt=y_zermatt,
        x_dvl_dr=x_dvl_dr,   y_dvl_dr=y_dvl_dr,
        e_imu=e_imu, e_static=e_static, e_surge=e_surge,
        e_zermatt=e_zermatt, e_dvl_dr=e_dvl_dr,
        x0=x0, y0=y0,
    )


# ── Figures ───────────────────────────────────────────────────────────────────

def fig1_velocity(vd):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True,
                                   gridspec_kw={'height_ratios': [3, 1]})
    ax1.plot(vd['vel_t'], vd['vel_dvl'],    color=COLORS['gt'],      lw=2.0, zorder=5,
             label='DVL (ground truth)')
    ax1.plot(vd['vel_t'], vd['vel_static'], color=COLORS['static'],  lw=1.5, ls='--',
             label=f'Static prior    RMSE={vd["r_static"]:.4f} m/s')
    ax1.plot(vd['vel_t'], vd['vel_surge'],  color=COLORS['surge'],   lw=1.5,
             label=f'Surge-trained   RMSE={vd["r_surge"]:.4f} m/s')
    ax1.plot(vd['vel_t'], vd['vel_zermatt'],color=COLORS['zermatt'], lw=1.5,
             label=f'Zermatt-trained RMSE={vd["r_zermatt"]:.4f} m/s')
    ax1.set_ylabel('Surge velocity $v_x$ (m/s)')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.set_title(f'Fig 1 — Velocity Estimation  |  {EVAL_VEL.name}  (held out from surge training)')

    srv_u_plot = np.array([(m.data[0] - PWM_NEUTRAL) / PWM_RANGE
                            for _, m in vd['srv_raw']])
    ax2.step(vd['srv_t'], srv_u_plot, color='#555', lw=1.2, where='post',
             label='Servo command u')
    ax2.axhline(0, color='k', lw=0.5, ls=':')
    ax2.set_ylabel('Command $u$')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylim(-1.3, 1.3)
    ax2.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    out = ANALYSIS_DIR / 'fig1_velocity.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Saved {out}')
    plt.show()


def fig2_position_drift(pd_):
    fig, ax = plt.subplots(figsize=(14, 5))
    gt_t = pd_['gt_t']
    ax.plot(gt_t, pd_['e_imu'],
            color=COLORS['imu'],     lw=2.0,         label=f'IMU-only        RMSE={rmse(pd_["e_imu"]):.3f} m')
    ax.plot(gt_t, pd_['e_static'],
            color=COLORS['static'],  lw=1.8, ls='--', label=f'Static prior    RMSE={rmse(pd_["e_static"]):.3f} m')
    ax.plot(gt_t, pd_['e_surge'],
            color=COLORS['surge'],   lw=1.8,          label=f'Surge-trained   RMSE={rmse(pd_["e_surge"]):.3f} m')
    ax.plot(gt_t, pd_['e_zermatt'],
            color=COLORS['zermatt'], lw=1.8,          label=f'Zermatt-trained RMSE={rmse(pd_["e_zermatt"]):.3f} m')
    ax.plot(gt_t, pd_['e_dvl_dr'],
            color=COLORS['gt'],      lw=1.5, ls=':',  label=f'DVL DR (ref)    RMSE={rmse(pd_["e_dvl_dr"]):.3f} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position error vs EKF ground truth (m)')
    ax.set_title(f'Fig 2 — Position Drift  |  {EVAL_POS.name}')
    ax.legend(loc='upper left', fontsize=10)
    ax.set_ylim(bottom=0)
    plt.tight_layout()
    out = ANALYSIS_DIR / 'fig2_position_drift.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Saved {out}')
    plt.show()


def fig3_trajectory(pd_):
    fig, ax = plt.subplots(figsize=(9, 9))
    s = 5   # subsample for legibility
    ax.plot(pd_['gt_x'][::s],      pd_['gt_y'][::s],      color=COLORS['gt'],      lw=2.5, zorder=6, label='EKF ground truth')
    ax.plot(pd_['x_imu'][::s],     pd_['y_imu'][::s],     color=COLORS['imu'],     lw=1.5,           label='IMU-only')
    ax.plot(pd_['x_static'][::s],  pd_['y_static'][::s],  color=COLORS['static'],  lw=1.5, ls='--',  label='Static prior')
    ax.plot(pd_['x_surge'][::s],   pd_['y_surge'][::s],   color=COLORS['surge'],   lw=1.5,           label='Surge-trained RLS')
    ax.plot(pd_['x_zermatt'][::s], pd_['y_zermatt'][::s], color=COLORS['zermatt'], lw=1.5,           label='Zermatt-trained RLS')
    ax.plot(pd_['x_dvl_dr'][::s],  pd_['y_dvl_dr'][::s],  color=COLORS['gt'],      lw=1.2, ls=':',   label='DVL DR (ref)')
    ax.plot(pd_['x0'], pd_['y0'], 'ko', ms=8, zorder=10, label='Start')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_aspect('equal')
    ax.set_title(f'Fig 3 — 2D Trajectory  |  {EVAL_POS.name}')
    ax.legend(loc='best', fontsize=10)
    plt.tight_layout()
    out = ANALYSIS_DIR / 'fig3_trajectory.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Saved {out}')
    plt.show()


def fig4_convergence(surge, zermatt, bag_boundaries):
    TRAIN_TOPICS = ['/pixhawk/servo_output_raw', '/sensors/dvl/odometry_cov']

    # Re-run surge training to capture per-update history
    surge_conv    = RLSModel()
    bounds        = [0]
    for bag in SURGE_TRAIN_BAGS:
        data = read_bag(bag, TRAIN_TOPICS)
        surge_conv.train(data)
        bounds.append(surge_conv._n)

    fig, (ax_k, ax_b, ax_p) = plt.subplots(3, 1, figsize=(14, 9))

    def _plot(model, label, color):
        if not model.history:
            return
        h = np.array(model.history)
        n = np.arange(1, len(h) + 1)
        ax_k.plot(n, h[:, 1], color=color, lw=1.5, label=label)
        ax_b.plot(n, h[:, 2], color=color, lw=1.5, label=label)
        ax_p.semilogy(n, h[:, 3], color=color, lw=1.5, label=label)

    _plot(surge_conv, 'Surge-trained',   COLORS['surge'])
    _plot(zermatt,    'Zermatt-trained', COLORS['zermatt'])

    for ax, val, name in [(ax_k, K_PRIOR, 'k prior'), (ax_b, B_PRIOR, 'b prior')]:
        ax.axhline(val, color='gray', ls=':', lw=1, label=f'{name}={val}')

    for i, bb in enumerate(bounds[1:-1]):
        for ax in (ax_k, ax_b, ax_p):
            ax.axvline(bb, color=COLORS['surge'], ls='--', alpha=0.35, lw=1)
        ax_k.text(bb + 2, ax_k.get_ylim()[1] * 0.98, f'bag {i+2}',
                  color=COLORS['surge'], fontsize=8, va='top')

    ax_k.set_ylabel('k  (m/s)');  ax_k.legend(fontsize=10)
    ax_b.set_ylabel('b  (–)');    ax_b.legend(fontsize=10)
    ax_p.set_ylabel('trace(P)');  ax_p.legend(fontsize=10)
    ax_p.set_xlabel('RLS update index')
    ax_k.set_title('Fig 4 — RLS Parameter Convergence')

    plt.tight_layout()
    out = ANALYSIS_DIR / 'fig4_convergence.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Saved {out}')
    plt.show()


def fig5_summary(vd, pd_):
    labels   = ['IMU-only', 'Static\nprior', 'Surge-\ntrained', 'Zermatt-\ntrained', 'DVL DR\n(ref)']
    vel_rmse = [float('nan'), vd['r_static'], vd['r_surge'], vd['r_zermatt'], float('nan')]
    pos_rmse = [rmse(pd_['e_imu']), rmse(pd_['e_static']), rmse(pd_['e_surge']),
                rmse(pd_['e_zermatt']), rmse(pd_['e_dvl_dr'])]
    colors   = [COLORS['imu'], COLORS['static'], COLORS['surge'], COLORS['zermatt'], COLORS['gt']]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    x = np.arange(len(labels));  w = 0.55

    bars = ax1.bar(x, vel_rmse, width=w, color=colors, edgecolor='white', linewidth=0.5)
    for bar, val in zip(bars, vel_rmse):
        if not np.isnan(val):
            ax1.text(bar.get_x() + bar.get_width() / 2, val + 0.001,
                     f'{val:.4f}', ha='center', va='bottom', fontsize=9)
    ax1.set_xticks(x); ax1.set_xticklabels(labels)
    ax1.set_ylabel('Velocity RMSE (m/s)')
    ax1.set_title(f'Velocity error\n({EVAL_VEL.name})')

    bars = ax2.bar(x, pos_rmse, width=w, color=colors, edgecolor='white', linewidth=0.5)
    for bar, val in zip(bars, pos_rmse):
        ax2.text(bar.get_x() + bar.get_width() / 2, val + 0.02,
                 f'{val:.2f}', ha='center', va='bottom', fontsize=9)
    ax2.set_xticks(x); ax2.set_xticklabels(labels)
    ax2.set_ylabel('Position RMSE (m)')
    ax2.set_title(f'Position error vs EKF GT\n({EVAL_POS.name})')

    plt.suptitle('Fig 5 — Error Summary', fontsize=13, y=1.01)
    plt.tight_layout()
    out = ANALYSIS_DIR / 'fig5_summary.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Saved {out}')
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    static, surge, zermatt, bag_boundaries, _ = train_models()
    vd  = load_velocity_eval(static, surge, zermatt)
    pd_ = load_position_eval(static, surge, zermatt)

    fig1_velocity(vd)
    fig2_position_drift(pd_)
    fig3_trajectory(pd_)
    fig4_convergence(surge, zermatt, bag_boundaries)
    fig5_summary(vd, pd_)

    print('\nAll figures saved to', ANALYSIS_DIR)


if __name__ == '__main__':
    main()

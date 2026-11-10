#!/usr/bin/env python3
"""A1.2: vyhodnotenie indoor misie z rosbagu (graf + suhrn).

  ros2 run lrs_control plot_indoor.py <bag_adresar_alebo_.mcap> [-o vystup] [--show]

Z bagu cita (MAVROS local ENU, prevedene do ramca mapy: map = R(-90 deg) * local):
  /mavros/local_position/pose    skutocna poloha + yaw
  /mavros/setpoint_raw/local     prikazana poloha + yaw (co posiela indoor_mission)
  /lrs/viz/plan                  waypointy misie a naplanovane useky (A*)
  /lrs/viz/map                   steny (voxely) a regaly (boxy)
  /rosout                        prechody stavoveho automatu a dosiahnute waypointy

Vystup (PNG do -o, predvolene vedla bagu):
  indoor_xy.png    pohlad zhora: steny, regaly, planovana a preletena trasa, smer nosa
  indoor_time.png  rychlost, vyska a odchylka nosa od smeru letu v case
"""
import argparse
import math
import os
import re
import sys

import numpy as np

# farby a tokeny (rovnake ako plot_outdoor.py)
C_FLOWN = '#2a78d6'      # skutocna (modra)
C_CMD = '#eb6834'        # prikazana / planovana (oranzova)
C_RACK = '#d9c3bd'       # regal (plny box)
C_RACK_EDGE = '#a8817a'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
WALL = '#b9b8ae'
SURFACE = '#fcfcfb'

RE_STATE = re.compile(r'\[t=\s*([\d.]+)s\] WP (\d+)/(\d+)\s+(\w+) -> (\w+)\s+\((.*)\)')
RE_ARRIVE = re.compile(r'WP(\d+) dosiahnuty, chyba ([\d.]+) m, v=([\d.]+) m/s')
RE_LABEL = re.compile(r'WP(\d+) \(.*\) (soft|hard) (\S+)')


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------------------
# nacitanie dat
# ---------------------------------------------------------------------------
TOPICS = ['/mavros/local_position/pose', '/mavros/setpoint_raw/local', '/lrs/viz/plan',
          '/lrs/viz/map', '/rosout']


def iter_bag(path):
    """(topic, sprava, cas [s]) z bagu; rosbag2_py, ak chyba, kniznica mcap."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        yield from iter_mcap(path)
        return
    sid = 'mcap' if path.endswith('.mcap') or os.path.isdir(path) and any(
        f.endswith('.mcap') for f in os.listdir(path)) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for w in TOPICS:
        if w not in types:
            print(f'POZOR: tema {w} v bagu nie je', file=sys.stderr)
    cls = {t: get_message(types[t]) for t in TOPICS if t in types}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in cls:
            yield topic, deserialize_message(raw, cls[topic]), t_ns * 1e-9


def iter_mcap(path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    if os.path.isdir(path):
        path = os.path.join(path, sorted(f for f in os.listdir(path) if f.endswith('.mcap'))[0])
    with open(path, 'rb') as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, msg, dec in r.iter_decoded_messages(topics=TOPICS):
            yield ch.topic, dec, msg.log_time * 1e-9


def read_bag(path, tf_yaw_deg):
    c, s = math.cos(math.radians(tf_yaw_deg)), math.sin(math.radians(tf_yaw_deg))
    to_map = lambda x, y: (c * x + s * y, -s * x + c * y)   # noqa: E731  (FrameTransform::toMap)
    tfy = math.radians(tf_yaw_deg)
    pose, sp, states, arrivals = [], [], [], []
    plan, mapm = None, None
    for topic, m, t in iter_bag(path):
        if topic == '/mavros/local_position/pose':
            p = m.pose.position
            x, y = to_map(p.x, p.y)
            pose.append((t, x, y, p.z, yaw_from_quat(m.pose.orientation) - tfy))
        elif topic == '/mavros/setpoint_raw/local':
            x, y = to_map(m.position.x, m.position.y)
            sp.append((t, x, y, m.position.z, m.yaw - tfy))
        elif topic == '/lrs/viz/plan':
            plan = m          # posledny = s naplanovanymi usekmi
        elif topic == '/lrs/viz/map':
            mapm = m
        elif topic == '/rosout' and m.name.endswith('indoor_mission'):
            ms = RE_STATE.search(m.msg)
            if ms:
                states.append((t, ms.group(4), ms.group(5), int(ms.group(2)), ms.group(6)))
            ma = RE_ARRIVE.search(m.msg)
            if ma:
                arrivals.append((t, int(ma.group(1)), float(ma.group(2)), float(ma.group(3))))
    if not pose:
        sys.exit('ziadne data z /mavros/local_position/pose')
    if plan is None:
        sys.exit('v bagu chyba /lrs/viz/plan')

    wps, labels, segs = {}, {}, {}
    for mk in plan.markers:
        if mk.ns == 'waypointy':
            q = mk.pose.position
            wps[mk.id] = (q.x, q.y, q.z)
        elif mk.ns == 'popisy':
            ml = RE_LABEL.search(mk.text)
            if ml:
                labels[mk.id] = (ml.group(2), ml.group(3))
        elif mk.ns == 'cesta':
            segs[mk.id] = np.array([(q.x, q.y, q.z) for q in mk.points])
    walls, racks, infl = np.zeros((0, 3)), [], 0.0
    if mapm is not None:
        for mk in mapm.markers:
            if mk.ns == 'voxely':
                walls = np.array([(q.x, q.y, q.z) for q in mk.points])
            elif mk.ns == 'regaly':
                q, sc = mk.pose.position, mk.scale
                racks.append(((q.x - sc.x / 2, q.y - sc.y / 2, q.z - sc.z / 2),
                              (q.x + sc.x / 2, q.y + sc.y / 2, q.z + sc.z / 2)))
            elif mk.ns == 'regaly_inflacia' and racks:
                infl = (mk.scale.x - (racks[-1][1][0] - racks[-1][0][0])) / 2
    d = {
        'pose': np.array(pose), 'sp': np.array(sp) if sp else np.zeros((0, 5)),
        'states': states, 'arrivals': arrivals,
        'wps': [wps[i] for i in sorted(wps)],
        'labels': [labels.get(i, ('soft', '-')) for i in sorted(wps)],
        'segs': [segs[i] for i in sorted(segs)], 'walls': walls, 'racks': racks, 'infl': infl,
    }
    d['t0'] = states[0][0] if states else d['pose'][0, 0]
    return d


# ---------------------------------------------------------------------------
# vyhodnotenie
# ---------------------------------------------------------------------------
def state_at(d, t):
    """nazov stavu automatu v casoch t (podla prechodov v /rosout)."""
    st = d['states']
    if not st:
        return np.array(['?'] * len(t))
    times = np.array([s[0] for s in st])
    names = np.array([st[0][1]] + [s[2] for s in st])
    return names[np.searchsorted(times, t, side='right')]


def dist_to_polyline(p, poly):
    """najmensia 3D vzdialenost bodov p (N x 3) od lomenej ciary poly (M x 3)."""
    best = np.full(len(p), np.inf)
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        L2 = float(ab @ ab)
        u = np.zeros(len(p)) if L2 < 1e-12 else np.clip(((p - a) @ ab) / L2, 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(p - (a + u[:, None] * ab), axis=1))
    return best


def dist_to_box(p, lo, hi):
    lo, hi = np.array(lo), np.array(hi)
    return np.linalg.norm(np.maximum(np.maximum(lo - p, 0.0), p - hi), axis=1)


def analyse(d):
    P = d['pose']
    t, x, y, z, yaw = P.T
    r = {}
    # rychlost a smer letu z polohy (stredova diferencia)
    vx, vy = np.gradient(x, t), np.gradient(y, t)
    r['vxy'] = np.hypot(vx, vy)
    travel = np.degrees(np.arctan2(vy, vx))
    r['hdg_err'] = np.abs(wrap_deg(np.degrees(yaw) - travel))
    r['state'] = state_at(d, t)
    # let medzi bodmi = NAVIGATE a ARRIVE. Otacanie na mieste (ROTATE), yaw uloha,
    # vzlet a pristatie pravidlo 4 vynima, tie sa do odchylky nosa nezapocitavaju.
    travel_st = np.isin(r['state'], ['NAVIGATE', 'ARRIVE'])
    flight_st = np.isin(r['state'], ['NAVIGATE', 'ARRIVE', 'ROTATE', 'TASK_YAW'])
    moving = travel_st & (r['vxy'] > 0.3)
    r['moving'] = moving
    dt = np.gradient(t)
    if moving.any():
        e = r['hdg_err'][moving]
        r['hdg_max'], r['hdg_mean'] = float(e.max()), float(e.mean())
        r['hdg_p95'] = float(np.percentile(e, 95))
        r['hdg_t30'] = float(dt[moving][e > 30.0].sum())
        r['t_moving'] = float(dt[moving].sum())
    # odchylka od naplanovanej cesty a vzdialenost od prekazok: len let misie
    # (pri vzlete a pristati je dron mimo usekov a blizko podlahy)
    pts = P[flight_st][:, 1:4]
    if d['segs'] and len(pts):
        dev = np.min([dist_to_polyline(pts, s) for s in d['segs'] if len(s) >= 2], axis=0)
        r['dev_max'], r['dev_rms'] = float(dev.max()), float(np.sqrt(np.mean(dev ** 2)))
    # najmensia vzdialenost od stien (voxely) a od regalov
    if len(d['walls']) and len(pts):
        W = d['walls']
        md = np.inf
        for k in range(0, len(pts), 200):
            q = pts[k:k + 200]
            dd = np.sqrt(((q[:, None, :] - W[None, :, :]) ** 2).sum(axis=2)).min()
            md = min(md, dd)
        r['wall_min'] = float(md - 0.05)    # stred voxelu -> jeho povrch (polovica 0.10 m)
    if d['racks'] and len(pts):
        r['rack_min'] = float(min(dist_to_box(pts, lo, hi).min() for lo, hi in d['racks']))
    r['duration'] = (d['states'][-1][0] - d['t0']) if d['states'] else t[-1] - t[0]
    r['z_max'] = float(z.max())
    r['pose_hz'] = (len(t) - 1) / (t[-1] - t[0])
    return r


def summary(d, r):
    print('\n=== A1.2 indoor misia: suhrn ===')
    if d['states']:
        print(f"koniec v stave {d['states'][-1][2]}, cas {r['duration']:.1f} s")
    rot = [s for s in d['states'] if s[2] == 'ROTATE']
    corner = [s for s in rot if 'v medzibode' in s[4]]
    print(f"otocenia na mieste: {len(rot)} (z toho v zakrutach medzi bodmi {len(corner)})")
    print('WP  presnost  uloha         chyba[m]  rychlost[m/s]')
    for _, i, err, v in d['arrivals']:
        prec, task = d['labels'][i - 1] if i - 1 < len(d['labels']) else ('?', '?')
        print(f'{i:<3} {prec:<9} {task:<13} {err:8.3f}  {v:8.2f}')
    if 'hdg_max' in r:
        print(f"nos vs. smer letu (pri > 0.3 m/s, {r['t_moving']:.0f} s letu): max {r['hdg_max']:.0f} deg, "
              f"95 % {r['hdg_p95']:.0f} deg, priemer {r['hdg_mean']:.1f} deg, nad 30 deg {r['hdg_t30']:.1f} s")
    if 'dev_max' in r:
        print(f"odchylka od naplanovanej cesty: max {r['dev_max']:.2f} m, RMS {r['dev_rms']:.2f} m")
    if 'wall_min' in r:
        print(f"najmensia vzdialenost stredu drona od steny {r['wall_min']:.2f} m")
    if 'rack_min' in r:
        print(f"najmensia vzdialenost stredu drona od regalu {r['rack_min']:.2f} m "
              f"(inflacia {d['infl']:.2f} m)")
    print(f"max vyska {r['z_max']:.2f} m, poloha v bagu {r['pose_hz']:.1f} Hz")


# ---------------------------------------------------------------------------
# grafy
# ---------------------------------------------------------------------------
def style(plt):
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': AXIS, 'axes.labelcolor': INK2, 'axes.titlecolor': INK,
        'axes.titlesize': 12, 'axes.labelsize': 10, 'axes.grid': True,
        'grid.color': GRID, 'grid.linewidth': 0.8, 'grid.linestyle': '-',
        'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelcolor': INK2,
        'ytick.labelcolor': INK2, 'legend.frameon': False, 'legend.labelcolor': INK2,
        'lines.linewidth': 1.8, 'lines.solid_capstyle': 'round', 'font.size': 10,
    })
    for s in ('top', 'right'):
        plt.rcParams[f'axes.spines.{s}'] = False


def plot_xy(plt, d, r, out):
    from matplotlib.patches import Rectangle
    from matplotlib.lines import Line2D
    P = d['pose']
    air = P[:, 3] > 0.3
    fig, ax = plt.subplots(figsize=(12, 7.2))
    ax.set_axisbelow(True)
    # steny vo vyske letu, premietnute zhora
    W = d['walls']
    if len(W):
        band = W[(W[:, 2] > 0.3) & (W[:, 2] < 3.0)]
        cells = np.unique(np.round(band[:, :2] / 0.1).astype(int), axis=0) * 0.1
        ax.scatter(cells[:, 0], cells[:, 1], s=1.2, marker='s', color=WALL, lw=0, zorder=1)
    for lo, hi in d['racks']:
        ax.add_patch(Rectangle(lo[:2], hi[0] - lo[0], hi[1] - lo[1], facecolor=C_RACK,
                               edgecolor=C_RACK_EDGE, lw=1.0, zorder=2))
        if d['infl'] > 0:
            g = d['infl']
            ax.add_patch(Rectangle((lo[0] - g, lo[1] - g), hi[0] - lo[0] + 2 * g, hi[1] - lo[1] + 2 * g,
                                   fill=False, edgecolor=C_RACK_EDGE, lw=0.9, ls=(0, (4, 3)), zorder=2))
    for k, s in enumerate(d['segs']):
        ax.plot(s[:, 0], s[:, 1], color=C_CMD, lw=1.8, ls=(0, (5, 3)), zorder=3)
        if len(s) > 2:
            ax.plot(s[1:-1, 0], s[1:-1, 1], 'o', ms=4, color=C_CMD, zorder=3)
    ax.plot(P[air, 1], P[air, 2], color=C_FLOWN, lw=1.8, zorder=4)
    # smer nosa: sipka kazde ~2 s letu
    mv = np.where(r['moving'])[0]
    last = -1e9
    for i in mv:
        if P[i, 0] - last >= 2.0:
            last = P[i, 0]
            ax.annotate('', xy=(P[i, 1] + 0.45 * math.cos(P[i, 4]), P[i, 2] + 0.45 * math.sin(P[i, 4])),
                        xytext=(P[i, 1], P[i, 2]), zorder=5,
                        arrowprops=dict(arrowstyle='-|>', color=INK2, lw=1.1, mutation_scale=9))
    offs = [(10, 8), (10, -14), (-34, 8), (10, 8), (10, -14), (-34, -14), (10, 8)]
    for i, ((wx, wy, _), (prec, task)) in enumerate(zip(d['wps'], d['labels'])):
        hard = prec == 'hard'
        ax.plot(wx, wy, 'o', ms=9, mfc=INK if hard else SURFACE, mec=INK, mew=1.6, zorder=6)
        name = f'WP{i + 1}' + ('' if task == '-' else f' {task}')
        ax.annotate(name, (wx, wy), xytext=offs[i % len(offs)], textcoords='offset points',
                    color=INK, fontsize=9, zorder=7)
    handles = [
        Line2D([], [], color=C_CMD, ls=(0, (5, 3)), marker='o', ms=4, label='planovana cesta (A*, po skrateni)'),
        Line2D([], [], color=C_FLOWN, label='preletena (local_position)'),
        Line2D([], [], color=INK2, marker=r'$\rightarrow$', ls='none', ms=12, label='smer nosa (kazde 2 s)'),
        Line2D([], [], color=INK, marker='o', mfc=INK, ls='none', ms=8, label='hard waypoint'),
        Line2D([], [], color=INK, marker='o', mfc=SURFACE, ls='none', ms=8, label='soft waypoint'),
        Rectangle((0, 0), 1, 1, facecolor=C_RACK, edgecolor=C_RACK_EDGE, label='regal (plny box)'),
        Line2D([], [], color=C_RACK_EDGE, ls=(0, (4, 3)), label=f"regal + inflacia {d['infl']:.1f} m"),
        Line2D([], [], color=WALL, marker='s', ls='none', ms=4, label='steny (voxely 0.3 az 3 m)'),
    ]
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=9)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x mapy [m]  (v hangari sever)')
    ax.set_ylabel('y mapy [m]  (v hangari zapad)')
    ax.set_title('Indoor misia zhora: planovana vs. preletena trasa')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'indoor_xy.png'), dpi=150)
    return fig


def bands(ax, d, t0, names, color):
    st = d['states']
    for k, s in enumerate(st):
        if s[2] in names:
            a = s[0] - t0
            b = (st[k + 1][0] if k + 1 < len(st) else s[0]) - t0
            ax.axvspan(a, b, color=color, alpha=0.5, lw=0, zorder=0)


def plot_time(plt, d, r, out):
    from matplotlib.patches import Patch
    P, S, t0 = d['pose'], d['sp'], d['t0']
    t = P[:, 0] - t0
    fig, axes = plt.subplots(3, 1, figsize=(12, 8.4), sharex=True)
    for ax in axes:
        bands(ax, d, t0, ('ROTATE', 'TASK_YAW'), GRID)
    # 1) rychlost + prichody na waypointy
    ax = axes[0]
    ax.plot(t, r['vxy'], color=C_FLOWN)
    for ta, i, err, v in d['arrivals']:
        hard = d['labels'][i - 1][0] == 'hard'
        ax.axvline(ta - t0, color=AXIS, lw=0.8, zorder=1)
        ax.text(ta - t0, 1.01, f'WP{i}', transform=ax.get_xaxis_transform(), ha='center', va='bottom',
                color=INK if hard else INK2, fontsize=8.5, fontweight='bold' if hard else 'normal')
    ax.set_ylabel('vodorovna rychlost [m/s]')
    ax.set_ylim(bottom=0)
    # 2) vyska
    ax = axes[1]
    if len(S):
        ax.step(S[:, 0] - t0, S[:, 3], where='post', color=C_CMD, lw=1.6, ls=(0, (5, 3)), label='setpoint z')
    ax.plot(t, P[:, 3], color=C_FLOWN, label='skutocna z')
    ax.set_ylabel('vyska z [m]')
    ax.set_ylim(bottom=0)
    # 3) nos vs. smer letu
    ax = axes[2]
    e = np.where(r['moving'], r['hdg_err'], np.nan)
    ax.plot(t, e, color=C_FLOWN, marker='o', ms=2.5, lw=1.2)
    ax.axhline(30, color=MUTED, lw=1.0, ls=(0, (2, 3)))
    ax.text(t[-1], 31, '30 deg', color=INK2, fontsize=8.5, ha='right', va='bottom')
    ax.set_ylabel('nos vs. smer letu [deg]')
    ax.set_ylim(0, 180)
    ax.set_yticks([0, 30, 60, 90, 120, 150, 180])
    ax.set_xlabel('cas od startu uzla [s]')
    h, _ = axes[1].get_legend_handles_labels()
    fig.legend(handles=h + [Patch(color=GRID, alpha=0.5, label='otacanie na mieste (ROTATE, yaw uloha)')],
               loc='upper center', ncol=3, bbox_to_anchor=(0.5, 0.955), fontsize=9)
    fig.suptitle('Indoor misia v case (nad grafom prichody na WP, tucne = hard)', color=INK, fontsize=12)
    axes[2].set_xlim(0, t[-1])
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(out, 'indoor_time.png'), dpi=150)
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('bag', help='adresar rosbagu alebo .mcap subor')
    ap.add_argument('-o', '--out', help='adresar pre PNG (predvolene vedla bagu)')
    ap.add_argument('--tf-yaw', type=float, default=90.0,
                    help='local_from_map_yaw_deg z indoor.yaml (predvolene 90)')
    ap.add_argument('--show', action='store_true', help='zobrazit grafy v okne')
    a = ap.parse_args()

    src = a.bag.rstrip('/')
    d = read_bag(src, a.tf_yaw)
    r = analyse(d)
    summary(d, r)

    import matplotlib
    if not a.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    style(plt)
    out = a.out or (src if os.path.isdir(src) else os.path.dirname(os.path.abspath(src)))
    os.makedirs(out, exist_ok=True)
    plot_xy(plt, d, r, out)
    plot_time(plt, d, r, out)
    print(f'grafy: {out}/indoor_{{xy,time}}.png')
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()

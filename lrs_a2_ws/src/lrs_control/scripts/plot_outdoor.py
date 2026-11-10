#!/usr/bin/env python3
"""A1.3: preletena vs. prikazana trajektoria outdoor misie (graf + suhrn).

  ros2 run lrs_control plot_outdoor.py <bag_adresar> [-o vystup] [--show]
  ros2 run lrs_control plot_outdoor.py --csv sim.csv   # zaznam z outdoor_sim (SIM_CSV=...)

Z bagu cita (vsetko v MAVROS local ENU, x = vychod, y = sever, yaw 0 = vychod, CCW):
  /mavros/local_position/pose      skutocna poloha + yaw
  /mavros/setpoint_raw/local       prikazana poloha + yaw (co posiela outdoor_mission)
  /lrs/outdoor/state               faza misie (SQUARE:1 .. SQUARE:4, FIG8, ...)
  /lrs/outdoor/poi                 POI
  /mavros/global_position/rel_alt  vyska nad home

Vystup (PNG do -o, predvolene vedla bagu):
  outdoor_xy.png    pohlad zhora: cela misia + detail osmicky
  outdoor_poi.png   chyba natocenia na POI na stvorci
  outdoor_fig8.png  osmicka: yaw vs. tangenta, odchylka polohy od setpointu
  outdoor_alt.png   vyska nad miestom vzletu (local z) a rel_alt
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

# farby a tokeny (svetly povrch, tlac do dokumentacie)
C_FLOWN = '#2a78d6'      # skutocna (modra)
C_CMD = '#eb6834'        # prikazana (oranzova)
C_ALT2 = '#1baf7a'       # druha vyska (rel_alt)
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
SURFACE = '#fcfcfb'

SQUARE_SIDES = ('SQUARE:1', 'SQUARE:2', 'SQUARE:3', 'SQUARE:4')
SIDE_NAMES = {'SQUARE:1': 'W1-W2', 'SQUARE:2': 'W2-W3', 'SQUARE:3': 'W3-W4', 'SQUARE:4': 'W4-W1'}


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------------------
# nacitanie dat
# ---------------------------------------------------------------------------
def storage_id(path):
    if path.endswith('.mcap'):
        return 'mcap'
    if path.endswith('.db3'):
        return 'sqlite3'
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.isfile(meta):
        import yaml
        with open(meta) as f:
            info = yaml.safe_load(f)
        return info['rosbag2_bagfile_information'].get('storage_identifier', '')
    return ''


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id(path)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ['/mavros/local_position/pose', '/mavros/setpoint_raw/local', '/lrs/outdoor/state',
              '/lrs/outdoor/poi', '/mavros/global_position/rel_alt']
    for w in wanted:
        if w not in types:
            print(f'POZOR: tema {w} v bagu nie je', file=sys.stderr)
    cls = {t: get_message(types[t]) for t in wanted if t in types}

    pose, sp, rel, phases, poi = [], [], [], [], None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in cls:
            continue
        m = deserialize_message(raw, cls[topic])
        t = t_ns * 1e-9
        if topic == '/mavros/local_position/pose':
            p = m.pose.position
            pose.append((t, p.x, p.y, p.z, yaw_from_quat(m.pose.orientation)))
        elif topic == '/mavros/setpoint_raw/local':
            sp.append((t, m.position.x, m.position.y, m.position.z, m.yaw))
        elif topic == '/mavros/global_position/rel_alt':
            rel.append((t, m.data))
        elif topic == '/lrs/outdoor/state':
            phases.append((t, m.data))
        elif topic == '/lrs/outdoor/poi':
            poi = (m.point.x, m.point.y)
    return build(pose, sp, rel, phases, poi)


def read_csv(path):
    pose, sp, rel, phases, poi = [], [], [], [], None
    last = None
    with open(path) as f:
        for r in csv.DictReader(f):
            t = float(r['t'])
            pose.append((t, float(r['x']), float(r['y']), float(r['z']), float(r['yaw'])))
            if r['sp_valid'] == '1':
                sp.append((t, float(r['sp_x']), float(r['sp_y']), float(r['sp_z']),
                           float(r['sp_yaw'])))
            rel.append((t, float(r['rel_alt'])))
            if r['phase'] != last:
                phases.append((t, r['phase']))
                last = r['phase']
            poi = (float(r['poi_x']), float(r['poi_y']))
    return build(pose, sp, rel, phases, poi)


def build(pose, sp, rel, phases, poi):
    if not pose:
        sys.exit('ziadne data z /mavros/local_position/pose')
    if not sp:
        sys.exit('ziadne data z /mavros/setpoint_raw/local (nahraval sa bag pocas letu?)')
    d = {'pose': np.array(pose), 'sp': np.array(sp),
         'rel': np.array(rel) if rel else np.zeros((0, 2)), 'phases': phases, 'poi': poi}
    d['t0'] = d['pose'][0, 0]
    return d


def intervals(d, names):
    """[(t_od, t_do, nazov)] pre fazy z `names`."""
    ph = d['phases']
    t_end = d['pose'][-1, 0]
    out = []
    for i, (t, n) in enumerate(ph):
        if n in names:
            out.append((t, ph[i + 1][0] if i + 1 < len(ph) else t_end, n))
    return out


def mask(t, ivs):
    m = np.zeros(len(t), dtype=bool)
    for a, b, _ in ivs:
        m |= (t >= a) & (t < b)
    return m


def sp_at(d, t):
    """prikazany setpoint platny v casoch t (posledny poslany pred t)."""
    sp = d['sp']
    i = np.clip(np.searchsorted(sp[:, 0], t, side='right') - 1, 0, len(sp) - 1)
    return sp[i]


# ---------------------------------------------------------------------------
# vyhodnotenie
# ---------------------------------------------------------------------------
def analyse(d):
    P = d['pose']
    t, x, y, z, yaw = P.T
    res = {}
    launch = P[0, 1:4]
    res['launch'] = launch
    res['r_max'] = float(np.max(np.hypot(x - launch[0], y - launch[1])))
    res['h_max'] = float(np.max(z - launch[2]))

    # stvorec: chyba natocenia na POI (strany W1-W2 .. W4-W1, bez presunu na W1)
    sq = intervals(d, SQUARE_SIDES)
    res['square_iv'] = sq
    if sq and d['poi'] is not None:
        m = mask(t, sq)
        px, py = d['poi']
        far = np.hypot(px - x, py - y) > 0.5
        m &= far
        bearing = np.degrees(np.arctan2(py - y[m], px - x[m]))
        err = wrap_deg(bearing - np.degrees(yaw[m]))
        res['poi_t'] = t[m]
        res['poi_err'] = err
        res['poi_max'] = float(np.max(np.abs(err)))
        res['poi_rms'] = float(np.sqrt(np.mean(err ** 2)))

    # osmicka: skutocna poloha vs. setpoint v tom istom case, yaw vs. tangenta
    f8 = intervals(d, ('FIG8',))
    res['fig8_iv'] = f8
    if f8:
        m = mask(t, f8)
        s = sp_at(d, t[m])
        dx, dy, dz = x[m] - s[:, 1], y[m] - s[:, 2], z[m] - s[:, 3]
        track = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        yerr = wrap_deg(np.degrees(s[:, 4] - yaw[m]))
        # unwrap oboch kriviek a posun prikazanej o k*360, aby zacinali v tej istej vetve
        y_act = np.degrees(np.unwrap(yaw[m]))
        y_cmd = np.degrees(np.unwrap(np.radians(wrap_deg(np.degrees(s[:, 4])))))
        y_cmd += 360.0 * np.round((y_act[0] - y_cmd[0]) / 360.0)
        res.update(f8_t=t[m], f8_track=track, f8_yerr=yerr, f8_yaw=y_act, f8_cmd=y_cmd,
                   f8_track_max=float(track.max()), f8_track_rms=float(np.sqrt(np.mean(track ** 2))),
                   f8_y_max=float(np.max(np.abs(yerr))), f8_y_rms=float(np.sqrt(np.mean(yerr ** 2))),
                   f8_dur=float(f8[-1][1] - f8[0][0]))
        tt = t[m]
        yy = np.unwrap(yaw[m])
        if len(tt) > 2:
            rate = np.abs(np.diff(yy) / np.maximum(np.diff(tt), 1e-3))
            res['f8_rate_max'] = float(np.degrees(np.percentile(rate, 99)))
        if d['poi'] is not None:
            res['f8_ext'] = (x[m].min() - d['poi'][0], x[m].max() - d['poi'][0],
                             y[m].min() - d['poi'][1], y[m].max() - d['poi'][1])
    return res


def summary(d, r):
    print('\n=== A1.3 outdoor misia: suhrn ===')
    print('fazy: ' + ' -> '.join(n for _, n in d['phases']))
    if 'poi_max' in r:
        print(f"stvorec   chyba natocenia na POI: max {r['poi_max']:.1f} deg, "
              f"RMS {r['poi_rms']:.1f} deg ({len(r['poi_err'])} vzoriek)")
    else:
        print('stvorec   v zazname nie je (chyba /lrs/outdoor/state alebo POI)')
    if 'f8_track_max' in r:
        print(f"osmicka   {r['f8_dur']:.1f} s, odchylka polohy od setpointu max "
              f"{r['f8_track_max']:.2f} m, RMS {r['f8_track_rms']:.2f} m")
        print(f"          yaw vs. tangenta max {r['f8_y_max']:.1f} deg, RMS {r['f8_y_rms']:.1f} deg"
              + (f", max rychlost otacania {r['f8_rate_max']:.0f} deg/s" if 'f8_rate_max' in r else ''))
        if 'f8_ext' in r:
            e = r['f8_ext']
            print(f"          rozmery od POI: E {e[0]:.2f} .. {e[1]:.2f} m, N {e[2]:.2f} .. {e[3]:.2f} m")
    else:
        print('osmicka   v zazname nie je')
    print(f"geofence  max. {r['r_max']:.2f} m od miesta vzletu, max. vyska {r['h_max']:.2f} m")


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


def phase_bands(ax, d, r, t0):
    """jemne pasy za stvorcom a osmickou + popis nad grafom."""
    for ivs, label in ((r.get('square_iv', []), 'stvorec'), (r.get('fig8_iv', []), 'osmicka')):
        if ivs:
            a, b = ivs[0][0] - t0, ivs[-1][1] - t0
            ax.axvspan(a, b, color=GRID, alpha=0.45, lw=0, zorder=0)
            ax.text((a + b) / 2, 1.01, label, transform=ax.get_xaxis_transform(),
                    ha='center', va='bottom', color=INK2, fontsize=9)


def plot_xy(plt, d, r, out):
    P, S = d['pose'], d['sp']
    armed_from = S[0, 0]
    fl = P[P[:, 0] >= armed_from]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={'width_ratios': [1.15, 1]})
    for k, ax in enumerate(axes):
        if k == 1 and r.get('fig8_iv'):
            m = mask(P[:, 0], r['fig8_iv'])
            ms = mask(S[:, 0], r['fig8_iv'])
            px, py, sx, sy = P[m, 1], P[m, 2], S[ms, 1], S[ms, 2]
            ax.set_title('Detail osmicky')
        elif k == 1:
            ax.set_visible(False)
            continue
        else:
            px, py, sx, sy = fl[:, 1], fl[:, 2], S[:, 1], S[:, 2]
            ax.set_title('Cela misia (pohlad zhora)')
        ax.plot(sx, sy, color=C_CMD, ls=(0, (5, 3)), lw=1.8, label='prikazana (setpoint)')
        ax.plot(px, py, color=C_FLOWN, lw=1.8, label='preletena (local_position)')
        if d['poi'] is not None:
            ax.plot(*d['poi'], marker='*', ms=14, color=INK, ls='none')
            ax.annotate('POI', d['poi'], xytext=(8, -14), textcoords='offset points', color=INK)
        if k == 0:
            L = r['launch']
            ax.plot(L[0], L[1], marker='^', ms=9, color=MUTED, ls='none')
            ax.annotate('start', (L[0], L[1]), xytext=(8, -4), textcoords='offset points',
                        color=INK2)
            # rohy stvorca = konce stran (setpoint na konci kazdej strany)
            for a, b, n in r.get('square_iv', []):
                c = sp_at(d, np.array([b - 1e-3]))[0]
                ax.annotate(SIDE_NAMES[n].split('-')[1], (c[1], c[2]), xytext=(6, 6),
                            textcoords='offset points', color=INK2, fontsize=9)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('vychod E [m]')
        ax.set_ylabel('sever N [m]')
    axes[0].legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'outdoor_xy.png'), dpi=150)
    return fig


def plot_poi(plt, d, r, out):
    if 'poi_err' not in r:
        return None
    t0 = r['poi_t'][0]
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(r['poi_t'] - t0, r['poi_err'], color=C_FLOWN)
    ax.axhline(0, color=AXIS, lw=1)
    for a, b, n in r['square_iv']:
        ax.axvline(a - t0, color=GRID, lw=1, zorder=0)
        ax.text((a + b) / 2 - t0, 1.01, SIDE_NAMES[n], transform=ax.get_xaxis_transform(),
                ha='center', va='bottom', color=INK2, fontsize=9)
    ax.set_title(f"Stvorec: nos drona vs. smer na POI  (max {r['poi_max']:.1f} deg, "
                 f"RMS {r['poi_rms']:.1f} deg)", pad=18)
    ax.set_xlabel('cas od zaciatku strany W1-W2 [s]')
    ax.set_ylabel('chyba natocenia [deg]')
    lim = max(5.0, math.ceil(r['poi_max'] + 1))
    ax.set_ylim(-lim, lim)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'outdoor_poi.png'), dpi=150)
    return fig


def plot_fig8(plt, d, r, out):
    if 'f8_t' not in r:
        return None
    t = r['f8_t'] - r['f8_t'][0]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    a1.plot(t, r['f8_cmd'], color=C_CMD, ls=(0, (5, 3)), label='prikazany yaw = tangenta')
    a1.plot(t, r['f8_yaw'], color=C_FLOWN, label='skutocny yaw')
    a1.set_ylabel('yaw ENU, unwrap [deg]')
    a1.set_title(f"Osmicka: natocenie (max odchylka {r['f8_y_max']:.1f} deg, "
                 f"RMS {r['f8_y_rms']:.1f} deg)")
    a1.legend(loc='best')
    a2.plot(t, r['f8_track'], color=C_FLOWN)
    a2.set_ylabel('odchylka od setpointu [m]')
    a2.set_xlabel('cas od startu osmicky [s]')
    a2.set_title(f"Osmicka: poloha (max {r['f8_track_max']:.2f} m, RMS {r['f8_track_rms']:.2f} m)")
    a2.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'outdoor_fig8.png'), dpi=150)
    return fig


def plot_alt(plt, d, r, out):
    P = d['pose']
    t0 = d['t0']
    fig, ax = plt.subplots(figsize=(11, 4))
    phase_bands(ax, d, r, t0)
    ax.plot(P[:, 0] - t0, P[:, 3] - r['launch'][2], color=C_FLOWN, label='local z - z vzletu (riadi sa)')
    if len(d['rel']):
        ax.plot(d['rel'][:, 0] - t0, d['rel'][:, 1], color=C_ALT2, ls=(0, (2, 2)),
                label='rel_alt (vyska nad home, kontrola)')
    ax.axhline(5.0, color=MUTED, lw=1, ls=(0, (6, 4)))
    ax.text(0.0, 5.0, ' 5 m', va='bottom', color=INK2, fontsize=9)
    ax.set_xlabel('cas [s]')
    ax.set_ylabel('vyska [m]')
    ax.set_title('Vyska nad miestom vzletu', pad=18)
    ax.legend(loc='lower center')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'outdoor_alt.png'), dpi=150)
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('bag', nargs='?', help='adresar rosbagu (ros2 bag record -o ...)')
    ap.add_argument('--csv', help='CSV z outdoor_sim (SIM_CSV=...)')
    ap.add_argument('-o', '--out', help='adresar pre PNG (predvolene vedla bagu/CSV)')
    ap.add_argument('--show', action='store_true', help='zobrazit grafy v okne')
    a = ap.parse_args()
    if not a.bag and not a.csv:
        ap.error('zadaj adresar bagu alebo --csv')

    src = a.csv or a.bag.rstrip('/')
    d = read_csv(a.csv) if a.csv else read_bag(src)
    r = analyse(d)
    summary(d, r)

    import matplotlib
    if not a.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    style(plt)
    out = a.out or (os.path.dirname(os.path.abspath(src)) if a.csv else src)
    os.makedirs(out, exist_ok=True)
    plot_xy(plt, d, r, out)
    plot_poi(plt, d, r, out)
    plot_fig8(plt, d, r, out)
    plot_alt(plt, d, r, out)
    print(f'grafy: {out}/outdoor_{{xy,poi,fig8,alt}}.png')
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()

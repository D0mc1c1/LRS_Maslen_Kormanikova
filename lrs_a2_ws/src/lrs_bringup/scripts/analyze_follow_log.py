#!/usr/bin/env python3
"""Vyhodnotenie testu §4 z logu follow.launch.py.

Pre kazdy dron z logu spocita:
  - kolkokrat presiel do TRACK (nasiel a potvrdil svoj rover),
  - kolkokrat rover stratil ("ciel strateny") a znova nasiel ("znova najdeny"),
  - kolko casu bol v ktorom stave (podiel TRACK po prvom najdeni),
  - ci niekedy reagoval na cudziu znacku (musi ju len ignorovat),
  - statistiku detektora (frekvencia snimok, detekcie, posun z oneskorenia).

§4 je pre dron splneny, ked: aspon raz TRACK, aspon jeden cyklus strata -> znova najdeny,
ziadny ABORT. (Zadanie: "aspon jeden skutocny cyklus strata -> opatovne najdenie pre kazdy dron".)

  ros2 run lrs_bringup analyze_follow_log.py bags/test_seed42_.../follow.log
  analyze_follow_log.py follow.log --out summary.txt
Navratovy kod 0 = oba drony PASS.
"""
import argparse
import re
import sys
from collections import defaultdict

# [follow_mission-1] [INFO] [1795350050.104155325] [drone_1.follow_mission]: SEARCH -> TRACK  (...)
LINE = re.compile(r'\[(?:INFO|WARN|ERROR)\]\s+\[(\d+\.\d+)\]\s+\[drone_(\d)\.(\w+)\]:\s*(.*)$')
TRANS = re.compile(r'^(\w+) -> (\w+)\s')
STATS = re.compile(r'snimky (\d+) \(([\d.]+) Hz\), vlastna znacka (\d+), cudzia (\d+).*?'
                   r'posun z oneskorenia ([\d.]+) m')


def analyze(lines):
    d = defaultdict(lambda: {
        'track': 0, 'lost': 0, 'refound': 0, 'abort': 0, 'foreign': 0,
        'stats': [], 'trans': [], 't_last': None, 'refound_after': []})
    for raw in lines:
        m = LINE.search(raw)
        if not m:
            continue
        t, n, node, text = float(m.group(1)), int(m.group(2)), m.group(3), m.group(4)
        r = d[n]
        r['t_last'] = t
        if node == 'follow_mission':
            tm = TRANS.match(text)
            if tm:
                r['trans'].append((t, tm.group(1), tm.group(2)))
                if tm.group(2) == 'TRACK':
                    r['track'] += 1
                if tm.group(2) == 'ABORT':
                    r['abort'] += 1
            if 'ciel strateny' in text:
                r['lost'] += 1
            ra = re.search(r'znova najdeny po ([\d.]+) s', text)
            if ra:
                r['refound'] += 1
                r['refound_after'].append(float(ra.group(1)))
        elif node == 'aruco_detector':
            if 'ignorujem cudziu' in text:
                r['foreign'] += 1
            sm = STATS.search(text)
            if sm:
                r['stats'].append(tuple(float(x) for x in sm.groups()))
    return d


def state_times(r):
    """Cas v jednotlivych stavoch od prveho prechodu po posledny riadok logu."""
    out = defaultdict(float)
    tr = r['trans']
    for i, (t, _, to) in enumerate(tr):
        t_end = tr[i + 1][0] if i + 1 < len(tr) else r['t_last']
        out[to] += max(0.0, t_end - t)
    return out


def report(d):
    lines, all_ok = [], True
    for n in (1, 2):
        lines.append(f'== drone_{n} ==')
        if n not in d:
            lines.append('  v logu nie je (drones:=? alebo uzly nenaštartovali)')
            all_ok = False
            continue
        r = d[n]
        st = state_times(r)
        lines.append(f'  TRACK (rover najdeny a potvrdeny): {r["track"]}x')
        lines.append(f'  strata cielu: {r["lost"]}x, znova najdeny: {r["refound"]}x'
                     + (f' (po {", ".join(f"{x:.1f}" for x in r["refound_after"])} s)'
                        if r['refound_after'] else ''))
        first_track = next((t for t, _, to in r['trans'] if to == 'TRACK'), None)
        if first_track is not None and r['t_last'] and r['t_last'] > first_track:
            after = defaultdict(float)
            tr = r['trans']
            for i, (t, _, to) in enumerate(tr):
                if t < first_track:
                    continue
                t_end = tr[i + 1][0] if i + 1 < len(tr) else r['t_last']
                after[to] += max(0.0, t_end - t)
            tot = sum(after.values()) or 1.0
            lines.append(f'  po prvom najdeni: TRACK {100 * after["TRACK"] / tot:.0f} % casu, '
                         f'SEARCH {100 * after["SEARCH"] / tot:.0f} % ({tot:.0f} s)')
        if st:
            lines.append('  cas v stavoch: ' + ', '.join(f'{k} {v:.0f} s' for k, v in st.items()))
        lines.append(f'  cudzia znacka ignorovana: {r["foreign"]}x (logy max. kazde 2 s)')
        if r['stats']:
            k = len(r['stats'])
            hz = sum(s[1] for s in r['stats']) / k
            own = sum(int(s[2]) for s in r['stats'])
            lag = sum(s[4] for s in r['stats']) / k
            lines.append(f'  detektor: priemerne {hz:.1f} snimok/s, vlastna znacka {own}x, '
                         f'posun z oneskorenia {lag:.3f} m')
        ok = r['track'] >= 1 and r['refound'] >= 1 and r['abort'] == 0
        why = []
        if r['track'] < 1:
            why.append('rover nikdy nenasiel')
        if r['track'] >= 1 and r['refound'] < 1:
            why.append('chyba cyklus strata -> znova najdeny (dlhsi test alebo iny seed)')
        if r['abort']:
            why.append(f'{r["abort"]}x ABORT')
        lines.append(('  PASS' if ok else '  FAIL: ' + ', '.join(why)))
        all_ok &= ok
    lines.append('')
    lines.append('§4 SPLNENY pre oba drony' if all_ok else '§4 zatial NESPLNENY (pozri FAIL vyssie)')
    return '\n'.join(lines), all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--out', default='', help='ulozit suhrn aj do suboru')
    a = ap.parse_args()
    with open(a.log, errors='replace') as f:
        text, ok = report(analyze(f))
    print(text)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

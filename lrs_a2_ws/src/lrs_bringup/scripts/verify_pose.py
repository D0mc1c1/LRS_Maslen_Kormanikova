#!/usr/bin/env python3
"""Overenie §3: odhad polohy roveru z detektora vs. skutocna poloha z Gazeba.

Porovnava /drone_N/target/pose (odhad detektora v lokalnom ENU drona) so skutocnou polohou
roveru /model/aruco_rover_N/odometry (Gazebo, ramec world), prepocitanou do lokalneho ramca
drona. Skutocna poloha sa pouziva LEN tu na overenie, nikdy v riadeni.

Prevod Gazebo -> lokalny ramec drona (plan 7.4): ArduPilot ma os x Gazeba ako sever, takze
  x_lok = -y_gz, y_lok = x_gz   (otocenie o +90°, --rot-deg 90, bez posunu)
Na zaciatku skript prevod overi: poloha drona na zemi z MAVROS sa musi zhodovat s miestom,
kde drony vo svete stoja, (-10,-10) a (10,-10) v Gazebe.

Pouzitie (bring-up s truth:=true a bezicou misiou alebo detektorom):
  ros2 run lrs_bringup verify_pose.py 1                # drone_1, Ctrl+C = suhrn
  ros2 run lrs_bringup verify_pose.py 2 --duration 120
  ros2 run lrs_bringup verify_pose.py 1 --csv ~/verify_d1.csv

Suhrn na konci:
  chyba polohy (median, priemer, max) -> §3 je splneny, ked median < 0.6 m (velkost znacky)
  posun (bias) x/y                    -> staly posun = zla montaz kamery alebo zly prevod ramca
  pomer vzdialenosti odhad/skutocnost -> != 1.0 = zla marker_size alebo intrinziky
"""
import argparse
import csv
import math
import os
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

# kde drony stoja vo svete (Gazebo, ramec world) -> kontrola prevodu ramcov
DRONE_START_GZ = {1: (-10.0, -10.0), 2: (10.0, -10.0)}
MARKER_SIZE = 0.638  # [m] kriterium §3: chyba mensia ako znacka


class VerifyPose(Node):
    def __init__(self, args):
        super().__init__(f'verify_pose_{args.drone}')
        self.args = args
        self.rot = math.radians(args.rot_deg)
        self.rover = None          # (x, y) lokalne, cas prijatia
        self.drone = None          # (x, y, z)
        self.frame_checked = False
        self.rows = []
        self.last_print = 0.0
        self.t0 = time.monotonic()
        n = args.drone
        self.create_subscription(
            PoseStamped, f'/drone_{n}/target/pose', self.on_target, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, f'/drone_{n}/mavros/local_position/pose', self.on_drone,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, f'/model/aruco_rover_{n}/odometry', self.on_rover, qos_profile_sensor_data)
        self.create_timer(5.0, self.on_watchdog)
        self.get_logger().info(
            f'porovnavam /drone_{n}/target/pose s /model/aruco_rover_{n}/odometry '
            f'(prevod: otocenie {args.rot_deg:.0f}°, posun {args.offset})')

    def gz_to_local(self, x, y):
        c, s = math.cos(self.rot), math.sin(self.rot)
        return (c * x - s * y + self.args.offset[0], s * x + c * y + self.args.offset[1])

    def on_drone(self, m):
        p = m.pose.position
        self.drone = (p.x, p.y, p.z)
        if not self.frame_checked:
            self.frame_checked = True
            ex, ey = self.gz_to_local(*DRONE_START_GZ[self.args.drone])
            d = math.hypot(p.x - ex, p.y - ey)
            msg = (f'kontrola ramca: drone_{self.args.drone} lokalne ({p.x:.2f}, {p.y:.2f}), '
                   f'z Gazeba prepocitane ({ex:.2f}, {ey:.2f}), rozdiel {d:.2f} m')
            if p.z > 1.0:
                self.get_logger().warn(msg + ' (dron leti, kontrola plati len na zemi)')
            elif d < 1.0:
                self.get_logger().info(msg + ' -> prevod sedi')
            else:
                self.get_logger().error(
                    msg + ' -> prevod NESEDI, skus ine --rot-deg / --offset (plan 7.4)')

    def on_rover(self, m):
        p = m.pose.pose.position
        self.rover = (*self.gz_to_local(p.x, p.y), time.monotonic())

    def on_target(self, m):
        if self.rover is None or self.drone is None:
            return
        rx, ry, rt = self.rover
        if time.monotonic() - rt > 0.5:
            return  # skutocna poloha je stara
        ex, ey = m.pose.position.x, m.pose.position.y
        dx, dy, dz = self.drone
        err = math.hypot(ex - rx, ey - ry)
        nadir = math.hypot(ex - dx, ey - dy)      # ako daleko od stredu obrazu (odhad)
        true_dist = math.hypot(rx - dx, ry - dy)  # skutocna vodorovna vzdialenost
        self.rows.append({
            't': round(time.monotonic() - self.t0, 2), 'drone_z': round(dz, 3),
            'est_x': round(ex, 3), 'est_y': round(ey, 3),
            'true_x': round(rx, 3), 'true_y': round(ry, 3),
            'err': round(err, 3), 'err_x': round(ex - rx, 3), 'err_y': round(ey - ry, 3),
            'est_dist': round(nadir, 3), 'true_dist': round(true_dist, 3)})
        now = time.monotonic()
        if now - self.last_print >= 1.0:
            self.last_print = now
            self.get_logger().info(
                f'odhad ({ex:6.2f}, {ey:6.2f})  skutocne ({rx:6.2f}, {ry:6.2f})  '
                f'chyba {err:4.2f} m  | vyska {dz:4.1f} m, od stredu obrazu {nadir:4.1f} m')

    def on_watchdog(self):
        missing = []
        if self.drone is None:
            missing.append(f'/drone_{self.args.drone}/mavros/local_position/pose')
        if self.rover is None:
            missing.append(f'/model/aruco_rover_{self.args.drone}/odometry '
                           '(bringup.launch.py truth:=true ?)')
        if missing:
            self.get_logger().warn('chyba: ' + ', '.join(missing))
        elif not self.rows:
            self.get_logger().info('zatial ziadna detekcia (dron este nevidi svoj rover)')
        if self.args.duration > 0 and time.monotonic() - self.t0 > self.args.duration:
            raise SystemExit

    def summary(self):
        rows = self.rows
        print()
        print(f'===== SUHRN drone_{self.args.drone}: {len(rows)} detekcii =====')
        if not rows:
            print('Ziadne detekcie, nie je co porovnat.')
            return 1
        errs = [r['err'] for r in rows]
        med = statistics.median(errs)
        print(f'chyba polohy:  median {med:.3f} m, priemer {statistics.mean(errs):.3f} m, '
              f'max {max(errs):.3f} m')
        bx = statistics.mean(r['err_x'] for r in rows)
        by = statistics.mean(r['err_y'] for r in rows)
        print(f'staly posun (bias): x {bx:+.3f} m, y {by:+.3f} m'
              + ('   <- velky posun: montaz kamery alebo prevod ramca' if math.hypot(bx, by) > 0.3
                 else '   (maly, v poriadku)'))
        over = [r['err'] for r in rows if r['true_dist'] < 0.5]
        if over:
            print(f'nad roverom (do 0.5 m): {len(over)} detekcii, median chyby '
                  f'{statistics.median(over):.3f} m')
        far = [r for r in rows if r['true_dist'] > 1.5]
        if len(far) >= 5:
            ratio = statistics.median(r['est_dist'] / r['true_dist'] for r in far)
            note = ('marker_size a intrinziky sedia' if abs(ratio - 1.0) < 0.08 else
                    'odhad je systematicky dalej/blizsie -> skontroluj marker_size')
            print(f'pomer vzdialenosti odhad/skutocnost: {ratio:.3f} ({len(far)} detekcii '
                  f'dalej ako 1.5 m od stredu) -> {note}')
        zs = [r['drone_z'] for r in rows]
        print(f'vyska drona pocas detekcii: {min(zs):.1f} az {max(zs):.1f} m')
        ok = med < MARKER_SIZE
        print(('PASS' if ok else 'FAIL') +
              f': median chyby {med:.2f} m {"<" if ok else ">="} velkost znacky {MARKER_SIZE} m')
        if self.args.csv:
            path = os.path.expanduser(self.args.csv)
            with open(path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print(f'CSV: {path}')
        return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('drone', type=int, choices=[1, 2])
    ap.add_argument('--rot-deg', type=float, default=90.0,
                    help='otocenie Gazebo -> lokalny ramec drona [°] (plan 7.4: 90)')
    ap.add_argument('--offset', type=float, nargs=2, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='posun po otoceni [m]')
    ap.add_argument('--duration', type=float, default=0.0, help='[s] 0 = do Ctrl+C')
    ap.add_argument('--csv', default='', help='ulozit vsetky detekcie do CSV')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = VerifyPose(args)
    code = 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:  # ExternalShutdownException pri Ctrl+C v novsom rclpy
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        code = node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

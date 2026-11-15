#!/usr/bin/env python3
"""Diagnostic RF2O (lecture seule) : quels scans /scan n'aboutissent pas à un /odom ?

Usage : scan_odom_logger.py [durée_s] [étiquette]
Un scan est ECHEC si aucun /odom ne porte exactement son header.stamp
(RF2O publie /odom avec le stamp du scan traité, uniquement si le calcul réussit).
"""
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
TAG = sys.argv[2] if len(sys.argv) > 2 else 'run'


def stamp_ns(s):
    return s.sec * 10**9 + s.nanosec


class Logger(Node):
    def __init__(self):
        super().__init__('rf2o_scan_odom_logger')
        self.scans = []
        self.odom_ns = set()
        self.prev = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, m):
        self.odom_ns.add(stamp_ns(m.header.stamp))

    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r > 0.0)            # critère de validité de RF2O
        row = {'ns': stamp_ns(m.header.stamp), 'n_valid': int(ok.sum()),
               'sectors': [int(c.sum()) for c in np.array_split(ok, 8)],
               'n_both': 0, 'frac_equal': float('nan'), 'mad': float('nan')}
        if self.prev is not None and len(self.prev[0]) == len(r):
            pr, pok = self.prev
            both = ok & pok
            n = int(both.sum())
            row['n_both'] = n
            if n:
                d = np.abs(r[both] - pr[both])
                row['frac_equal'] = float((d == 0.0).mean())
                row['mad'] = float(d.mean())
        self.prev = (r, ok)
        self.scans.append(row)

    def report(self):
        rows = self.scans[3:-2]     # 1er scan = init RF2O ; les derniers peuvent être en cours
        if len(rows) < 20:
            print(f'Pas assez de scans ({len(self.scans)}). /scan publié ?')
            return
        for s in rows:
            s['failed'] = s['ns'] not in self.odom_ns
        bad = [s for s in rows if s['failed']]
        good = [s for s in rows if not s['failed']]

        def m(lst, key):
            v = [s[key] for s in lst if s[key] == s[key]]      # exclut NaN
            return float(np.mean(v)) if v else float('nan')

        print(f'\n=== {TAG} : {len(rows)} scans, {len(bad)} sans /odom '
              f'({100.0 * len(bad) / len(rows):.1f} %) ===')
        print(f'{"":40s}{"OK":>10s}{"ECHEC":>10s}')
        for key, label in (('n_valid', 'points valides'),
                           ('frac_equal', 'part identique au scan précédent'),
                           ('mad', 'écart moyen au précédent (m)')):
            print(f'{label:40s}{m(good, key):10.4f}{m(bad, key):10.4f}')
        for name, lst in (('OK', good), ('ECHEC', bad)):
            if lst:
                sec = np.mean([s['sectors'] for s in lst], axis=0).round(1)
                print(f'points valides par secteur (8 x 45°) {name:5s}: {sec.tolist()}')

        path = f'/tmp/rf2o_diag_{TAG}.csv'
        with open(path, 'w') as f:
            f.write('stamp_ns,failed,n_valid,n_both,frac_equal,mad,sectors\n')
            for s in rows:
                sec = ' '.join(str(c) for c in s['sectors'])
                f.write(f"{s['ns']},{int(s['failed'])},{s['n_valid']},{s['n_both']},"
                        f"{s['frac_equal']:.4f},{s['mad']:.5f},{sec}\n")
        print(f'CSV : {path}')


def main():
    rclpy.init()
    node = Logger()
    t0 = time.monotonic()
    print(f'Enregistrement {DURATION:.0f} s ({TAG})...')
    try:
        while time.monotonic() - t0 < DURATION:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

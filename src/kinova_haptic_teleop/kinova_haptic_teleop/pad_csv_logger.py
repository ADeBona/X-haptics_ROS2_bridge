#!/usr/bin/env python3
"""
Shared CSV schema for the screw-torque validation log.

Used both by the live bridge node (kinova_haptic_bridge.py) and the offline
rosbag replay script (scripts/replay_screw_torque_bag.py), so the live and
replayed logs can never drift apart from each other.
"""
import csv
import os

FIELDNAMES = ['t', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
              'tau_raw', 'tau_screw', 'tau_perp_mag', 'u', 'pA', 'pB']


class ScrewTorqueCsvLogger:
    def __init__(self, path):
        self.path = os.path.expanduser(path)
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self._file = open(self.path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(FIELDNAMES)

    def write_row(self, t, force_xyz, torque_xyz, tau_raw, tau_screw,
                  tau_perp_mag, u, pA, pB):
        fx, fy, fz = force_xyz
        tx, ty, tz = torque_xyz
        self._writer.writerow(
            [t, fx, fy, fz, tx, ty, tz, tau_raw, tau_screw, tau_perp_mag, u, pA, pB])
        self._file.flush()

    def close(self):
        try:
            self._file.close()
        except Exception:
            pass

#!/usr/bin/env python3
"""
Pure functions for screw-torque extraction and pad pressure mapping.

Physics (fixed for this task, do not generalise): the task is screwing a nut
onto a vertical bolt with the end effector held parallel to the table, so the
sensor's vertical axis stays aligned with the bolt axis at all times. No TF
lookup, no rotation matrix, no frame conversion is involved anywhere here.

    tau_screw = tau_vertical + lever_sign * lever_L * f_lateral

tau_vertical is the sensor torque component about the vertical (screw) axis.
f_lateral is the sensor force component horizontal and perpendicular to the
arm. The correction term removes the false twist that a sideways force
produces at the wrist because the nut sits on a lever arm of length lever_L
away from the sensor origin. Gravity contributes nothing to this component
and must NOT be compensated here.

Everything below is either a pure function or a small ROS-free state holder
(TareEstimator), so it can be unit-tested and replayed from a rosbag with no
hardware and no rclpy dependency - see scripts/replay_screw_torque_bag.py.
"""
import math
from collections import namedtuple

ScrewTorque = namedtuple('ScrewTorque', ['tau_raw', 'tau_screw', 'tau_perp_mag'])


def compute_screw_torque(torque_xyz, force_xyz, axis_vertical, axis_lateral,
                          lever_L, lever_sign):
    """Extract the screw-axis torque from one raw wrench sample.

    torque_xyz / force_xyz: (x, y, z) tuples, straight from the wrench msg.
    axis_vertical: index (0/1/2) of the vertical/screw torque axis.
    axis_lateral:  index (0/1/2) of the lateral force axis used for the
                   lever-arm correction.

    Returns ScrewTorque(tau_raw, tau_screw, tau_perp_mag):
      tau_raw      - the uncorrected vertical torque component
      tau_screw    - tau_raw with the lever-arm correction applied
      tau_perp_mag - magnitude of the two torque components NOT used
                     (the ones this model deliberately discards)
    """
    tau_raw = torque_xyz[axis_vertical]
    f_lateral = force_xyz[axis_lateral]
    tau_screw = tau_raw + lever_sign * lever_L * f_lateral

    discarded = [torque_xyz[i] for i in range(3) if i != axis_vertical]
    tau_perp_mag = math.hypot(*discarded)

    return ScrewTorque(tau_raw, tau_screw, tau_perp_mag)


def _sign(x):
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_u(tau_tared, tau_deadband, tau_max):
    """u = sign(t) * clamp((|t| - tau_deadband) / (tau_max - tau_deadband), 0, 1)."""
    span = tau_max - tau_deadband
    if span <= 0.0:
        raise ValueError('tau_max must be greater than tau_deadband')
    mag = _clamp((abs(tau_tared) - tau_deadband) / span, 0.0, 1.0)
    return _sign(tau_tared) * mag


def compute_pad_pressures(u, pressure_bias, pressure_span):
    """Signed u -> (group_A, group_B) pressures.

    At most one of the two groups is ever above pressure_bias: group_A takes
    the positive side of u, group_B the negative side, and the other one
    sits at bias.
    """
    pA = pressure_bias + pressure_span * max(u, 0.0)
    pB = pressure_bias + pressure_span * max(-u, 0.0)
    return pA, pB


class TareEstimator:
    """Averages the next N tau_screw samples on request and freezes the
    result as an offset - not a running average. The offset stays exactly
    what it was captured as until the next start_tare()/reset().
    """

    def __init__(self, samples):
        self.samples = int(samples)
        self.offset = 0.0
        self._collecting = False
        self._buffer = []

    def start_tare(self):
        """Begin (re)capturing: the next `samples` feed() calls set offset."""
        self._collecting = True
        self._buffer = []

    def reset(self):
        """Stop any capture in progress and zero the offset."""
        self._collecting = False
        self._buffer = []
        self.offset = 0.0

    def feed(self, tau_screw):
        """Feed one untared tau_screw sample.

        Returns True the instant the offset is (re)frozen by this call.
        """
        if not self._collecting:
            return False
        self._buffer.append(tau_screw)
        if len(self._buffer) >= self.samples:
            self.offset = sum(self._buffer) / len(self._buffer)
            self._collecting = False
            self._buffer = []
            return True
        return False

    def apply(self, tau_screw):
        return tau_screw - self.offset

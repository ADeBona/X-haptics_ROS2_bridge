#!/usr/bin/env python3
"""
Unit tests for the pure screw-torque / pad-mapping functions.

No ROS, no hardware, no rclpy import - run with plain pytest:

    cd src/kinova_haptic_teleop
    python3 -m pytest test/test_screw_torque.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'kinova_haptic_teleop'))

from screw_torque import (          # noqa: E402
    compute_screw_torque, compute_u, compute_pad_pressures, TareEstimator,
)


# ---------------- compute_screw_torque ----------------

def test_lever_correction_adds_signed_term():
    torque = (0.1, 0.2, 0.9)   # x, y, z
    force = (3.0, 0.0, 0.0)    # x, y, z
    st = compute_screw_torque(torque, force, axis_vertical=2, axis_lateral=0,
                               lever_L=0.2, lever_sign=1)
    assert st.tau_raw == 0.9
    assert math.isclose(st.tau_screw, 0.9 + 1 * 0.2 * 3.0)


def test_lever_sign_flips_correction():
    torque = (0.0, 0.0, 1.0)
    force = (2.0, 0.0, 0.0)
    st_plus = compute_screw_torque(torque, force, 2, 0, 0.2, 1)
    st_minus = compute_screw_torque(torque, force, 2, 0, 0.2, -1)
    assert math.isclose(st_plus.tau_screw, 1.0 + 0.4)
    assert math.isclose(st_minus.tau_screw, 1.0 - 0.4)


def test_tau_perp_mag_is_the_two_discarded_axes():
    torque = (3.0, 4.0, 1.0)
    force = (0.0, 0.0, 0.0)
    st = compute_screw_torque(torque, force, axis_vertical=2, axis_lateral=0,
                               lever_L=0.2, lever_sign=1)
    assert math.isclose(st.tau_perp_mag, 5.0)   # hypot(3, 4)


def test_gravity_like_force_on_non_lateral_axis_is_ignored():
    # A force on an axis other than axis_lateral must not leak into tau_screw.
    torque = (0.0, 0.0, 1.0)
    force = (0.0, 0.0, 50.0)   # e.g. a large vertical (gravity) force
    st = compute_screw_torque(torque, force, axis_vertical=2, axis_lateral=0,
                               lever_L=0.2, lever_sign=1)
    assert math.isclose(st.tau_screw, 1.0)


# ---------------- compute_u ----------------

def test_u_zero_inside_deadband():
    assert compute_u(0.03, tau_deadband=0.05, tau_max=2.0) == 0.0
    assert compute_u(-0.03, tau_deadband=0.05, tau_max=2.0) == 0.0


def test_u_sign_follows_input():
    assert compute_u(1.0, 0.05, 2.0) > 0
    assert compute_u(-1.0, 0.05, 2.0) < 0


def test_u_clamped_to_unity_above_tau_max():
    assert compute_u(10.0, 0.05, 2.0) == 1.0
    assert compute_u(-10.0, 0.05, 2.0) == -1.0


def test_u_linear_between_deadband_and_max():
    # halfway between deadband and max -> u = 0.5
    t = 0.05 + 0.5 * (2.0 - 0.05)
    assert math.isclose(compute_u(t, 0.05, 2.0), 0.5)


def test_u_invalid_span_raises():
    try:
        compute_u(1.0, tau_deadband=2.0, tau_max=1.0)
    except ValueError:
        return
    raise AssertionError('expected ValueError for tau_max <= tau_deadband')


# ---------------- compute_pad_pressures ----------------

def test_positive_u_only_drives_group_a():
    pA, pB = compute_pad_pressures(0.6, pressure_bias=5.0, pressure_span=40.0)
    assert math.isclose(pA, 5.0 + 40.0 * 0.6)
    assert pB == 5.0


def test_negative_u_only_drives_group_b():
    pA, pB = compute_pad_pressures(-0.6, pressure_bias=5.0, pressure_span=40.0)
    assert math.isclose(pB, 5.0 + 40.0 * 0.6)
    assert pA == 5.0


def test_zero_u_both_groups_at_bias():
    pA, pB = compute_pad_pressures(0.0, pressure_bias=5.0, pressure_span=40.0)
    assert pA == 5.0 and pB == 5.0


def test_at_most_one_group_above_bias():
    for u in (-1.0, -0.3, 0.0, 0.3, 1.0):
        pA, pB = compute_pad_pressures(u, pressure_bias=5.0, pressure_span=40.0)
        above_a = pA > 5.0
        above_b = pB > 5.0
        assert not (above_a and above_b)


# ---------------- TareEstimator ----------------

def test_tare_ignores_samples_until_requested():
    tare = TareEstimator(samples=3)
    tare.feed(100.0)
    tare.feed(200.0)
    assert tare.offset == 0.0


def test_tare_freezes_mean_after_n_samples():
    tare = TareEstimator(samples=3)
    tare.start_tare()
    assert tare.feed(1.0) is False
    assert tare.feed(2.0) is False
    assert tare.feed(3.0) is True
    assert math.isclose(tare.offset, 2.0)


def test_tare_does_not_running_average_after_freeze():
    tare = TareEstimator(samples=2)
    tare.start_tare()
    tare.feed(0.0)
    tare.feed(2.0)
    assert math.isclose(tare.offset, 1.0)
    # further samples must NOT move a frozen offset
    tare.feed(1000.0)
    tare.feed(-1000.0)
    assert math.isclose(tare.offset, 1.0)


def test_tare_apply_subtracts_frozen_offset():
    tare = TareEstimator(samples=2)
    tare.start_tare()
    tare.feed(1.0)
    tare.feed(3.0)   # offset -> 2.0
    assert math.isclose(tare.apply(2.5), 0.5)


def test_reset_zeroes_offset_and_stops_capture():
    tare = TareEstimator(samples=2)
    tare.start_tare()
    tare.feed(5.0)
    tare.reset()
    assert tare.offset == 0.0
    # a stray feed() after reset (capture stopped) must not resume capturing
    assert tare.feed(999.0) is False
    assert tare.offset == 0.0

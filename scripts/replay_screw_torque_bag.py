#!/usr/bin/env python3
"""
Offline replay: rosbag2 -> the same pure screw-torque / pad-mapping
functions the live bridge uses -> the same CSV schema.

Lets the whole tau_screw / tare / pad-mapping pipeline be validated against
recorded data with no robot and no hardware attached.

Usage:
    python3 replay_screw_torque_bag.py --bag /path/to/bag_dir \
        --topic /ft_sensor_link/wrench --out replayed_log.csv

Optional tare simulation: pass --tare-at-start to mimic triggering ~/tare
right after the first sample (average the first `--tare-samples` samples
and freeze that as the offset), matching how the live node would behave if
tare was requested at t=0.

Runs inside the ROS container ([HOST] does not have rosbag2_py) - source
install/setup.bash first:

    source install/setup.bash
    python3 scripts/replay_screw_torque_bag.py --bag my_bag --out replay.csv
"""
import argparse
import os
import sys

# Import the exact same pure functions the live bridge node uses.
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..',
    'src', 'kinova_haptic_teleop', 'kinova_haptic_teleop'))

from screw_torque import (          # noqa: E402
    compute_screw_torque, compute_u, compute_pad_pressures, TareEstimator,
)
from pad_csv_logger import ScrewTorqueCsvLogger    # noqa: E402


def iter_wrench_messages(bag_path, topic):
    """Yield (t_seconds, WrenchStamped) for every message on `topic` in the bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import WrenchStamped

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_by_topic:
        raise SystemExit(
            f'Topic {topic!r} not found in bag. Available: {sorted(type_by_topic)}')
    if type_by_topic[topic] != 'geometry_msgs/msg/WrenchStamped':
        raise SystemExit(
            f'Topic {topic!r} is {type_by_topic[topic]!r}, not WrenchStamped')

    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    while reader.has_next():
        _topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, WrenchStamped)
        yield t_ns * 1e-9, msg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', required=True, help='rosbag2 directory')
    ap.add_argument('--topic', default='/ft_sensor_link/wrench')
    ap.add_argument('--out', default='replayed_screw_torque_log.csv')

    ap.add_argument('--axis-vertical', type=int, default=2)
    ap.add_argument('--axis-lateral', type=int, default=0)
    ap.add_argument('--lever-L', type=float, default=0.20)
    ap.add_argument('--lever-sign', type=int, default=1)

    ap.add_argument('--tau-deadband', type=float, default=0.05)
    ap.add_argument('--tau-max', type=float, default=2.0)
    ap.add_argument('--pressure-bias', type=float, default=0.0)
    ap.add_argument('--pressure-span', type=float, default=40.0)

    ap.add_argument('--tare-samples', type=int, default=20)
    ap.add_argument('--tare-at-start', action='store_true',
                     help='simulate a ~/tare request on the first sample')
    args = ap.parse_args()

    tare = TareEstimator(args.tare_samples)
    if args.tare_at_start:
        tare.start_tare()

    logger = ScrewTorqueCsvLogger(args.out)
    n = 0
    try:
        for t, msg in iter_wrench_messages(args.bag, args.topic):
            force = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
            torque = (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)

            st = compute_screw_torque(
                torque, force, args.axis_vertical, args.axis_lateral,
                args.lever_L, args.lever_sign)

            if tare.feed(st.tau_screw):
                print(f'[{t:.3f}] tare captured: offset={tare.offset:.4f} Nm')

            tau_tared = tare.apply(st.tau_screw)
            u = compute_u(tau_tared, args.tau_deadband, args.tau_max)
            pA, pB = compute_pad_pressures(u, args.pressure_bias, args.pressure_span)

            logger.write_row(
                t=t, force_xyz=force, torque_xyz=torque, tau_raw=st.tau_raw,
                tau_screw=st.tau_screw, tau_perp_mag=st.tau_perp_mag,
                u=u, pA=pA, pB=pB)
            n += 1
    finally:
        logger.close()

    print(f'Wrote {n} rows to {args.out}')


if __name__ == '__main__':
    main()

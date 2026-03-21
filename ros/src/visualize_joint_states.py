#!/usr/bin/env python3
"""Visualize joint states (position, velocity, effort) from a rosbag."""

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

typestore = get_typestore(Stores.ROS2_HUMBLE)

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def read_bag(bag_path):
    timestamps = []
    positions = []
    velocities = []

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != '/joint_states':
                continue

            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            timestamps.append(timestamp * 1e-9)

            # Reorder to match JOINT_NAMES order
            idx_map = {name: i for i, name in enumerate(msg.name)}
            pos = [msg.position[idx_map[j]] if j in idx_map and len(msg.position) > idx_map[j] else 0.0
                   for j in JOINT_NAMES]
            vel = [msg.velocity[idx_map[j]] if j in idx_map and len(msg.velocity) > idx_map[j] else 0.0
                   for j in JOINT_NAMES]

            positions.append(pos)
            velocities.append(vel)

    timestamps = np.array(timestamps)
    timestamps -= timestamps[0]  # relative time from start
    positions = np.array(positions)
    velocities = np.array(velocities)

    return timestamps, positions, velocities


def trim_wait(timestamps, positions, velocities, offset=0.5, threshold=0.01):
    """Remove the initial idle period. Starts 0.5s after the first velocity spike."""
    max_vel = np.max(np.abs(velocities), axis=1)
    spike_indices = np.where(max_vel > threshold)[0]
    if len(spike_indices) == 0:
        return timestamps, positions, velocities
    
    spike_time = timestamps[spike_indices[0]]
    start_time = spike_time + offset
    mask = timestamps >= start_time
    timestamps = timestamps[mask] - timestamps[mask][0]
    return timestamps, positions[mask], velocities[mask]


def plot(timestamps, positions, velocities, save_path=None):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    for i, name in enumerate(JOINT_NAMES):
        axes[0].plot(timestamps, positions[:, i], label=name)
        axes[1].plot(timestamps, velocities[:, i], label=name)

    axes[0].set_ylabel('Position (rad)')
    axes[0].set_title('Joint Positions')
    axes[0].legend(loc='upper right', fontsize='small')
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].set_title('Joint Velocities')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(loc='upper right', fontsize='small')
    axes[1].grid(True, alpha=0.3)

    fig.suptitle('Joint States from Rosbag', fontsize=14)
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved plot to {save_path}')

    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize joint states from a rosbag.')
    parser.add_argument('bag_path', help='Path to the rosbag directory')
    parser.add_argument('--trim-wait', action='store_true',
                        help='Remove initial idle period before first movement')
    parser.add_argument('--time-range', nargs=2, type=float, metavar=('START', 'END'),
                        help='Only plot data between START and END seconds')
    args = parser.parse_args()

    timestamps, positions, velocities = read_bag(args.bag_path)
    print(f'Read {len(timestamps)} messages over {timestamps[-1]:.2f}s')

    if args.trim_wait:
        timestamps, positions, velocities = trim_wait(timestamps, positions, velocities)
        print(f'After trimming: {len(timestamps)} messages over {timestamps[-1]:.2f}s')

    if args.time_range:
        t_start, t_end = args.time_range
        mask = (timestamps >= t_start) & (timestamps <= t_end)
        timestamps = timestamps[mask] - timestamps[mask][0]
        positions = positions[mask]
        velocities = velocities[mask]
        print(f'After time range [{t_start}, {t_end}]: {len(timestamps)} messages over {timestamps[-1]:.2f}s')

    save_path = os.path.join(args.bag_path, 'joint_states.png')
    plot(timestamps, positions, velocities, save_path=save_path)


if __name__ == '__main__':
    main()

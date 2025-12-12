#!/usr/bin/env python3
"""
Visualize 3D positions from a single log file with base positions.
Shows only accepted positions (first arrival at each desired position).
Based on log_analysis.py parsing logic.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import sys
from pathlib import Path
from datetime import datetime
from collections import defaultdict


def parse_log_file_for_arrivals(log_path):
    """
    Parse log file to extract base positions and first arrival data.
    Only captures positions when system reaches desired position (RUN state).
    Returns base positions and list of arrival records.
    """
    base_positions = {}
    arrival_timing = []

    # State tracking
    awaiting_first_arrival = False
    new_position_start_time = None
    new_position_start_distance = None
    new_position_coordinates = None
    last_position = None
    last_desired_position = None
    last_in_plane_distance = None

    with open(log_path, 'r') as f:
        for line in f:
            if not line.strip():
                continue

            # Extract timestamp
            timestamp_match = re.search(r'\[((\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}))\]', line)
            current_time = timestamp_match.group(1) if timestamp_match else None

            # Extract static base positions
            if 'Static base 0 position stored:' in line:
                match = re.search(r'Static base 0 position stored:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
                if match:
                    base_positions['base1'] = np.array([float(match.group(1)),
                                                        float(match.group(2)),
                                                        float(match.group(3))])

            elif 'Static base 1 position stored:' in line:
                match = re.search(r'Static base 1 position stored:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
                if match:
                    base_positions['base2'] = np.array([float(match.group(1)),
                                                        float(match.group(2)),
                                                        float(match.group(3))])

            elif 'Static base 2 position stored:' in line:
                match = re.search(r'Static base 2 position stored:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
                if match:
                    base_positions['base3'] = np.array([float(match.group(1)),
                                                        float(match.group(2)),
                                                        float(match.group(3))])

            # Track new desired position being set
            if 'New desired position set:' in line:
                match = re.search(r'New desired position set:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
                if match and current_time:
                    x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
                    awaiting_first_arrival = True
                    new_position_start_time = current_time
                    new_position_coordinates = {'x': x, 'y': y, 'z': z}

                    # Calculate starting distance if we have current position
                    if last_position:
                        dx = x - last_position['x']
                        dy = y - last_position['y']
                        dz = z - last_position['z']
                        new_position_start_distance = (dx**2 + dy**2 + dz**2)**0.5
                    else:
                        new_position_start_distance = None

            # Track current position
            if 'Current position:' in line:
                match = re.search(r'Current position:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
                if match and current_time:
                    last_position = {
                        'timestamp': current_time,
                        'x': float(match.group(1)),
                        'y': float(match.group(2)),
                        'z': float(match.group(3))
                    }

            # Track desired position (for fallback)
            if 'Desired position:' in line:
                match = re.search(r'Desired position:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)', line)
                if match and current_time:
                    last_desired_position = {
                        'timestamp': current_time,
                        'x': float(match.group(1)),
                        'y': float(match.group(2)),
                        'z': float(match.group(3))
                    }

            # Track in-plane distance
            if 'In-plane distance' in line:
                match = re.search(r'In-plane distance \(projected desired to projected current\):\s*([-\d.]+)', line)
                if match:
                    last_in_plane_distance = float(match.group(1))

            # Detect state transitions to RUN (arrival at position)
            if '->' in line and 'RUN' in line:
                match = re.search(r'(\w+)\s*->\s*RUN', line)
                if match:
                    previous_state = match.group(1)
                    # Check if we're arriving from a movement state
                    if previous_state in ['MOVE', 'MOVING', 'TENSE', 'TENSION']:
                        if awaiting_first_arrival and new_position_start_time and current_time:
                            try:
                                start_dt = datetime.strptime(new_position_start_time, '%Y-%m-%d %H:%M:%S.%f')
                                arrival_dt = datetime.strptime(current_time, '%Y-%m-%d %H:%M:%S.%f')
                                time_to_reach_sec = (arrival_dt - start_dt).total_seconds()

                                # Record the arrival
                                arrival_record = {
                                    'position_set_time': new_position_start_time,
                                    'arrival_time': current_time,
                                    'time_to_reach_sec': time_to_reach_sec,
                                    'starting_distance_mm': new_position_start_distance,
                                    'target_position': new_position_coordinates,
                                    'error_magnitude_mm': last_in_plane_distance,
                                    'current_position': last_position,
                                    'desired_position': last_desired_position if last_desired_position else new_position_coordinates
                                }
                                arrival_timing.append(arrival_record)

                                # Reset flag
                                awaiting_first_arrival = False
                            except (ValueError, TypeError):
                                pass

    return base_positions, arrival_timing


def calculate_plane_normal(base1, base2, base3):
    """Calculate normalized plane normal from three base positions."""
    v1 = base2 - base1
    v2 = base3 - base1
    normal = np.cross(v1, v2)
    return normal / np.linalg.norm(normal)


def project_onto_plane(point, plane_point, normal):
    """Project a point onto a plane defined by a point and normal."""
    v = point - plane_point
    dist = np.dot(v, normal)
    return point - dist * normal


def create_plane_mesh(base1, base2, base3):
    """Create a mesh grid for visualizing the plane."""
    v1 = base2 - base1
    v2 = base3 - base1

    u = np.linspace(-0.2, 1.2, 10)
    v = np.linspace(-0.2, 1.2, 10)
    U, V = np.meshgrid(u, v)

    X = base1[0] + U * v1[0] + V * v2[0]
    Y = base1[1] + U * v1[1] + V * v2[1]
    Z = base1[2] + U * v1[2] + V * v2[2]

    return X, Y, Z


def plot_all_arrivals_with_bases(base_positions, arrival_data, log_name, save_path=None):
    """
    Create 3D plot showing all accepted arrival positions on a single chart.

    Parameters:
    -----------
    base_positions : dict
        Dictionary with 'base1', 'base2', 'base3' as numpy arrays
    arrival_data : list
        List of arrival records with current_position, desired_position, etc.
    log_name : str
        Name of the log file
    save_path : str, optional
        Path to save the figure. If None, displays interactively.
    """
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Extract base positions
    base1 = base_positions['base1']
    base2 = base_positions['base2']
    base3 = base_positions['base3']

    # Plot base positions
    ax.scatter(*base1, c='red', s=250, marker='s', label='Base 1',
               edgecolors='black', linewidths=2, zorder=10)
    ax.scatter(*base2, c='green', s=250, marker='s', label='Base 2',
               edgecolors='black', linewidths=2, zorder=10)
    ax.scatter(*base3, c='blue', s=250, marker='s', label='Base 3',
               edgecolors='black', linewidths=2, zorder=10)

    # Calculate centroid
    centroid = (base1 + base2 + base3) / 3

    # Plot centroid
    ax.scatter(*centroid, c='black', s=200, marker='o', label='Centroid',
               edgecolors='white', linewidths=2, zorder=10)

    # Draw triangle connecting bases
    bases = np.array([base1, base2, base3, base1])
    ax.plot(bases[:, 0], bases[:, 1], bases[:, 2], 'k--', linewidth=2, alpha=0.6, zorder=5)

    # Plot the base plane
    X, Y, Z = create_plane_mesh(base1, base2, base3)
    ax.plot_surface(X, Y, Z, alpha=0.1, color='gray', zorder=1)

    # Calculate plane normal for projections
    normal = calculate_plane_normal(base1, base2, base3)

    # Extract current and desired positions from arrivals
    current_positions = []
    desired_positions = []
    error_magnitudes = []

    for arrival in arrival_data:
        curr_pos = arrival.get('current_position')
        des_pos = arrival.get('desired_position')

        if curr_pos and isinstance(curr_pos, dict):
            current_positions.append([curr_pos['x'], curr_pos['y'], curr_pos['z']])

        if des_pos and isinstance(des_pos, dict):
            desired_positions.append([des_pos['x'], des_pos['y'], des_pos['z']])

        error_mag = arrival.get('error_magnitude_mm')
        if error_mag is not None:
            error_magnitudes.append(error_mag)

    current_positions = np.array(current_positions)
    desired_positions = np.array(desired_positions)

    if len(current_positions) == 0:
        print("No valid arrival positions found!")
        return None

    # Project current positions onto the base plane
    projected_positions = np.array([project_onto_plane(pos, base1, normal) for pos in current_positions])

    # Plot projected current positions (green circles)
    ax.scatter(projected_positions[:, 0], projected_positions[:, 1], projected_positions[:, 2],
               c='green', s=120, marker='o', alpha=0.8, label='Projected Current',
               edgecolors='darkgreen', linewidths=1.5, zorder=4)

    # Draw trajectory line connecting projected positions
    ax.plot(projected_positions[:, 0], projected_positions[:, 1], projected_positions[:, 2],
            '-', color='green', linewidth=1.5, alpha=0.5, zorder=3)

    # Plot desired positions (blue circles)
    if len(desired_positions) > 0:
        # Project desired positions onto the base plane
        projected_desired = np.array([project_onto_plane(pos, base1, normal) for pos in desired_positions])

        ax.scatter(projected_desired[:, 0], projected_desired[:, 1], projected_desired[:, 2],
                   c='blue', s=120, marker='o', alpha=0.6, label='Desired Positions',
                   edgecolors='darkblue', linewidths=1.5, zorder=5)

        # Draw error vectors between projected positions
        for proj_curr, proj_des in zip(projected_positions, projected_desired):
            ax.plot([proj_curr[0], proj_des[0]],
                    [proj_curr[1], proj_des[1]],
                    [proj_curr[2], proj_des[2]],
                    'r-', linewidth=1.5, alpha=0.5, zorder=2)

    # Calculate statistics
    times_to_reach = [arr['time_to_reach_sec'] for arr in arrival_data]

    info_text = f"Log: {log_name}\n"
    info_text += f"Accepted Arrivals: {len(arrival_data)}\n"
    if error_magnitudes:
        info_text += f"\nError Statistics:\n"
        info_text += f"  Mean:   {np.mean(error_magnitudes):.2f} mm\n"
        info_text += f"  Median: {np.median(error_magnitudes):.2f} mm\n"
        info_text += f"  Max:    {np.max(error_magnitudes):.2f} mm\n"
    if times_to_reach:
        info_text += f"\nTiming Statistics:\n"
        info_text += f"  Mean time:   {np.mean(times_to_reach):.2f} s\n"
        info_text += f"  Median time: {np.median(times_to_reach):.2f} s"

    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes,
             fontsize=9, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))

    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_zlabel('Z (mm)', fontsize=12)
    ax.set_title(f'Accepted Position Arrivals - {log_name}', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)

    # Set equal aspect ratio including all points
    all_points = np.vstack([bases, current_positions, desired_positions])
    max_range = np.ptp(all_points, axis=0).max() / 2.0
    mid = np.mean(all_points, axis=0)

    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    # Set view angle: looking down at the plane
    # Base 3 (blue) in top left, Base 2 (green) in bottom middle, Base 1 (red) in top right
    ax.view_init(elev=90, azim=-90)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved visualization to {save_path}")
    else:
        plt.show()

    return fig


def main():
    """Main function to visualize accepted arrivals from a log file."""

    if len(sys.argv) < 2:
        print("Usage: python visualize_runs_with_bases.py <log_file_path> [--save]")
        print("  --save: Save the visualization to a file instead of displaying")
        sys.exit(1)

    log_path = Path(sys.argv[1])

    if not log_path.exists():
        print(f"Error: Log file not found: {log_path}")
        sys.exit(1)

    print(f"Parsing log file: {log_path}")
    base_positions, arrival_data = parse_log_file_for_arrivals(log_path)

    if len(base_positions) != 3:
        print("Error: Could not find all three base positions in log file!")
        sys.exit(1)

    if not arrival_data:
        print("No arrival data found in log file!")
        print("Make sure the log contains:")
        print("  - 'New desired position set:' entries")
        print("  - State transitions to RUN (e.g., 'TENSE -> RUN')")
        sys.exit(1)

    print(f"Found {len(arrival_data)} accepted position arrivals")

    # Check if we should save
    save_path = None
    if len(sys.argv) > 2 and sys.argv[2] == '--save':
        output_dir = Path("position_visualizations")
        output_dir.mkdir(exist_ok=True)
        save_path = output_dir / f"{log_path.stem}_arrivals.png"

    # Create visualization
    plot_all_arrivals_with_bases(base_positions, arrival_data, log_path.name, save_path=save_path)


if __name__ == "__main__":
    main()

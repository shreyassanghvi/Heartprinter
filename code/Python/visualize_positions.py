#!/usr/bin/env python3
"""
Visualize 3D positions from Heartprinter logs.
Shows base positions, current position, desired position, and the base plane.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import sys
from pathlib import Path


def parse_log_file(log_path):
    """Parse positions from log file."""
    positions = []

    with open(log_path, 'r') as f:
        for line in f:
            # Look for status update lines with Base positions
            if 'Status update written:' in line:
                # Extract Base1, Base2, Base3, Current positions
                base_pattern = r'Base(\d)\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'

                bases = re.findall(base_pattern, line)

                if len(bases) == 3:
                    pos_data = {
                        'base1': np.array([float(bases[0][1]), float(bases[0][2]), float(bases[0][3])]),
                        'base2': np.array([float(bases[1][1]), float(bases[1][2]), float(bases[1][3])]),
                        'base3': np.array([float(bases[2][1]), float(bases[2][2]), float(bases[2][3])])
                    }

            if 'Current position: (' in line:
                current_pattern = r'Current position:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'
                match = re.search(current_pattern, line)
                if match and 'pos_data' in locals():
                    pos_data['current'] = np.array([float(match.group(1)),
                                                    float(match.group(2)),
                                                    float(match.group(3))])

            # Look for desired position
            if 'Desired position: (' in line:
                desired_pattern = r'Desired position:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'
                match = re.search(desired_pattern, line)
                if match and 'pos_data' in locals():
                    pos_data['desired'] = np.array([float(match.group(1)),
                                                    float(match.group(2)),
                                                    float(match.group(3))])

            if 'Error vector magnitude ' in line:
                positions.append(pos_data)
                pos_data = {}

    return positions


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


def create_plane_mesh(base1, base2, base3, extend=1.2):
    """Create a mesh grid for visualizing the plane."""
    # Get two vectors in the plane
    v1 = base2 - base1
    v2 = base3 - base1

    # Create a grid
    u = np.linspace(-0.2, 1.2, 10)
    v = np.linspace(-0.2, 1.2, 10)
    U, V = np.meshgrid(u, v)

    # Calculate plane points
    X = base1[0] + U * v1[0] + V * v2[0]
    Y = base1[1] + U * v1[1] + V * v2[1]
    Z = base1[2] + U * v1[2] + V * v2[2]

    return X, Y, Z


def plot_positions(pos_data, title="Heartprinter Position Visualization"):
    """Create 3D plot of positions."""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    base1 = pos_data['base1']
    base2 = pos_data['base2']
    base3 = pos_data['base3']
    current = pos_data['current']
    desired = pos_data.get('desired')

    print(f"Current: {current}\tDesired: {desired}")

    # Plot base positions
    ax.scatter(*base1, c='red', s=200, marker='s', label='Base 1', edgecolors='black', linewidths=2)
    ax.scatter(*base2, c='green', s=200, marker='s', label='Base 2', edgecolors='black', linewidths=2)
    ax.scatter(*base3, c='blue', s=200, marker='s', label='Base 3', edgecolors='black', linewidths=2)

    # Draw triangle connecting bases
    bases = np.array([base1, base2, base3, base1])
    ax.plot(bases[:, 0], bases[:, 1], bases[:, 2], 'k--', linewidth=1, alpha=0.5)

    # Plot the base plane
    X, Y, Z = create_plane_mesh(base1, base2, base3)
    ax.plot_surface(X, Y, Z, alpha=0.2, color='gray')

    # Calculate and show plane normal
    normal = calculate_plane_normal(base1, base2, base3)
    centroid = (base1 + base2 + base3) / 3
    ax.scatter(*centroid, c='black', s=200, marker='s', label='Centroid', edgecolors='black', linewidths=2)
    normal_scale = 20
    # ax.quiver(centroid[0], centroid[1], centroid[2],
    #          normal[0], normal[1], normal[2],
    #          length=normal_scale, color='purple', arrow_length_ratio=0.3,
    #          linewidth=2, label='Plane Normal')

    # Plot current position
    ax.scatter(*current, c='orange', s=150, marker='o', label='Current Position',
               edgecolors='black', linewidths=2)

    # Project current onto plane
    current_proj = project_onto_plane(current, base1, normal)
    ax.scatter(*current_proj, c='orange', s=100, marker='x',
               label='Current Projected', linewidths=3)

    # Draw line from current to its projection (perpendicular distance)
    # ax.plot([current[0], current_proj[0]],
    #         [current[1], current_proj[1]],
    #         [current[2], current_proj[2]],
    #         'orange', linestyle=':', linewidth=2, alpha=0.7)

    # Plot desired position if available
    if desired is not None:
        ax.scatter(*desired, c='cyan', s=150, marker='*', label='Desired Position',
                   edgecolors='black', linewidths=2)

        # Project desired onto plane
        desired_proj = project_onto_plane(desired, base1, normal)
        ax.scatter(*desired_proj, c='cyan', s=100, marker='x',
                   label='Desired Projected', linewidths=3)

        # # Draw line from desired to its projection
        # ax.plot([desired[0], desired_proj[0]],
        #         [desired[1], desired_proj[1]],
        #         [desired[2], desired_proj[2]],
        #         'cyan', linestyle=':', linewidth=2, alpha=0.7)

        # Draw error vector from current to desired (3D)
        ax.quiver(current[0], current[1], current[2],
                 desired[0] - current[0],
                 desired[1] - current[1],
                 desired[2] - current[2],
                 color='red', arrow_length_ratio=0.15, linewidth=2,
                 label='3D Error Vector', alpha=0.7)

        # Draw in-plane error vector (projected positions)
        ax.quiver(current_proj[0], current_proj[1], current_proj[2],
                 desired_proj[0] - current_proj[0],
                 desired_proj[1] - current_proj[1],
                 desired_proj[2] - current_proj[2],
                 color='magenta', arrow_length_ratio=0.15, linewidth=3,
                 label='In-Plane Error Vector')

        # Calculate and display errors
        error_3d = np.linalg.norm(desired - current)
        error_inplane = np.linalg.norm(desired_proj - current_proj)
        dist_to_plane_current = np.abs(np.dot(current - base1, normal))
        dist_to_plane_desired = np.abs(np.dot(desired - base1, normal))

        info_text = f"3D Error: {error_3d:.2f} mm\n"
        info_text += f"In-Plane Error: {error_inplane:.2f} mm\n"
        info_text += f"Current ⊥ to plane: {dist_to_plane_current:.2f} mm\n"
        info_text += f"Desired ⊥ to plane: {dist_to_plane_desired:.2f} mm"

        ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes,
                 fontsize=10, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_zlabel('Z (mm)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)

    # Set equal aspect ratio
    max_range = np.ptp(np.array([
        base1[0], base2[0], base3[0], current[0]
    ] + ([desired[0]] if desired is not None else [])))

    mid_x = np.mean([base1[0], base2[0], base3[0]])
    mid_y = np.mean([base1[1], base2[1], base3[1]])
    mid_z = np.mean([base1[2], base2[2], base3[2]])

    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

    plt.tight_layout()
    return fig


def main():
    if len(sys.argv) > 1:
        # Parse from log file
        log_path = sys.argv[1]
        print(f"Parsing log file: {log_path}")
        positions = parse_log_file(log_path)

        if not positions:
            print("No position data found in log file!")
            return

        print(f"Found {len(positions)} position snapshots")

        # Plot each snapshot or just the last one
        if len(sys.argv) > 2 and sys.argv[2] == '--all':
            for i, pos in enumerate(positions):
                plot_positions(pos, f"Position Snapshot {i+1}")
                plt.show()
        else:
            # Plot last snapshot
            plot_positions(positions[-1], "Latest Position")
            plt.show()
    else:
        # Use example data from your log
        pos_data = {
            'base1': np.array([151.13, 85.50, -44.43]),
            'base2': np.array([178.82, 41.52, -57.82]),
            'base3': np.array([187.75, 86.62, -74.79]),
            'current': np.array([165.87, 68.98, -66.53]),
            'desired': np.array([172.641, 71.140, -59.085])
        }

        plot_positions(pos_data)
        plt.show()


if __name__ == "__main__":
    main()
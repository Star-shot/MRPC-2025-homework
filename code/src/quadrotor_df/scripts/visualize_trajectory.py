#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize lemniscate trajectory and quadrotor attitude
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import sys
import os

def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion [x, y, z, w] to rotation matrix
    """
    x, y, z, w = q[0], q[1], q[2], q[3]
    
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
    
    return R

def visualize_trajectory(csv_file, output_file=None, step=10):
    """
    Visualize lemniscate trajectory with quadrotor attitude
    
    Args:
        csv_file: CSV file path containing quaternion data
        output_file: Output image path (optional)
        step: Step size for displaying attitude frames (every Nth point)
    """
    # Read CSV file
    try:
        df = pd.read_csv(csv_file, skipinitialspace=True)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Extract data
    t = df['t'].values
    x_q = df['x'].values
    y_q = df['y'].values
    z_q = df['z'].values
    w_q = df['w'].values
    
    # Compute position from trajectory equation
    positions = []
    for ti in t:
        sin_t = np.sin(ti)
        cos_t = np.cos(ti)
        denom = 1.0 + sin_t * sin_t
        x = 10.0 * cos_t / denom
        y = 10.0 * sin_t * cos_t / denom
        z = 10.0
        positions.append([x, y, z])
    
    positions = np.array(positions)
    
    # Create 3D figure
    fig = plt.figure(figsize=(16, 12))
    
    # Main 3D trajectory plot
    ax1 = fig.add_subplot(2, 2, (1, 3), projection='3d')
    
    # Plot trajectory
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
             'b-', linewidth=2, label='Trajectory', alpha=0.7)
    
    # Plot attitude frames at regular intervals
    frame_length = 1.5
    colors = ['r', 'g', 'b']  # x, y, z axes
    
    for i in range(0, len(t), step):
        pos = positions[i]
        q = np.array([x_q[i], y_q[i], z_q[i], w_q[i]])
        R = quaternion_to_rotation_matrix(q)
        
        # Draw coordinate frame (x, y, z axes)
        for axis_idx in range(3):
            axis_vec = R[:, axis_idx] * frame_length
            ax1.quiver(pos[0], pos[1], pos[2],
                      axis_vec[0], axis_vec[1], axis_vec[2],
                      color=colors[axis_idx], arrow_length_ratio=0.3,
                      linewidth=1.5, alpha=0.8)
    
    # Mark start and end points
    ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
               color='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
               color='red', s=100, marker='s', label='End', zorder=5)
    
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_zlabel('Z (m)', fontsize=12)
    ax1.set_title('Lemniscate Trajectory with Quadrotor Attitude', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Set equal aspect ratio
    max_range = np.array([positions[:, 0].max() - positions[:, 0].min(),
                          positions[:, 1].max() - positions[:, 1].min(),
                          positions[:, 2].max() - positions[:, 2].min()]).max() / 2.0
    mid_x = (positions[:, 0].max() + positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max() + positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Top view (XY plane)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Trajectory', alpha=0.7)
    ax2.scatter(positions[0, 0], positions[0, 1], color='green', s=100, marker='o', label='Start', zorder=5)
    ax2.scatter(positions[-1, 0], positions[-1, 1], color='red', s=100, marker='s', label='End', zorder=5)
    
    # Draw some attitude frames in top view
    for i in range(0, len(t), step*2):
        pos = positions[i]
        q = np.array([x_q[i], y_q[i], z_q[i], w_q[i]])
        R = quaternion_to_rotation_matrix(q)
        
        # Draw x and y axes (projected to XY plane)
        x_axis = R[:, 0] * frame_length * 0.5
        y_axis = R[:, 1] * frame_length * 0.5
        
        ax2.arrow(pos[0], pos[1], x_axis[0], x_axis[1],
                 head_width=0.3, head_length=0.2, fc='r', ec='r', alpha=0.6)
        ax2.arrow(pos[0], pos[1], y_axis[0], y_axis[1],
                 head_width=0.3, head_length=0.2, fc='g', ec='g', alpha=0.6)
    
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Y (m)', fontsize=12)
    ax2.set_title('Top View (XY Plane)', fontsize=13)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    # Side view (XZ plane)
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=2, label='Trajectory', alpha=0.7)
    ax3.scatter(positions[0, 0], positions[0, 2], color='green', s=100, marker='o', label='Start', zorder=5)
    ax3.scatter(positions[-1, 0], positions[-1, 2], color='red', s=100, marker='s', label='End', zorder=5)
    
    # Draw some attitude frames in side view
    for i in range(0, len(t), step*2):
        pos = positions[i]
        q = np.array([x_q[i], y_q[i], z_q[i], w_q[i]])
        R = quaternion_to_rotation_matrix(q)
        
        # Draw x and z axes (projected to XZ plane)
        x_axis = R[:, 0] * frame_length * 0.5
        z_axis = R[:, 2] * frame_length * 0.5
        
        ax3.arrow(pos[0], pos[2], x_axis[0], z_axis[2],
                 head_width=0.3, head_length=0.2, fc='r', ec='r', alpha=0.6)
        ax3.arrow(pos[0], pos[2], z_axis[0], z_axis[2],
                 head_width=0.3, head_length=0.2, fc='b', ec='b', alpha=0.6)
    
    ax3.set_xlabel('X (m)', fontsize=12)
    ax3.set_ylabel('Z (m)', fontsize=12)
    ax3.set_title('Side View (XZ Plane)', fontsize=13)
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect('equal')
    
    plt.tight_layout()
    
    # Save figure
    if output_file is None:
        output_file = csv_file.replace('.csv', '_trajectory_visualization.png')
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Trajectory visualization saved to: {output_file}")
    
    # Create a separate detailed 3D view
    fig2 = plt.figure(figsize=(14, 10))
    ax = fig2.add_subplot(111, projection='3d')
    
    # Plot trajectory with color based on time
    scatter = ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                        c=t, cmap='viridis', s=20, alpha=0.6)
    
    # Plot attitude frames
    for i in range(0, len(t), step):
        pos = positions[i]
        q = np.array([x_q[i], y_q[i], z_q[i], w_q[i]])
        R = quaternion_to_rotation_matrix(q)
        
        for axis_idx in range(3):
            axis_vec = R[:, axis_idx] * frame_length
            ax.quiver(pos[0], pos[1], pos[2],
                     axis_vec[0], axis_vec[1], axis_vec[2],
                     color=colors[axis_idx], arrow_length_ratio=0.3,
                     linewidth=2, alpha=0.9)
    
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
              color='green', s=150, marker='o', label='Start', zorder=5)
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
              color='red', s=150, marker='s', label='End', zorder=5)
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('Lemniscate Trajectory with Attitude Frames (Colored by Time)', 
                fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Set equal aspect ratio
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
    cbar.set_label('Time (s)', fontsize=12)
    
    output_file_detailed = csv_file.replace('.csv', '_trajectory_detailed.png')
    plt.savefig(output_file_detailed, dpi=300, bbox_inches='tight')
    print(f"Detailed trajectory visualization saved to: {output_file_detailed}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        csv_file = os.path.expanduser('solutions/df_quaternion.csv')
        if not os.path.exists(csv_file):
            print(f"Usage: {sys.argv[0]} <csv_file> [output_image] [step]")
            print(f"Default path not found: {csv_file}")
            sys.exit(1)
    else:
        csv_file = sys.argv[1]
    
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    
    if not os.path.exists(csv_file):
        print(f"Error: CSV file not found: {csv_file}")
        sys.exit(1)
    
    visualize_trajectory(csv_file, output_file, step)


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot quaternion variation curves
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
import os

def plot_quaternion(csv_file, output_file=None):
    """
    Read CSV file and plot quaternion variation curves
    
    Args:
        csv_file: CSV file path
        output_file: Output image path (optional)
    """
    # Read CSV file
    try:
        df = pd.read_csv(csv_file, skipinitialspace=True)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Extract data
    t = df['t'].values
    x = df['x'].values
    y = df['y'].values
    z = df['z'].values
    w = df['w'].values
    
    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Quaternion Variation Over Time', fontsize=16, fontweight='bold')
    
    # Plot x component
    axes[0, 0].plot(t, x, 'r-', linewidth=1.5, label='x')
    axes[0, 0].set_xlabel('Time t (s)', fontsize=12)
    axes[0, 0].set_ylabel('Quaternion x', fontsize=12)
    axes[0, 0].set_title('Quaternion x Component', fontsize=13)
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    # Plot y component
    axes[0, 1].plot(t, y, 'g-', linewidth=1.5, label='y')
    axes[0, 1].set_xlabel('Time t (s)', fontsize=12)
    axes[0, 1].set_ylabel('Quaternion y', fontsize=12)
    axes[0, 1].set_title('Quaternion y Component', fontsize=13)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # Plot z component
    axes[1, 0].plot(t, z, 'b-', linewidth=1.5, label='z')
    axes[1, 0].set_xlabel('Time t (s)', fontsize=12)
    axes[1, 0].set_ylabel('Quaternion z', fontsize=12)
    axes[1, 0].set_title('Quaternion z Component', fontsize=13)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    # Plot w component
    axes[1, 1].plot(t, w, 'm-', linewidth=1.5, label='w')
    axes[1, 1].set_xlabel('Time t (s)', fontsize=12)
    axes[1, 1].set_ylabel('Quaternion w', fontsize=12)
    axes[1, 1].set_title('Quaternion w Component', fontsize=13)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    
    plt.tight_layout()
    
    # Save figure
    if output_file is None:
        output_file = csv_file.replace('.csv', '_quaternion.png')
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Figure saved to: {output_file}")
    
    # Show figure (if in interactive environment)
    # plt.show()
    
    # Create combined plot (all components in one figure)
    fig2, ax = plt.subplots(figsize=(12, 6))
    ax.plot(t, x, 'r-', linewidth=1.5, label='x', alpha=0.8)
    ax.plot(t, y, 'g-', linewidth=1.5, label='y', alpha=0.8)
    ax.plot(t, z, 'b-', linewidth=1.5, label='z', alpha=0.8)
    ax.plot(t, w, 'm-', linewidth=1.5, label='w', alpha=0.8)
    ax.set_xlabel('Time t (s)', fontsize=12)
    ax.set_ylabel('Quaternion Value', fontsize=12)
    ax.set_title('Quaternion Variation Over Time (All Components)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    output_file_combined = csv_file.replace('.csv', '_quaternion_combined.png')
    plt.savefig(output_file_combined, dpi=300, bbox_inches='tight')
    print(f"Combined figure saved to: {output_file_combined}")
    
    # Verify quaternion normalization
    norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
    print(f"\nQuaternion Normalization Check:")
    print(f"  Mean: {np.mean(norm):.10f}")
    print(f"  Min: {np.min(norm):.10f}")
    print(f"  Max: {np.max(norm):.10f}")
    print(f"  Std: {np.std(norm):.10f}")
    
    if np.allclose(norm, 1.0, atol=1e-6):
        print("  ✓ Quaternion is properly normalized")
    else:
        print("  ⚠ Warning: Quaternion is not fully normalized")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        # Default path
        csv_file = os.path.expanduser('solutions/df_quaternion.csv')
        if not os.path.exists(csv_file):
            print(f"Usage: {sys.argv[0]} <csv_file> [output_image]")
            print(f"Default path not found: {csv_file}")
            sys.exit(1)
    else:
        csv_file = sys.argv[1]
    
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(csv_file):
        print(f"Error: CSV file not found: {csv_file}")
        sys.exit(1)
    
    plot_quaternion(csv_file, output_file)


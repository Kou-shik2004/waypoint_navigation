#!/usr/bin/env python3
"""
Plot Generation Script for Trajectory Tracking Analysis

Generates error vs time plots from CSV metrics files exported by trajectory_monitor.
Creates high-resolution (300 DPI) plots suitable for reports and presentations.

Usage:
    python3 generate_plots.py
    
Output:
    plots/straight_error.png
    plots/circle_error.png
    plots/scurve_error.png
"""

import os
import numpy as np
import matplotlib.pyplot as plt


def parse_csv(csv_path):
    """
    Parse trajectory metrics CSV file exported by trajectory_monitor.
    
    Extracts summary statistics from header and time-series data.
    
    Args:
        csv_path: Path to CSV file
        
    Returns:
        dict: Contains 'rms', 'max', 'mean' (floats) and 'time', 'error' (numpy arrays)
    """
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    
    # Extract metrics from header
    metrics = {}
    for line in lines:
        if line.startswith('RMS Error'):
            metrics['rms'] = float(line.split(',')[1].strip())
        elif line.startswith('Max Error'):
            metrics['max'] = float(line.split(',')[1].strip())
        elif line.startswith('Mean Error'):
            metrics['mean'] = float(line.split(',')[1].strip())
    
    # Find start of time-series data
    data_start = None
    for i, line in enumerate(lines):
        if '# TIME-SERIES DATA' in line:
            data_start = i + 2  # Skip header line and column names
            break
    
    if data_start is None:
        raise ValueError(f"Could not find time-series data in {csv_path}")
    
    # Parse time-series data
    time_data = []
    error_data = []
    
    for line in lines[data_start:]:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        
        parts = line.split(',')
        if len(parts) >= 4:
            time_val = float(parts[0])
            error_val = float(parts[3])
            time_data.append(time_val)
            error_data.append(error_val)
    
    metrics['time'] = np.array(time_data)
    metrics['error'] = np.array(error_data)
    
    return metrics


def plot_error_vs_time(test_name, metrics, output_path):
    """
    Generate error vs time plot with statistics overlay.
    
    Creates publication-quality plot (300 DPI) showing tracking error over time
    with mean error baseline and RMS/Max statistics in title.
    
    Args:
        test_name: Name of test (e.g., 'straight', 'circle', 'scurve')
        metrics: Dictionary with rms, max, mean, time, error
        output_path: Path to save PNG file
    """
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 5), dpi=300)
    
    # Plot error line
    ax.plot(metrics['time'], metrics['error'], 
            color='#0066CC', linewidth=1.5, label='Tracking Error')
    
    # Plot mean error line
    ax.axhline(y=metrics['mean'], color='#DD0000', 
               linestyle='--', linewidth=1.5, 
               label=f"Mean: {metrics['mean']:.4f}m")
    
    # Formatting
    title = f"{test_name.upper()} - Tracking Error (RMS={metrics['rms']:.4f}m, Max={metrics['max']:.4f}m)"
    ax.set_title(title, fontsize=16, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=14)
    ax.set_ylabel('Cross-Track Error (m)', fontsize=14)
    
    # Grid and limits
    ax.grid(True, alpha=0.3, linestyle=':')
    ax.set_ylim(bottom=0, top=metrics['max'] * 1.1)  # 10% headroom
    
    # Legend
    ax.legend(loc='upper right', fontsize=12, framealpha=0.9)
    
    # Tight layout
    plt.tight_layout()
    
    # Save
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"  ✓ Saved: {output_path}")


def main():
    """Main execution function."""
    print("\n" + "="*60)
    print("TRAJECTORY ERROR PLOT GENERATOR")
    print("="*60 + "\n")
    
    # Configuration
    tests = {
        'straight': 'path_data/traj_straight.csv',
        'circle': 'path_data/traj_circle.csv',
        'scurve': 'path_data/traj_scurve.csv',
        'hybrid': 'path_data/traj_hybrid.csv'
    }
    
    # Create output directory
    output_dir = 'path_data/plots'
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}\n")
    
    # Process each test
    for test_name, csv_path in tests.items():
        print(f"Processing {test_name.upper()}...")
        
        # Load data
        print(f"  → Loading {csv_path}")
        metrics = parse_csv(csv_path)
        print(f"  → RMS Error: {metrics['rms']:.6f}m")
        print(f"  → Max Error: {metrics['max']:.6f}m")
        print(f"  → Mean Error: {metrics['mean']:.6f}m")
        print(f"  → Data points: {len(metrics['time'])}")
        
        # Generate plot
        output_path = f"{output_dir}/{test_name}_error.png"
        plot_error_vs_time(test_name, metrics, output_path)
        print()
    
    print("="*60)
    print("ALL PLOTS GENERATED SUCCESSFULLY")
    print(f"Location: {output_dir}/")
    print("Files:")
    print("  - straight_error.png")
    print("  - circle_error.png")
    print("  - scurve_error.png")
    print("  - hybrid_error.png")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()


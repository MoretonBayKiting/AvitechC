import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

def parse_debug_file(file_path):
    data = []
    current_pattern = None
    current_zone = None
    with open(file_path, 'r') as file:
        for line in file:
            # Match the pattern and zone line
            pattern_match = re.match(r'\d+:\d+:\d+\.\d+\s+RS\. Zone: (\d+) Pattern: (\d+)', line)
            if pattern_match:
                current_zone = int(pattern_match.group(1))
                current_pattern = int(pattern_match.group(2))
            
            # Match the line with X and Y values
            xy_match = re.match(r'\d+:\d+:\d+\.\d+\s+ind:\s*\d+,\s*X:\s*(-?\d+),\s*Y:\s*(-?\d+)', line)
            if xy_match and current_pattern is not None and current_zone is not None:
                x_value = int(xy_match.group(1))
                y_value = int(xy_match.group(2))
                data.append({'time': len(data), 'X': x_value, 'Y': y_value, 'pattern': current_pattern, 'zone': current_zone})
    return pd.DataFrame(data)

def plot_2d_render_of_3d(df, output_image_path):
    zones = df['zone'].unique()
    colors = ['blue', 'green', 'red', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
    for zone in zones:
        zone_df = df[df['zone'] == zone]
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')
        cumulative_time = 0
        for pattern in zone_df['pattern'].unique():
            pattern_df = zone_df[zone_df['pattern'] == pattern]
            time_steps = range(cumulative_time, cumulative_time + len(pattern_df))
            cumulative_time += len(pattern_df)
            ax.scatter(time_steps, pattern_df['X'], pattern_df['Y'], c=colors[pattern % len(colors)], label=f'Pattern {pattern}')
            ax.plot(time_steps, pattern_df['X'], pattern_df['Y'], linestyle='-', color=colors[pattern % len(colors)], alpha=0.5)
        ax.set_xlabel('Time Step')
        ax.set_ylabel('X')
        ax.set_zlabel('Y')
        ax.set_title(f'3D Scatter Plot with Time Order for Zone {zone}')
        ax.legend()
        plt.grid(True)
        # Save the plot to a file
        output_file = f"{output_image_path}_3d_Z{zone}.png"
        plt.savefig(output_file)
        plt.close()

def plot_2d_data(df, output_image_path):
    zones = df['zone'].unique()
    colors = ['blue', 'green', 'red', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
    for zone in zones:
        zone_df = df[df['zone'] == zone]
        patterns = zone_df['pattern'].unique()
        for pattern in patterns:
            pattern_df = zone_df[zone_df['pattern'] == pattern]
            plt.plot(pattern_df['X'], pattern_df['Y'], linestyle='-', marker='o', color=colors[pattern % len(colors)], label=f'Pattern {pattern}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(f'2D Plot for Zone {zone}')
        plt.legend()
        plt.grid(True)
        # Save the plot to a file
        output_file = f"{output_image_path}_2d_Z{zone}.png"
        plt.savefig(output_file)
        plt.close()

def export_to_csv(df, output_csv_path):
    df.to_csv(output_csv_path, index=False)

def main(file_path):
    df = parse_debug_file(file_path)
    base_name = os.path.splitext(file_path)[0]
    plot_2d_render_of_3d(df, base_name)
    plot_2d_data(df, base_name)
    csv_output_path = f"{base_name}.csv"
    export_to_csv(df, csv_output_path)
    print(f"Processed file saved to {base_name}_2d_Z<zone>.png, {base_name}_3d_Z<zone>.png, and {csv_output_path}")

if __name__ == '__main__':
    file_path = 'debug/Debug0110N.txt'  # Replace with your input file name
    main(file_path)
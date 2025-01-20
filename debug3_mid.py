import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

def decode_twos_complement(hex_value):
    """Decode a 4-digit hexadecimal string as a 2's complement integer."""
    value = int(hex_value, 16)
    if value >= 0x8000:
        value -= 0x10000
    return value

def parse_debug_file(file_path):
    data = []
    current_pattern = None
    current_zone = None
    with open(file_path, 'r') as file:
        for line in file:
            # Check if the line contains "RS. Zone"
            if "RS. Zone" in line:
                # Extract the zone value
                zone_start = line.find("Zone:") + len("Zone: ")
                zone_end = line.find(" ", zone_start)
                current_zone = int(line[zone_start:zone_end])

                # Extract the pattern value
                pattern_start = line.find("Pattern:") + len("Pattern: ")
                pattern_end = line.find(" ", pattern_start)
                current_pattern = int(line[pattern_start:pattern_end])
            
            # Check if the line contains X value
            match_34 = re.search(r'<34:([0-9a-fA-F]{4})>', line)
            if match_34 and current_pattern is not None and current_zone is not None:
                hex_value = match_34.group(1)
                x_value = decode_twos_complement(hex_value)
                data.append({'time': len(data), 'X': x_value, 'pattern': current_pattern, 'zone': current_zone})
            
            # Check if the line contains Y value
            match_35 = re.search(r'<35:([0-9a-fA-F]{4})>', line)
            if match_35 and current_pattern is not None and current_zone is not None:
                hex_value = match_35.group(1)
                y_value = decode_twos_complement(hex_value)
                data[-1]['Y'] = y_value
    return pd.DataFrame(data)

def plot_2d_data(df, output_image_path, axis):
    zones = df['zone'].unique()
    colors = ['blue', 'green', 'red', 'orange']
    for zone in zones:
        zone_df = df[df['zone'] == zone]
        plt.figure()
        for pattern in zone_df['pattern'].unique():
            pattern_df = zone_df[zone_df['pattern'] == pattern]
            time_steps = pattern_df['time']
            plt.scatter(time_steps, pattern_df[axis], label=f'Pattern {pattern}', color=colors[pattern % len(colors)])
            plt.plot(time_steps, pattern_df[axis], linestyle='-', color=colors[pattern % len(colors)], alpha=0.5)
        plt.xlabel('Time')
        plt.ylabel(axis)
        plt.title(f'2D Plot for Zone {zone}')
        plt.legend()
        plt.grid(True)
        # Save the plot to a file
        output_file = f"{output_image_path}_2d{axis}_Z{zone}.png"
        plt.savefig(output_file)
        plt.close()

def plot_2d_xy(df, output_image_path):
    zones = df['zone'].unique()
    colors = ['blue', 'green', 'red', 'orange']
    for zone in zones:
        zone_df = df[df['zone'] == zone]
        plt.figure()
        for pattern in zone_df['pattern'].unique():
            pattern_df = zone_df[zone_df['pattern'] == pattern]
            plt.scatter(pattern_df['X'], pattern_df['Y'], label=f'Pattern {pattern}', color=colors[pattern % len(colors)])
            plt.plot(pattern_df['X'], pattern_df['Y'], linestyle='-', color=colors[pattern % len(colors)], alpha=0.5)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(f'2D Plot for Zone {zone}')
        plt.legend()
        plt.grid(True)
        # Save the plot to a file
        output_file = f"{output_image_path}_2dXY_Z{zone}.png"
        plt.savefig(output_file)
        plt.close()

def plot_2d_render_of_3d(df, output_image_path):
    zones = df['zone'].unique()
    colors = ['blue', 'green', 'red', 'orange']
    for zone in zones:
        zone_df = df[df['zone'] == zone]
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')
        for pattern in zone_df['pattern'].unique():
            pattern_df = zone_df[zone_df['pattern'] == pattern]
            time_steps = pattern_df['time']
            ax.scatter(time_steps, pattern_df['X'], pattern_df['Y'], c=colors[pattern % len(colors)], label=f'Pattern {pattern}')
            ax.plot(time_steps, pattern_df['X'], pattern_df['Y'], linestyle='-', color=colors[pattern % len(colors)], alpha=0.5)
        ax.set_xlabel('Time')
        ax.set_ylabel('X')
        ax.set_zlabel('Y')
        plt.title(f'3D Plot for Zone {zone}')
        plt.legend()
        plt.grid(True)
        # Save the plot to a file
        output_file = f"{output_image_path}_3d_Z{zone}.png"
        plt.savefig(output_file)
        plt.close()

def export_to_csv(df, output_csv_path):
    df.to_csv(output_csv_path, index=False)

def main(file_path):
    df = parse_debug_file(file_path)
    base_name = os.path.splitext(file_path)[0]
    plot_2d_render_of_3d(df, base_name)
    plot_2d_data(df, base_name, 'X')
    plot_2d_data(df, base_name, 'Y')
    plot_2d_xy(df, base_name)
    csv_output_path = f"{base_name}.csv"
    export_to_csv(df, csv_output_path)
    print(f"Processed file saved to {base_name}_2dX_Z<zone>.png, {base_name}_2dY_Z<zone>.png, {base_name}_2dXY_Z<zone>.png, {base_name}_3d_Z<zone>.png, and {csv_output_path}")

if __name__ == '__main__':
    file_path = 'debug/Debug0121G.txt'  # Replace with your input file name
    main(file_path)
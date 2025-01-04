import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

def parse_debug_file(file_path):
    data = []
    current_pattern = None
    with open(file_path, 'r') as file:
        for line in file:
            # Match the pattern line
            pattern_match = re.match(r'\d+:\d+:\d+\.\d+\s+RS\. Zone: \d+ Pattern: (\d+)', line)
            if pattern_match:
                current_pattern = int(pattern_match.group(1))
            
            # Match the line with ind, X, Y, AbsX, AbsY values
            match = re.match(r'\d+:\d+:\d+\.\d+\s+<50:\s*ind:\s*(\d+),\s*X:\s*(-?\d+),\s*Y:\s*(-?\d+),\s*AbsX:\s*(-?\d+),\s*AbsY:\s*(-?\d+)>', line)
            if match and current_pattern is not None:
                ind, x, y, abs_x, abs_y = match.groups()
                data.append({'ind': int(ind), 'AbsX': int(abs_x), 'AbsY': int(abs_y), 'X': int(x), 'Y': int(y), 'pattern': current_pattern})
    return pd.DataFrame(data)

def plot_2d_render_of_3d(df, output_image_path):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    colors = ['blue', 'green', 'red', 'orange']
    cumulative_time = 0
    for pattern in df['pattern'].unique():
        pattern_df = df[df['pattern'] == pattern]
        time_steps = range(cumulative_time, cumulative_time + len(pattern_df))
        cumulative_time += len(pattern_df)
        ax.scatter(time_steps, pattern_df['X'], pattern_df['Y'], c=colors[pattern % len(colors)], label=f'Pattern {pattern}')
        ax.plot(time_steps, pattern_df['X'], pattern_df['Y'], linestyle='-', color=colors[pattern % len(colors)], alpha=0.5)
    ax.set_xlabel('Time Step')
    ax.set_ylabel('X')
    ax.set_zlabel('Y')
    ax.set_title('3D Scatter Plot with Time Order')
    ax.legend()
    plt.grid(True)
    # Save the plot to a file
    plt.savefig(output_image_path)
    plt.close()

def plot_2d_data(df, output_image_path):
    plt.figure(figsize=(10, 6))
    colors = ['blue', 'green', 'red', 'orange']
    for pattern in df['pattern'].unique():
        pattern_df = df[df['pattern'] == pattern]
        plt.scatter(pattern_df['X'], pattern_df['Y'], c=colors[pattern % len(colors)], label=f'Pattern {pattern}')
        plt.plot(pattern_df['X'], pattern_df['Y'], linestyle='-', color=colors[pattern % len(colors)], alpha=0.5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Scatter Plot with Time Order')
    plt.legend()
    plt.grid(True)
    # Save the plot to a file
    plt.savefig(output_image_path)
    plt.close()

def main(name):
    file_path = f'debug/Debug{name}.txt'  # Input file path
    output_csv_path = f'debug/Coords{name}.csv'  # Output CSV path

    df = parse_debug_file(file_path)

    output_3d_image_path = f'debug/chart3d{name}.png'  # Output 3D image path
    output_2d_image_path = f'debug/chart{name}.png'  # Output 2D image path
    plot_2d_render_of_3d(df, output_3d_image_path)
    plot_2d_data(df, output_2d_image_path)
    # Save the DataFrame to a CSV file
    df.to_csv(output_csv_path, index=False)

if __name__ == '__main__':
    main('20241226L_Z0')  # Replace with your desired name parameter
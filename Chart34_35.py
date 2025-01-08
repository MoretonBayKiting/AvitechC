import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import os

def parse_debug_file(file_path):
    data = []
    current_x = None
    current_y = None
    current_time = None
    with open(file_path, 'r') as file:
        for line in file:
            # Extract the timestamp
            time_match = re.match(r'(\d+:\d+:\d+\.\d+)', line)
            if time_match:
                current_time = time_match.group(1)
            
            # Match the line with X value in <34:0x> format
            x_match = re.match(r'\d+:\d+:\d+\.\d+\s+<34:([0-9a-fA-F]+)>', line)
            if x_match:
                current_x = int(x_match.group(1), 16)
                if current_x & 0x8000:  # Handle 2's complement for 16-bit values
                    current_x -= 0x10000
            
            # Match the line with Y value in <35:0x> format
            y_match = re.match(r'\d+:\d+:\d+\.\d+\s+<35:([0-9a-fA-F]+)>', line)
            if y_match:
                current_y = int(y_match.group(1), 16)
                if current_y & 0x8000:  # Handle 2's complement for 16-bit values
                    current_y -= 0x10000
            
            # If both X and Y values are found, append them to the data
            if current_x is not None and current_y is not None:
                data.append({'time': current_time, 'X': current_x, 'Y': current_y})
                current_x = None
                current_y = None

    return pd.DataFrame(data)

def plot_2d_render_of_3d(df, output_image_path):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    time_steps = range(len(df))
    ax.scatter(time_steps, df['X'], df['Y'], c='blue', label='Data Points')
    ax.plot(time_steps, df['X'], df['Y'], linestyle='-', color='blue', alpha=0.5)
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
    plt.scatter(df['X'], df['Y'], c='blue', label='Data Points')
    plt.plot(df['X'], df['Y'], linestyle='-', color='blue', alpha=0.5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Scatter Plot')
    plt.legend()
    plt.grid(True)
    # Save the plot to a file
    plt.savefig(output_image_path)
    plt.close()

if __name__ == '__main__':
    filename = 'debug/Debug0108M.txt'  # Replace with your input file name
    base_filename = os.path.splitext(filename)[0]
    output_image_path_3d = f'{base_filename}_3d.png'
    output_image_path_2d = f'{base_filename}_2d.png'
    output_csv_path = f'{base_filename}.csv'
    df = parse_debug_file(filename)
    plot_2d_render_of_3d(df, output_image_path_3d)
    plot_2d_data(df, output_image_path_2d)
    df.to_csv(output_csv_path, index=False)
    print(f"Plots saved to {output_image_path_3d} and {output_image_path_2d}")
    print(f"CSV saved to {output_csv_path}")
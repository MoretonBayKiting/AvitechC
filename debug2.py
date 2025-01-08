import pandas as pd
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

def parse_debug_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        current_pattern = None
        for line in file:
            pattern_match = re.match(r'\d+:\d+:\d+\.\d+\s+RS\. Zone: \d+ Pattern: (\d+)', line)
            if pattern_match:
                current_pattern = int(pattern_match.group(1))
            match = re.match(r'(\d+:\d+:\d+\.\d+)\s+X:\s*(-?\d+),\s+Y:\s*(-?\d+)', line)
            if match and current_pattern is not None:
                time, x, y = match.groups()
                data.append({'time': time, 'X': int(x), 'Y': int(y), 'pattern': current_pattern})
    return pd.DataFrame(data)

def plot_3d_data(df, output_html_path):
    time_steps = list(range(len(df)))
    fig = go.Figure(data=[go.Scatter3d(
        x=time_steps,
        y=df['X'],
        z=df['Y'],
        mode='markers+lines',
        marker=dict(size=5, color='blue', opacity=0.8),
        line=dict(color='blue', width=2)
    )])
    fig.update_layout(
        scene=dict(
            xaxis=dict(title='Time Step'),
            yaxis=dict(title='X'),
            zaxis=dict(title='Y')
        ),
        title='3D Scatter Plot with Time Order'
    )
    # Save the plot to an HTML file
    fig.write_html(output_html_path)

def plot_2d_render_of_3d(df, output_image_path):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    time_steps = range(len(df))
    ax.scatter(time_steps, df['X'], df['Y'], c='blue', label='Points')
    ax.plot(time_steps, df['X'], df['Y'], linestyle='-', color='blue', alpha=0.5, label='Path')
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
    plt.scatter(df['X'], df['Y'], c='blue', label='Points')
    plt.plot(df['X'], df['Y'], linestyle='-', color='blue', alpha=0.5, label='Path')
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
    df = parse_debug_file(file_path)

    for pattern in df['pattern'].unique():
        pattern_df = df[df['pattern'] == pattern]
        output_html_path = f'debug/chart{name}_pattern{pattern}.html'  # Output HTML path
        output_3d_image_path = f'debug/chart3d{name}_pattern{pattern}.png'  # Output 3D image path
        output_2d_image_path = f'debug/chart{name}_pattern{pattern}.png'  # Output 2D image path
        output_csv_path = f'debug/Coords{name}_pattern{pattern}.csv'  # Output CSV path

        plot_3d_data(pattern_df, output_html_path)
        plot_2d_render_of_3d(pattern_df, output_3d_image_path)
        plot_2d_data(pattern_df, output_2d_image_path)
        # Save the DataFrame to a CSV file
        pattern_df.to_csv(output_csv_path, index=False)

if __name__ == '__main__':
    main('20241205XA')  # Replace with your desired name parameter
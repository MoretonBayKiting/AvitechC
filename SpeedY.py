import pandas as pd
import matplotlib.pyplot as plt

def parse_debug_file(file_path):
    data = []
    current_speed_scale = "No <16:n>"
    with open(file_path, 'r') as file:
        for line in file:
            # Check for SpeedScale16 value
            if "<16:" in line:
                start = line.find("<16:") + len("<16:")
                end = line.find(">", start)
                current_speed_scale = int(line[start:end])
            
            # Check for speed and Y values
            if "speed:" in line and "Y:" in line:
                parts = line.split(',')
                speed_part = parts[0].strip()
                y_part = parts[1].strip()
                
                speed_start = speed_part.find("speed:") + len("speed:")
                speed = int(speed_part[speed_start:].strip())
                
                y_start = y_part.find("Y:") + len("Y:")
                y_value = int(y_part[y_start:].strip())
                
                data.append({'SpeedScale16': current_speed_scale, 'speed': speed, 'AbsY': y_value})
    
    return pd.DataFrame(data)

def plot_data(df, output_image_path):
    fig, ax1 = plt.subplots()

    # Plot speed
    ax1.set_xlabel('Time (notional)')
    ax1.set_ylabel('Speed', color='tab:blue')
    ax1.plot(df.index, df['speed'], label='Speed', color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')

    # Create a second y-axis for Y values
    ax2 = ax1.twinx()
    ax2.set_ylabel('AbsY', color='tab:orange')
    ax2.plot(df.index, df['AbsY'], label='AbsY', color='tab:orange')
    ax2.tick_params(axis='y', labelcolor='tab:orange')

    # Add vertical bars to separate SpeedScale16 values
    unique_speed_scales = df['SpeedScale16'].unique()
    for speed_scale in unique_speed_scales:
        indices = df[df['SpeedScale16'] == speed_scale].index
        if len(indices) > 0:
            ax1.axvline(x=indices[0], color='gray', linestyle='--', linewidth=0.5)
            ax1.text(indices[0], ax1.get_ylim()[1], f'SpeedScale16: {speed_scale}', rotation=90, verticalalignment='bottom')

    fig.tight_layout()
    plt.title('Speed and Tilt by Time')
    plt.savefig(output_image_path)
    plt.close()

def plot_scatter(df, output_image_path):
    fig, ax = plt.subplots()

    # Plot Speed vs AbsY with different series for different SpeedScale16 values
    unique_speed_scales = df['SpeedScale16'].unique()
    for speed_scale in unique_speed_scales:
        subset = df[df['SpeedScale16'] == speed_scale]
        ax.scatter(subset['speed'], subset['AbsY'], label=f'SpeedScale16: {speed_scale}')

    ax.set_xlabel('Speed')
    ax.set_ylabel('AbsY')
    ax.legend()
    plt.title('Speed vs AbsY')
    plt.savefig(output_image_path)
    plt.close()

def main():
    name = '0124C'
    file_path = f'debug/Debug{name}.txt'  # Replace with your input file path
    output_image_path = f'debug/SpeedY{name}.png'  # Replace with your desired output image file path
    scatter_output_image_path = f'debug/SpeedVsAbsY{name}.png'  # Output path for scatter plot
    csv_path = f'debug/SpeedY{name}.csv'
    df = parse_debug_file(file_path)
    # print(df)
    df.to_csv(csv_path)
    # Plot the data and save to a PNG file
    plot_data(df, output_image_path)
    # Plot the scatter chart and save to a PNG file
    plot_scatter(df, scatter_output_image_path)

if __name__ == '__main__':
    main()
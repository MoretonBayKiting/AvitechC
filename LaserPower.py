import pandas as pd
import matplotlib.pyplot as plt

def parse_debug_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            # Check for B_volt, SR, ULP, MLP, and LP values
            if "B_volt:" in line and "SR:" in line and "ULP:" in line and "MLP:" in line and "LP:" in line:
                parts = line.split(',')
                b_volt_part = parts[0].strip()
                sr_part = parts[1].strip()
                ulp_part = parts[2].strip()
                mlp_part = parts[3].strip()
                lp_part = parts[4].strip()
                
                b_volt_start = b_volt_part.find("B_volt:") + len("B_volt:")
                b_volt = int(b_volt_part[b_volt_start:].strip())
                
                sr_start = sr_part.find("SR:") + len("SR:")
                sr = int(sr_part[sr_start:].strip())
                
                ulp_start = ulp_part.find("ULP:") + len("ULP:")
                ulp = int(ulp_part[ulp_start:].strip())
                
                mlp_start = mlp_part.find("MLP:") + len("MLP:")
                mlp = int(mlp_part[mlp_start:].strip())
                
                lp_start = lp_part.find("LP:") + len("LP:")
                lp = int(lp_part[lp_start:].strip())
                
                data.append({'B_volt': b_volt, 'SR': sr, 'ULP': ulp, 'MLP': mlp, 'LP': lp})
    
    return pd.DataFrame(data)

def plot_data(df, output_image_path):
    fig, ax1 = plt.subplots()

    # Plot SR
    ax1.set_xlabel('Time (notional)')
    ax1.set_ylabel('SR', color='tab:blue')
    ax1.plot(df.index, df['SR'], label='SR', color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')

    # Create a second y-axis for ULP, MLP, and LP values
    ax2 = ax1.twinx()
    ax2.set_ylabel('ULP, MLP, LP', color='tab:orange')
    ax2.plot(df.index, df['ULP'], label='ULP', color='tab:orange')
    ax2.plot(df.index, df['MLP'], label='MLP', color='tab:green')
    ax2.plot(df.index, df['LP'], label='LP', color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:orange')

    fig.tight_layout()
    plt.title('SR, ULP, MLP, and LP as a Function of Time')
    fig.legend(loc='upper right', bbox_to_anchor=(1,1), bbox_transform=ax1.transAxes)
    plt.savefig(output_image_path)
    plt.close()

def main():
    name = '0124D'
    file_path = f'debug/Debug{name}.txt'  # Replace with your input file path
    output_image_path = f'debug/LaserPower{name}.png'  # Replace with your desired output image file path
    df = parse_debug_file(file_path)
    # print(df)
    # Plot the data and save to a PNG file
    plot_data(df, output_image_path)

if __name__ == '__main__':
    main()
import pandas as pd
import matplotlib.pyplot as plt

def parse_debug_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            # Check for BV, LL, SR, ULP, MLP, and LP values
            if "BV:" in line and "LL:" in line and "SR:" in line and "ULP:" in line and "MLP:" in line and "LP:" in line:
                parts = line.split(',')
                BV_part = parts[0].strip()
                LL_part = parts[1].strip()
                sr_part = parts[2].strip()
                ulp_part = parts[3].strip()
                mlp_part = parts[4].strip()
                lp_part = parts[5].strip()
                
                BV_start = BV_part.find("BV:") + len("BV:")
                BV = int(BV_part[BV_start:].strip())
                
                LL_start = LL_part.find("LL:") + len("LL:")
                LL = int(LL_part[LL_start:].strip())

                sr_start = sr_part.find("SR:") + len("SR:")
                sr = int(sr_part[sr_start:].strip())
                
                ulp_start = ulp_part.find("ULP:") + len("ULP:")
                ulp = int(ulp_part[ulp_start:].strip())
                
                mlp_start = mlp_part.find("MLP:") + len("MLP:")
                mlp = int(mlp_part[mlp_start:].strip())
                
                lp_start = lp_part.find("LP:") + len("LP:")
                lp = int(lp_part[lp_start:].strip())
                
                data.append({'BV': BV, 'LL': LL, 'SR': sr, 'ULP': ulp, 'MLP': mlp, 'LP': lp})
    
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
    name = '0127A'
    file_path = f'debug/Debug{name}.txt'  # Replace with your input file path
    output_image_path = f'debug/LaserPower{name}.png'  # Replace with your desired output image file path
    csv_path = f'debug/LaserPower{name}.csv'
    df = parse_debug_file(file_path)
    df.to_csv(csv_path, index=False)
    # print(df)
    # Plot the data and save to a PNG file
    plot_data(df, output_image_path)

if __name__ == '__main__':
    main()
import pandas as pd
import matplotlib.pyplot as plt
import re

# Function to parse the log file
def parse_log_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            # Match boundary points
            match_boundary = re.match(r'(\d+:\d+:\d+\.\d+)\s+pat\s+(\d+),\s+seg\s+(\d+),\s+segPt\s+(\d+),\s+wigglyPt\s+(\d+),\s+nbrSegPts\s+(\d+),\s+ind\s+(\d+),\s+X\s+(-?\d+),\s+Y\s+(-?\d+)', line)
            if match_boundary:
                _, pat, seg, segPt, wigglyPt, nbrSegPts, ind, X, Y = match_boundary.groups()
                data.append({
                    'pat': int(pat),
                    'seg': str(seg),  # Convert seg to string
                    'X': int(X),
                    'Y': int(Y)
                })
                continue

            # Match zone crossing points
            match_zone = re.match(r'(\d+:\d+:\d+\.\d+)\s+pat\s+(\d+),\s*rnd\s+(\d+),\s*nbrRungs\s+(\d+),\s*tilt\s+(\d+),\s*rhoMin\s+(\d+),\s*fstSeg\s+(\d+),\s*sndSeg\s+(\d+),\s*X\s+(-?\d+),\s*Y\s+(-?\d+)', line)
            if match_zone:
                _, pat, rnd, nbrRungs, tilt, rhoMin, fstSeg, sndSeg, X, Y = match_zone.groups()
                data.append({
                    'pat': int(pat),
                    'seg': 'cross',  # Label all zone crossing points as 'cross'
                    'X': int(X),
                    'Y': int(Y)
                })
    return pd.DataFrame(data)

# Function to plot the data
def plot_data(df):
    patterns = df['pat'].unique()
    for pat in patterns:
        pat_data = df[df['pat'] == pat]
        # Export DataFrame to CSV
        pat_data.to_csv(f'debug/pattern_{pat}.csv', index=False)
        # Print DataFrame for review
        # print(f'Data for Pattern {pat}:\n', pat_data)
        plt.figure(figsize=(10, 6))
        for seg in pat_data['seg'].unique():
            seg_data = pat_data[pat_data['seg'] == seg]
            plt.plot(seg_data['X'], seg_data['Y'], marker='o' if seg != 'cross' else 'x', linestyle='-' if seg != 'cross' else '--', label=f'Segment {seg}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(f'X, Y Plot for Pattern {pat}')
        plt.legend()
        plt.grid(True)
        # Save the plot to a file
        plt.savefig(f'debug/pattern_{pat}.png')
        plt.close()

# Main function
def main():
    file_path = 'debug/Debug.txt'  # Use the provided file path
    df = parse_log_file(file_path)
    plot_data(df)

if __name__ == '__main__':
    main()
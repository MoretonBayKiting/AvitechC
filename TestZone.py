import re

# Function to parse the log file for zone crossing points
def parse_zone_crossings(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            # Match zone crossing points
            match_zone = re.match(r'(\d+:\d+:\d+\.\d+)\s+pat\s+(\d+),\s*rnd\s+(\d+),\s*nbrRungs\s+(\d+),\s*tilt\s+(\d+),\s*rhoMin\s+(\d+),\s*fstSeg\s+(\d+),\s*sndSeg\s+(\d+),\s*X\s+(-?\d+),\s*Y\s+(-?\d+)', line)
            if match_zone:
                time, pat, rnd, nbrRungs, tilt, rhoMin, fstSeg, sndSeg, X, Y = match_zone.groups()
                if int(pat) == 1:
                    print(f"Match found for zone crossing: pat={pat}, rnd={rnd}, X={X}, Y={Y}")

# Main function
def main():
    file_path = 'debug/Debug.txt'  # Use the provided file path
    parse_zone_crossings(file_path)

if __name__ == '__main__':
    main()
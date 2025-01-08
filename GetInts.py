import re

def convert_debug_file(input_filename):
    output_filename = input_filename.replace('.txt', '_int.txt')
    
    def hex_to_signed_int(hex_str):
        value = int(hex_str, 16)
        if value >= 0x8000:  # Check if the value is negative in 16-bit signed integer
            value -= 0x10000
        return value
    
    with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile:
        for line in infile:
            line = line.strip()
            print(f"Processing line: {line}")  # Debugging print
            # Match the pattern with timestamp <cmd:inst>
            match = re.match(r'(\d+:\d+:\d+\.\d+)\s+<(\d+):([0-9a-fA-F]+)>', line)
            if match:
                timestamp = match.group(1)
                cmd = int(match.group(2))
                inst_hex = match.group(3)
                inst_int = hex_to_signed_int(inst_hex)
                
                # Append the integer value for cmd 34 or 35
                if cmd == 34 or cmd == 35:
                    print(f"Matched: {timestamp} <{cmd}:{inst_hex}> ({inst_int})")  # Debugging print
                    outfile.write(f'{timestamp} <{cmd}:{inst_hex}> ({inst_int})\n')
                else:
                    outfile.write(f'{timestamp} <{cmd}:{inst_hex}>\n')
            else:
                print(f"No match: {line}")  # Debugging print
                outfile.write(line + '\n')

if __name__ == '__main__':
    input_filename = 'debug/Debug20241225I.txt'  # Replace with your input file name
    convert_debug_file(input_filename)
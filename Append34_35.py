import re

def process_file(file_path):
    def decode_twos_complement(hex_value, bits=16):
        """Decode a 2's complement encoded hexadecimal value."""
        value = int(hex_value, 16)
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value

    with open(file_path, 'r') as file:
        lines = file.readlines()

    with open(file_path, 'w') as file:
        for line in lines:
            # Search for <34:%04x> or <35:%04x> patterns
            match_34 = re.search(r'<34:([0-9a-fA-F]{4})>', line)
            match_35 = re.search(r'<35:([0-9a-fA-F]{4})>', line)
            
            if match_34:
                hex_value = match_34.group(1)
                decimal_value = decode_twos_complement(hex_value)
                line = line.strip() + f' (34: {decimal_value})\n'
            
            if match_35:
                hex_value = match_35.group(1)
                decimal_value = decode_twos_complement(hex_value)
                line = line.strip() + f' (35: {decimal_value})\n'
            
            file.write(line)

if __name__ == '__main__':
    file_path = 'debug\Debug20250104H.txt'  # Replace with your input file name
    process_file(file_path)
    print(f"Processed file saved to {file_path}")
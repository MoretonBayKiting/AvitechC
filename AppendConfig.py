import re
import os

# Define the command mappings for TransmitData(), PrintAppData(), and PrintConfigData()
command_mappings = {
    # TransmitData() commands
    20: "MaxLaserPower",         # Variables[0] = MaxLaserPower;   // 20
    30: "AccelTemp",             # Variables[1] = Accel.Acceltemp; // 30
    31: "Index",                 # Variables[2] = Index;           // 31
    32: "X",                     # Variables[3] = X;               // 32
    33: "Y",                     # Variables[4] = Y;               // 33
    34: "AbsX",                  # Variables[5] = AbsX;            // 34
    35: "AbsY",                  # Variables[6] = AbsY;            // 35
    36: "Z_accel",               # Variables[7] = Accel_Z.Z_accel; // 36
    37: "Tod_tick",              # Variables[8] = Tod_tick / 2;    // 37 T
    38: "BattVoltAvg",           # Variables[9] = BattVoltAvg;     // 38
    39: "Frame_size",            # Variables[10] = Frame_size;     // 39
    42: "Wd_flag",               # Variables[10] = Wd_flag;        // 42
    43: "Boardrevision",         # Variables[11] = Boardrevision;  // 43
    44: "LaserID",               # Variables[12] = LaserID;        // 44
    46: "DSS_preload",           # Variables[13] = DSS_preload;    // 46
    47: "AccelTripPoint",        # Variables[14] = AccelTripPoint; // 47
    48: "ResetSeconds",          # Variables[15] = ResetSeconds / 2; // 48
    49: "OperationMode",         # Variables[16] = OperationMode;  // 49

    # PrintAppData() commands
    17: "MapRunning",
    18: "PatternRunning",
    2: "LaserTemperature",
    3: "BatteryVoltage",
    21: "UserLaserPower",
    25: "LightLevel",
    45: "LaserPower",

    # PrintConfigData() commands
    15: "ActiveMapZones",
    4: "MapTotalPoints",
    23: "EramMicroMajor",
    24: "EramMicroMinor",
    26: "UserLightTripLevel",
    27: "LightTriggerOperation",
    28: "Laser2OperateFlag",
    29: "FactoryLightTripLevel",
    50: "Laser2TempTrip",
    51: "Laser2BattTrip",
    53: "NIGHT_TRIP_TIME_FROM_STARTUP"
}

# Function to convert 2's complement hexadecimal to signed integer
def twos_complement(hex_str, bits):
    value = int(hex_str, 16)
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

# Function to translate command to human-readable format
def translate_command(command, instruction, original_line):
    if command in command_mappings:
        description = command_mappings[command]
        value = twos_complement(instruction, 16)  # Assuming 16-bit values
        return f"{original_line.strip()} ({description}: {value})"
    else:
        return original_line.strip()

# Read the serial output file
input_file = "debug/Debug0112E.txt"
temp_file = "debug/Debug0112E_temp.txt"

# Process the file line by line and write to the temporary file
with open(input_file, "r") as infile, open(temp_file, "w") as outfile:
    for line in infile:
        if '<' in line and '>' in line:
            start = line.find('<') + 1
            end = line.find('>')
            content = line[start:end]
            command_str, instruction = content.split(':')
            command = int(command_str)
            translated_line = translate_command(command, instruction, line)
            outfile.write(translated_line + "\n")
        else:
            outfile.write(line)

# Replace the original file with the temporary file
os.replace(temp_file, input_file)

print("Translation complete. Translated output saved to", input_file)
import intelhex

# Define the addresses of the EEPROM variables
eeprom_addresses = {
    "LaserPower": 0x0000,
    "MaxLaserPower": 0x0001,
    "Laser2TempTrip": 0x0002,
    "MapTotalPoints": 0x0003,
    "GyroAddress": 0x0004,
    "ActiveMapZones": 0x0005,
    "ActivePatterns": 0x0006,
    "LaserID": 0x0007,
    "AccelTrip": 0x0008,
    "OperationMode": 0x0009,
    "FirstTimeOn": 0x000A,
    "UserLightTripLevel": 0x000B,
    "FactoryLightTripLevel": 0x000C,
    "LightTriggerOperation": 0x000D,
}

def read_eeprom_values(hex_file, addresses):
    ih = intelhex.IntelHex(hex_file)
    values = {}
    for name, address in addresses.items():
        values[name] = ih[address]
    return values

def main():
    hex_file = 'eeprom_dump.hex'  # Replace with your Intel HEX file path
    values = read_eeprom_values(hex_file, eeprom_addresses)
    for name, value in values.items():
        print(f"{name}: {value}")

if __name__ == '__main__':
    main()
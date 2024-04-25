#!/usr/bin/env python3.10

from smbus2 import SMBus, i2c_msg

# MPU6050 details
MPU6050_DEFAULT_ADDRESS = 0x68
MPU6050_ALTERNATE_ADDRESS = 0x69
WHO_AM_I_REG = 0x75

# Function to read the WHO_AM_I register from the MPU6050
def read_who_am_i(bus_number, address):
    with SMBus(bus_number) as bus:
        # Read a single byte from the WHO_AM_I register
        who_am_i = bus.read_byte_data(address, WHO_AM_I_REG)
        return who_am_i

# Test each address on each MPU6050
for i2c_bus in [1, 2, 3]:
    for address in [MPU6050_DEFAULT_ADDRESS, MPU6050_ALTERNATE_ADDRESS]:
        try:
            who_am_i_value = read_who_am_i(i2c_bus, address)
            print(f"Bus {i2c_bus} Address {hex(address)}: WHO_AM_I register value = {hex(who_am_i_value)}")
            if who_am_i_value == MPU6050_DEFAULT_ADDRESS:
                print(f"MPU6050 on bus {i2c_bus} at address {hex(address)} is responding correctly.")
            else:
                print(f"MPU6050 on bus {i2c_bus} at address {hex(address)} is not responding correctly.")
        except OSError as e:
            print(f"No response from MPU6050 on bus {i2c_bus} at address {hex(address)}. Error: {e}")


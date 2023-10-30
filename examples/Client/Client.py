"""
Host-side MODBUS server (master) app for the sensors PCB.
It polls for new sensors measurements from the PCB, which operates as a MODBUS client (slave).
"""
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import sys

if len(sys.argv) != 2:
    print(f'Host-side MODBUS server (master) app for the sensors PCB')
    print(f'It polls for new sensors measurements from the PCB, which operates as a MODBUS client (slave)')
    print(f'Usage: {sys.argv[0]} <port>')
    print(f'Example: {sys.argv[0]} /dev/ttyUSB0')
    sys.exit(1)

serial_port = sys.argv[1]
serial_baudrate = 9600

client = ModbusClient(method='rtu', port=serial_port, baudrate=serial_baudrate, timeout=1)

client.connect()

# 0 for master, 1-247 for slave
device_id = 1

# Activity bit mask:
# 0 - device inactive
# 1 << 1 - first value is collected
# 1 << 2 - second value is collected
# 1 << 3 - third, forth and fifth values are collected
ActiveRegAddr = 0x0

# First value.
Value1RegAddr = 0x1

# Second value.
Value2RegAddr = 0x2

# Third value (3 components).
Value3RegAddr = 0x3

# Activate collection of all 3 values
active_mask = (1 << Value1RegAddr) | (1 << Value2RegAddr) | (1 << Value3RegAddr)
client.write_register(ActiveRegAddr, active_mask, unit = device_id)

#time.sleep(1)

# Read back the active mask.
read = client.read_holding_registers(address = ActiveRegAddr, count = 1, unit = device_id)
if read.isError():
    print(f'MODBUS error, while checking the active mask')
    sys.exit(-1)
if read.registers[0] != active_mask:
    print(f'MODBUS active mask I/O failure: expected {active_mask}, got {read.registers[0]}')
    sys.exit(-1)

# Read collected values in a loop
old_values = [0] * 5
while True:
    read = client.read_holding_registers(address = Value1RegAddr, count = 5, unit = device_id)
    if read.isError():
        print(f'MODBUS error, while checking the active mask')
        sys.exit(-1)
    
    values = read.registers
    print(f'Value1 = {values[0]}')
    print(f'Value2 = {values[1]}')
    print(f'Value3 = {values[2]}, {values[3]}, {values[4]}')
    
    for i in range (0, 5):
        if old_values[i] >= values[i]:
            print(f'New value must be always greater than old one, but old_values[{i}] >= values[{i}]: {old_values[i]} >= {values[i]}')
            sys.exit(-1)

    old_values = values

client.close()


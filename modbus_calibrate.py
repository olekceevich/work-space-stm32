import minimalmodbus
from time import sleep

instrument = minimalmodbus.Instrument('COM54', 0x03)    # port name, slave address (in decimal)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 0.1

print("start")
instrument.write_register(19, 0xAA, 0)
print("ok")
sleep(2)

print("V = ", instrument.read_register(0, signed=True, number_of_decimals=1)) # Read voltage
print("I = ", instrument.read_register(1, signed=True, number_of_decimals=1)) # Read current




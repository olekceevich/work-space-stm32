import minimalmodbus
from time import sleep

VOLT_REG_ADDR = 0
CURR_REG_ADDR = 1
CRHG_REG_ADDR = 2
RLED_REG_ADDR = 3
GLED_REG_ADDR = 4
BLED_REG_ADDR = 5
MPOW_REG_ADDR = 6
ACTR_REG_ADDR = 7
BEEP_REG_ADDR = 8
ALRM_REG_ADDR = 9
BAUD_REG_ADDR = 10
ROID_REG_ADDR = 11
ROGR_REG_ADDR = 12
BTIM_REG_ADDR = 13
FWID_REG_ADDR = 18
CALI_REG_ADDR = 19

instrument = minimalmodbus.Instrument('COM54', 0x03)    # port name, slave address (in decimal)
instrument.serial.baudrate = 115200                     # Baud


instrument.write_register(ALRM_REG_ADDR, 0, 0) # Reset alarm

instrument.write_register(BEEP_REG_ADDR, 1, 0)
sleep(0.1)
instrument.write_register(BEEP_REG_ADDR, 0, 0)

instrument.write_register(RLED_REG_ADDR, 0xFF00, 0) # R2R1
instrument.write_register(GLED_REG_ADDR, 0x0000, 0) # G2G1
instrument.write_register(BLED_REG_ADDR, 0x00FF, 0) # B2B1
instrument.write_register(BTIM_REG_ADDR, 250, 0) # blink period per color in ms
sleep(2)
instrument.write_register(RLED_REG_ADDR, 0, 0) # R
instrument.write_register(GLED_REG_ADDR, 0, 0) # G
instrument.write_register(BLED_REG_ADDR, 0, 0) # B
instrument.write_register(BTIM_REG_ADDR, 0, 0) # blink period

print("V = ", instrument.read_register(VOLT_REG_ADDR, signed=True, number_of_decimals=1)) # Read voltage
print("I = ", instrument.read_register(CURR_REG_ADDR, signed=True, number_of_decimals=1)) # Read current

print(instrument.read_register(CRHG_REG_ADDR))  # get charging state
print(instrument.read_register(FWID_REG_ADDR))  # get firmware id
print(instrument.read_register(ACTR_REG_ADDR))  # get lift position

if 0:
    instrument.write_register(MPOW_REG_ADDR, 1, 0)
    sleep(0.1)
    instrument.write_register(ACTR_REG_ADDR, 1, 0)
    sleep(3)
    instrument.write_register(ACTR_REG_ADDR, 0, 0)
    sleep(0.1)
    instrument.write_register(ACTR_REG_ADDR, 2, 0)
    sleep(3)
    instrument.write_register(ACTR_REG_ADDR, 0, 0)
    sleep(0.1)
    instrument.write_register(MPOW_REG_ADDR, 0, 0)



import serial
import libscrc
import minimalmodbus
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("file", help="Firmware file path")
parser.add_argument("port", help="Serial port name")
parser.add_argument("-baud", help="Modbus baud rate", default = 115200)
parser.add_argument("-nm", help="Use when bootloader is already running (no firmware loaded)", action="store_true")
args = parser.parse_args()

baudrate = int(args.baud)

print("Starting...")
print(args.file)
print(args.port)
print("Speed: %d" % baudrate)

if args.nm:
    print("No MODBUS")
else:
    instrument = minimalmodbus.Instrument(args.port, 0x03)
    instrument.serial.baudrate = baudrate
    instrument.write_register(20, 0xBB, 0)
    instrument.serial.close()

ser = serial.Serial(args.port, 115200, timeout=3.0)
f = open(args.file, 'rb')

CRC_PAGE_NUM = 50

checksum = 0xFFFF
page_num = 0
while True:
    data = f.read(1024)
    if not data:
        break

    ser.write(bytes([0x01]))
    ans = ser.read(1)

    if ans[0] == 0x01:
        if page_num != CRC_PAGE_NUM:
            checksum = libscrc.modbus(data, checksum)
        else:
            checksum = libscrc.modbus(data[:-2], checksum)
            data = data[:-2] + checksum.to_bytes(2, "little")

        data = bytes([page_num, 0x00]) + data
        crc = libscrc.modbus(data)
        data = data + crc.to_bytes(2, "big")
        ser.write(data)
        ans = ser.read(1)

        if ans[0] == 0x01:
            print("Page #%d"%page_num + "...successful")
        else:
            print("Page #%d" % page_num + "...error")

        page_num += 1

ser.write(bytes([0x02]))
ans = ser.read(1)
if ans[0] == 0x01:
    print("OK")
else:
    print("ERROR: Firmware CRC is wrong!")

ser.close()






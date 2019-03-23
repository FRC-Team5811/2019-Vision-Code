import serial

var1 = 0.3
var2 = 65.22

with serial.Serial(port = "/dev/serial0", baudrate=115200) as ser:
    data = str(var1) + "|" + str(var2) + "|\n"
    ser.write(data)

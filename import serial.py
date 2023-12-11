import serial
import time
# Replace 'COM3' with the appropriate serial port for your device
port = '/dev/ttyUSB1'

# Define the baud rate and other serial parameters
baud_rate = 9600
timeout = 10

# Open the serial port
ser = serial.Serial(port, baud_rate, timeout=timeout)

try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()

        # Split the line into individual values
        values = line.split(',')
        tds_value = values[1]
        # Print the individual values
        print("Value 1:", values[0])
        print("Value 2:", values[1])
        time.sleep(1)
except KeyboardInterrupt:
    # Close the serial port when the program is interrupted
    ser.close()
    print("Serial port closed.")

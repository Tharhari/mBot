#!/usr/bin/env python3
import serial

'''
This program takes input from user, which
will later be replaced by reading from
the app by sending data over bluetooth

If the arduino sends something back we read
and print it to the Raspberry pi terminal
'''
if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
	ser.flush()

	while True:
		command = input()
		command = command.encode()
		# send to arduino
		ser.write(bytearray(command))
		if ser.in_waiting > 0:
			line = ser.readline().decode('utf-8').rstrip()
			print(line)

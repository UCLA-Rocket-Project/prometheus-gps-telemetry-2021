#!/usr/bin/env python3
'''
pySerial Demo

Provides a simple demo of reading from, decoding, and writing/storing data read
over a serial input connection using the `pySerial` API.
NOTE: This code is currently untested, but based on online source, should run
without issue.
'''

import serial
import time
import csv

# Configurable parameters
PORT = 'COM0' # Can be found by locating port name in Arduino IDE; Windows: 'COM.*', GNU/Linux: '/dev/ttyUSB.*'
BAUDRATE = 9600
OUTPUT_FILE = 'data.csv'


def main():

	# Establish the connection on a specific port (timeout=0 ensures non-blocking, so `read` returns immediately)
	ser = serial.Serial(PORT, BAUDRATE, timeout=0)

	# Open file for saving data read (mode='a' for writing/appending to end if file exists)
	with open(OUTPUT_FILE, mode='a') as f:

		writer = csv.writer(f, delimiter=",")
		
		while True:

			# Read any bytes from connection; returns `bytes` std object
			data = ser.read()
			decoded_data = data.decode('utf-8')

			# Check size before handling/storing (no need to write if we have no data)
			if len(decoded_data) == 0:
				continue

			# Store data read (`time` returns time since epoch using system clock)
			writer.writerow([time.time(), decoded_data])


if __name__ == '__main__':
	main()
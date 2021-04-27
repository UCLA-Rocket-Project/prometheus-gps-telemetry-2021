import matplotlib.pyplot as plt
from time import sleep
import serial
from serial import Serial

#ser = serial.Serial(port='/dev/cu.usbmodem144301', baudrate=9600)

voltage = []
pressure = []
i = 0

#This code goes through 5 iterations, then creates a graph. Takes in random data from a plain arduino uno, just plugged into a computer, and reads lines in. Plots that dummy data on x axis, and 1s on y axis.
ser = serial.Serial("/dev/cu.usbmodem144301", 9600)
while (i < 5):
     cc=str(ser.readline())
     voltage += [cc[2:][:-5]]
     pressure += [1]
     print("I am working")
     plt.plot(voltage, pressure, linewidth = 3,
         marker='o', markerfacecolor='black', markersize=8)
     i += 1
     
   #  print(cc[2:][:-5])

#plt.plot(voltage, pressure, linewidth = 3,
        # marker='o', markerfacecolor='black', markersize=8)
  
# X axis
plt.xlabel('Voltage')

# Y axis
plt.ylabel('Pressure')
  
plt.title('Voltage v. Pressure test from ARES PT')
  
plt.show()

# TO DO:
# What should the terminating condition be for the while loop? How can I get it to update the graph on live feed in front of us?
# Data backups: how can we save the data in front of us?

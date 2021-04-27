import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import serial
from serial import Serial

# The code below creates a live plot that plots in front of us.
# Adjust as needed: x and y axes (voltage up to 600, ec)
                  # pressure data
                  # currently, voltage = dummy data from arduino

voltage = []
pressure = []
##i = 0

plt.axis([0, 20, 0, 10])

ser = serial.Serial("/dev/cu.usbmodem141301", 9600)
while True:
    cc=str(ser.readline())
    voltage += [cc[2:][:-5]]
    pressure += [np.random.random()]
    plt.plot(voltage, pressure, linewidth = 1,
         marker='o', markerfacecolor='black', markersize=2)
    plt.pause(0.05)

plt.show()


# The code below creates a stagnant plot that only shows up after the arduino is done collecting data.

##ser = serial.Serial("/dev/cu.usbmodem141301", 9600)
##while (i < 5):
##     cc=str(ser.readline())
##     voltage += [cc[2:][:-5]]
##     pressure += [1]
##     print("I am working")
##     plt.plot(voltage, pressure, linewidth = 3,
##         marker='o', markerfacecolor='black', markersize=8)
##     i += 1
##    # plt.show()
##     
##   #  print(cc[2:][:-5])
##
###plt.plot(voltage, pressure, linewidth = 3,
##        # marker='o', markerfacecolor='black', markersize=8)
##  
### X axis
##plt.xlabel('Voltage')
##
### Y axis
##plt.ylabel('Pressure')
##  
##plt.title('Voltage v. Pressure test from ARES PT')
##  
##plt.show()


#terminating condition for while loop
#update plot as we go (live)

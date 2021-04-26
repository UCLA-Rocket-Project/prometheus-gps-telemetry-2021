import matplotlib.pyplot as plt
from time import sleep
import serial


pressure = [];
ser = serial.Serial('(WHAT SERIAL PORT):/dev/tty.usbmodem1d11', 9600) # Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish
while True:
     counter +=1
     currPress = str(chr(counter));
     pressure += currPress;
     #ser.write(str(chr(counter))) # Convert the decimal number to ASCII then send it to the Arduino
     #print(ser.readline()) # Read the newest output from the Arduino
     sleep(.1) # Delay for one tenth of a second
     if counter == 255:
         counter = 32
  
#voltage = [0.98, 1.15,1.29, 1.51,1.65,1.79,1.98,2.12,2.28,2.45,2.6,2.76,2.93,3.04,3.22,3.34,3.58,3.73,3.9,4.04,4.2,4.37,4.54,4.7,4.92 ]

#pressure = [0,40,80,120,160,200,240,280,320,360,400,440,480,520,560,600,640,680,720,760,800,840,880,920,980]
 
plt.plot(voltage, pressure, linewidth = 3,
         marker='o', markerfacecolor='black', markersize=8)
  
# X axis
plt.xlabel('Voltage')

# Y axis
plt.ylabel('Pressure')
  
plt.title('Voltage v. Pressure test from ARES PT')
  
plt.show()


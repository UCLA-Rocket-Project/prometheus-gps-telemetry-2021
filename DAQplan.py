#!/usr/bin/env python3

import pandas as pd
import matplotlib as mat
import serial as ser

##DATA ACQUISITION:
##    GPS coordinates
##        - list view, last coordinate highlighted within several degrees of accuracy. continuous measurement; program stops collection after GPS coordinates have been stable for a certain period of time.
##    Accelerometer
##        - versus time
##    Altimeter (BMP390L)
##    Temperature (IMU)
##    Angular velocity (IMU)
##    Barometric pressure (BMP390L)


## pandas initial test
## a really quick plot of velocity versus time from testData.csv

data = pd.read_csv("testData.csv") ## note: pd.read_csv is how we'll read csv files

t = 0

while (t != 538):    
    data = {'Velocity': data,
            'Time': t}
    t += 1

df = pd.DataFrame(data,columns=['Velocity','Time'])

print (df)

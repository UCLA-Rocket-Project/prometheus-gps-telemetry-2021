UNZIP libraries.zip AND PUT THE FILES FROM THE LIBRARIES FOLDER INTO YOUR ARDUINO LIBRARIES FOLDER!!!

^ now read this again to make sure you do it

okay now on to the rest

I wrote all of this in Arduino 1.8.10, but any version should theoretically work.

The GPS_Tele folder has a sketch that goes on the rocket. The Ground_Data one goes on the ground, connected to the Yagi and to a computer. The code is commented, mostly. DM me on slack if you have any issues.

TODO:
Main issue: memory issues. Not enough memory on Feather 32u4 for all the stuff we want to do.
One solution is to spend $40 and buy a Feather M0 instead. We probably aren't gonna do that, so an alternative solution is to spend a few hours ripping out the accelerometer code and library (it takes up 20% of the available memory) and replace it with altimeter and shock accelerometer code. It should work, hopefully.

Of course we could also just invest $40.

Anyway, more notes from troubleshooting and random stuff from assembly below:











about 500-600ms per message sent

sending not gyro: about 100ms better

things to mention
- through-hole antenna mounting option
- how much data do we send
    - and how long it takes
    - most we can expect is 2/s over radio, probably less, possibly more…
- long-range testing plan
- the radio guys sent us the wrong adapter lol


PIN 9 (black) to FONA TX
PIN 11 (red) to FONA RX
PIN 10 (red) to FONA Reset

4,7,8 used for radio

MEMORY TIME
Currently at 124% dropping sprintf cuts 8%
connect() is 5%
watchdog is basically nothing
LM6DSOX uses 20 percent, how the fuck
all of radio uses 16%
	that 120char buffer uses like nothing
SoftwareSerial uses 4%
Fona uses 0
MQTT uses 12%
openlog uses 1%
the MQTT/Fona vars use 11%

THE PLAN
drop sprintf
find an alternative to the LSM lib or cut the dead weight
cut dead weight from radio
remove every serial print
remove the time variable, just use millis()

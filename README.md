# Prometheus GPS/Telemetry 2020-2021
Software for Prometheus' GPS/Telemetry Electronics for the 2020-2021 season. 

Some objectives/the basic plan: 
 
- Save and display all data in real time via transmittion from LoRa module. Would be first prometheus mission to recover data in recent years

- Flask (basic) and ReactJS (prettier) on the front end for the GUI

- Pandas and matplotlib for the data representation and backend analysis

- Importing data as structs/using the pySerial library to read in serial input

- Using SQLite as database

- Radio stuff: 10s of bytes per second. Compromise between range and data rate.

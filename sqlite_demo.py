#!/usr/bin/env python3

import sqlite3

# Connect to database
conn = sqlite3.connect('tmp.db')
print("Opened database successfully")

c = conn.cursor()

# Create table
try:
    c.execute('''CREATE TABLE stocks
                 (date text, trans text, symbol text, qty real, price real)''')
    print("Table `stocks` created successfully")

except sqlite3.OperationalError:
    print("'stocks' table already created! Skipping...")

# Insert a row of data
c.execute("INSERT INTO stocks VALUES ('2006-01-05','BUY','RHAT',100,35.14)")
print("Inserted row of data into table `stocks`.")

# Save (commit) the changes
conn.commit()
print("Committed changes to db.")

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()
print("Closed connection to db.")
#!/usr/bin/python
# sudo pigpiod

import pigpio
import time

pi = None
slave_addr = 0x77

def handler(v, w):
    global pi
    global slave_addr
    status, nbytes, data = pi.bsc_i2c(slave_addr)

    if nbytes > 0:
        print("data=", data.decode('ascii', errors='replace'), " (", str(nbytes), " bytes) status=", str(status), sep='')

pi = pigpio.pi()
pi.event_callback(pigpio.EVENT_BSC, handler)
pi.bsc_i2c(slave_addr)

while(True):
    time.sleep(10)
    print("Waiting...")
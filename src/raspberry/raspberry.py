#!/usr/bin/python
# sudo pigpiod

import pigpio

pi = None
slave_addr = 0x77

def handler():
   global pi
   global slave_addr
   status, nbytes, data = pi.bsc_i2c(slave_addr) 

   if nbytes > 0:
      print("data=", data, "(", nbytes, "bytes) status=", status)

pi = pigpio.pi()
pi.event_callback(pigpio.EVENT_BSC, handler)
pi.bsc_i2c(slave_addr)
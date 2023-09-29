from usbd import CDC, get_usbdevice
import os
import time

cdc = CDC()  # adds itself automatically
cdc.init(timeout=0)  # zero timeout makes this non-blocking, suitable for os.dupterm()

print("Triggering reenumerate...")

ud = get_usbdevice()
ud.reenumerate()

print('Waiting for CDC port to open...')

# cdc.is_open() returns true after enumeration finishes.
# cdc.dtr is not set until the host opens the port and asserts DTR
while not (cdc.is_open() and cdc.dtr):
    time.sleep_ms(20)

print('CDC port is open, duplicating REPL...')

old_term = os.dupterm(cdc)

print('Welcome to REPL on CDC implemented in Python?')

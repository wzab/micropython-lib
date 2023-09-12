from usbd import CDC, get_usbdevice
import time

cdc = CDC()  # adds itself automatically
cdc.init(txbuf=32, rxbuf=32)

print('trigger reenumerate')

ud = get_usbdevice()
ud.reenumerate()

while not cdc.is_open():
    time.sleep_ms(100)

print(cdc)

# sending something over CDC
while True:
    print(cdc)
    print('writing...')
    cdc.write(b'Hello World')
    print('reading...')
    # receiving something..
    print(cdc.read(1))


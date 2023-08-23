from usbd import CDC, get_usbdevice
import time

cdc = CDC()  # adds itself automatically
cdc.init(txbuf=32, rxbuf=32)

print('trigger reenumerate')

ud = get_usbdevice()
ud.reenumerate()

while not cdc.is_open():
    time.sleep_ms(100)

# sending something over CDC
print('writing...')
cdc.write(b'Hello World')
print('write done')
print('reading...')
# receiving something..
print(cdc.read(10))
print('done')


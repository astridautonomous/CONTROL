import smbus

import time
 
bus = smbus.SMBus(1) # 1 indicates /dev/i2c-1

device_address = 0x04 # Change according to your STM32 I2C address
 
try:

    bus.write_byte(device_address, 0xFF)  # Sending a byte

    print("Byte sent")

except Exception as e:

    print(f"Failed to send data: {e}")

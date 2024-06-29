import smbus2
import time
import crc8

bus = smbus2.SMBus(7)  # 7 indicates /dev/i2c-7
time.sleep(1)
address = 0x12

def calculate_crc(data):
    hash = crc8.crc8()
    hash.update(data)
    return hash.digest()[0]

def data_encryption(data):
    return (data + 30)*256/60

def data_decryption(data):
    return (data*60/256) - 30

def write_number(value):
    try:
        encrypted_value = data_encryption(value)
        crc_value = calculate_crc(bytes([encrypted_value]))
        bus.write_i2c_block_data(address, 0, [encrypted_value, crc_value])
        print('Sent data:', value, '-> Encrypted:', encrypted_value, 'CRC:', crc_value)
        return 0
    except Exception as e:
        print("Error in write_number:", e)
        return -1

def read_number():
    try:
        data = bus.read_i2c_block_data(address, 0, 2)
        encrypted_value, received_crc = data[0], data[1]
        calculated_crc = calculate_crc(bytes([encrypted_value]))
        if calculated_crc != received_crc:
            print("CRC mismatch! Received:", received_crc, "Calculated:", calculated_crc)
            return None
        decrypted_value = data_decryption(encrypted_value)
        return decrypted_value
    except Exception as e:
        print("Error in read_number:", e)
        return None

# Test loop
while 1:
    value_to_send = 15
    write_number(value_to_send)

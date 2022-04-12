from email import header
import struct

def calc_crc16(string):
    # data = bytearray.fromhex(string)
    data = string
    crc = 0xFFFF
    for pos in data:
            crc ^= pos
            for i in range(8):
                    if((crc & 1) != 0):
                            crc >>= 1
                            crc ^= 0xA001
                    else:
                            crc >>= 1

    return ((crc & 0xff) << 8) + (crc >> 8)

def checkCrc16(list_data, check_data):
    return calc_crc16(list_data) == (check_data[0]<<8)+check_data[1]

def hex_to_short(raw_data):
    return list(struct.unpack("hhh", bytearray(raw_data)))

if __name__ == "__main__":
    flag = checkCrc16([0x50,0x03,0x18,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x15,0xFA,0xB1],[0xdd,0x64])
    print(flag)
    print(hex_to_short([0x00,0x00,0x00,0x00,0x00,0x08]))
    a = [0x00,0x00,0x00,0x00,0x00,0x08]
    print(a[7:0:-1])
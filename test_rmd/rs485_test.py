import serial

def gen_crc16(data):
    CRC16 = 0x8005
    out = 0
    bits_read = 0

    # Iterate over each byte in the data
    for byte in data:
        for _ in range(8):  # Process each bit in the byte
            bit_flag = out >> 15
            out <<= 1
            out |= (byte >> bits_read) & 1  # Work from the least significant bits

            bits_read += 1
            if bits_read > 7:
                bits_read = 0

            # Cycle check
            if bit_flag:
                out ^= CRC16

    # "Push out" the last 16 bits
    for _ in range(16):
        bit_flag = out >> 15
        out <<= 1
        if bit_flag:
            out ^= CRC16

    # Reverse the bits
    crc = 0
    i = 0x8000
    j = 0x0001
    while i != 0:
        if i & out:
            crc |= j
        i >>= 1
        j <<= 1

    return crc

# Convert the CRC to low and high bytes
def crc16_low_high(data):
    crc = gen_crc16(data)
    crc16_low = crc & 0xFF
    crc16_high = (crc >> 8) & 0xFF
    return crc16_low, crc16_high

# Function to create the complete message frame
def create_message(data):
    crc16_low, crc16_high = crc16_low_high(data)
    return data + [crc16_low, crc16_high]

# Set up the serial connection for RS485
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Replace with your actual port name (e.g., 'COM3' on Windows)
    baudrate=115200,      # Set the correct baud rate for your motor
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

# Message data (example)
data = [0x3E, 0x01, 0x08, 0x60, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00]
full_message = create_message(data)

# Send the message
ser.write(bytearray(full_message))
print(f"Sent: {[f'{byte:02X}' for byte in full_message]}")

# Read the response from the motor (if applicable)
response = ser.read(20)  # Adjust the number of bytes as needed
print(f"Received: {[f'{byte:02X}' for byte in response]}")

# Close the serial connection
ser.close()

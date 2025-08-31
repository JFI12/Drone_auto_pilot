from machine import I2C, Pin

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)  # Adjust GPIOs if necessary

QMC5883_ADDRESS = 0x0D

# Try reading a known register, like the status or data registers
try:
    data = i2c.readfrom_mem(QMC5883_ADDRESS, 0x00, 1)  # Replace 0x00 with the actual register address
    print("Data:", data)
except OSError as e:
    print("Error:", e)
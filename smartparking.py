from machine import Pin, PWM, I2C, SPI
from time import sleep
from lcd import I2C_LCD
import time
#from mfrc522 import MFRC522

# Define pin numbers
IR1_pin = 2
IR2_pin = 4
servo_pin = 5  # Change this to the appropriate pin
# RST_PIN = 25    # Change this to the appropriate pin
# SCL_PIN = 26    # Change this to the appropriate pin

# Initialize I2C for LCD
i2c = I2C(0, sda=Pin(21), scl=Pin(22))
lcd = I2C_LCD(i2c, 0x27, 2, 16)  # Change the address if necessary

# Initialize Servo
servo_pwm = PWM(Pin(servo_pin), freq=50)
servo_pwm.duty(0)
# Initialize IR sensors
IR1 = Pin(IR1_pin, Pin.IN)
IR2 = Pin(IR2_pin, Pin.IN)

# Initialize RFID reader
# spi = SPI(1, baudrate=1000000, sck=Pin(SCL_PIN), mosi=Pin(23), miso=Pin(19))
# mfrc = MFRC522(spi, Pin(RST_PIN), Pin(22))

# Initialize variables
Slot = 5
flag1 = 0
flag2 = 0

def servo():
    servo_pwm.duty(115)  # Adjust the duty cycle for your servo specifications
    time.sleep(1)

    # Rotate 90 degrees counterclockwise (returning to the initial position)
    servo_pwm.duty(77)  # Adjust the duty cycle for your servo specifications
    time.sleep(1)

def setup():
    lcd.clear()
    lcd.putstr("MicroPython")
    lcd.move_to(0, 1)
    lcd.putstr(" Car Parking System  ")
    sleep(2)
    lcd.clear()

    # Set servo to the initial position

# def rfid_scan():
#     (stat, tag_type) = mfrc.request(mfrc.REQIDL)
#     if stat == mfrc.OK:
#         (stat, raw_uid) = mfrc.anticoll()
#         if stat == mfrc.OK:
#             uid = raw_uid[0] << 24 | raw_uid[1] << 16 | raw_uid[2] << 8 | raw_uid[3]
#             return uid
#     return None

def loop():
    global Slot, flag1, flag2

    # RFID scan
#     uid = rfid_scan()
# 
#     if uid is not None:
#         lcd.clear()
#         lcd.move_to(0, 0)
#         lcd.print(" RFID Detected   ")
#         lcd.move_to(0, 1)
#         lcd.print("UID: {:X}".format(uid))
#         sleep(2)
#         lcd.clear()

    if IR1.value() == 0 :
        if Slot > 0:
            flag1 = 1
            if flag2 == 0:
                servo()
                Slot -= 1
        else:
            lcd.clear()
            lcd.move_to(0, 0)
            lcd.putstr("    SORRY :    ")
            lcd.move_to(0, 1)
            lcd.putstr("  Parking Full  ")
            sleep(3)
            lcd.clear()

    if IR2.value() == 0 :
        flag2 = 1
        if flag1 == 0 :
            
            servo()
            print("Outing the vehicle")
            Slot += 1

    if flag1 == 1 and flag2 == 1:
        sleep(1)
        servo()
        flag1 = 0
        flag2 = 0

    lcd.clear()
    lcd.move_to(0, 0)
    lcd.putstr("WELCOME!")
    lcd.move_to(0, 1)
    lcd.putstr(f"Slot Left:{Slot}")
    sleep(1)

# Run the setup function once
setup()

# Run the loop function continuously
while True:
    loop()
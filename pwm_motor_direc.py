from machine import I2C, ADC, PWM, Pin
import time

from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd

I2C_ADDR     = 0x27
I2C_NUM_ROWS = 4
I2C_NUM_COLS = 20

i2c = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

adc = ADC(Pin(26))

pin_switch = Pin(16, Pin.IN)
pin_forward = Pin(6, Pin.OUT)
pin_reverse = Pin(7, Pin.OUT)
pwm_signal = PWM(Pin(8))
pwm_signal.freq(1000)

def trimmer():
    return adc.read_u16()

def main_lcd():
    lcd.clear()
    lcd.putstr(str(trimmer()))
   
def motor_start():
    print("Start")
    duty_cycle = round(trimmer())
    pwm_signal.duty_u16(duty_cycle)
    pin_forward.high()
    pin_reverse.low()
    
def motor_stop():
    print("Stop")
    pin_forward.low()
    pin_reverse.low()
    
init_state = 0
curr_state = 0
while True:
    main_lcd()
    time.sleep(1)
    curr_state = pin_switch.value()
    if curr_state != init_state:
        print("switch ON")
        motor_start()
    else:
        print("Switch OFF")
        motor_stop()
    pass
   # if pin_switch.value() == 1:
    #    motor_start()
     #else:
       # motor_stop()
        

        


    
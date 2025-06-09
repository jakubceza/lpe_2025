from machine import I2C, ADC, PWM, Pin
import time
import asyncio

from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd

I2C_ADDR     = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 20

i2c = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

adc0 = ADC(Pin(26))
adc1 = ADC(Pin(27)) 

pin_switch = Pin(16, Pin.IN)
pin_forward = Pin(6, Pin.OUT)
pin_reverse = Pin(7, Pin.OUT)
pwm_signal = PWM(Pin(8))
pwm_signal.freq(1000)

def trimmer():
    raw_value = adc0.read_u16()
    t_voltage = raw_value * (3.3/65535)
    return round(t_voltage, 2)

def magnet_sensor():
    field_prox = adc1.read_u16()
    m_voltage = field_prox * (3.3/65535)
    m_voltage = m_voltage * 1000
    return round(m_voltage)

count = 0
magnet_state = False

def magnet_counter():
    global count, magnet_state
    
    sensor_value = magnet_sensor()
    
    if sensor_value < 1500:
        # Magnet detected
        magnet_state = True
        return count
    else:
        # No magnet - count when magnet leaves sensor
        if magnet_state:  # Only count when magnet changes from present to absent
            magnet_state = False
            count += 1
            return count
    return count
        
def main_lcd():
    lcd.clear()
    lcd.move_to(0,0)
    lcd.putstr("TRIMMER: " + str(trimmer()) + " V")
    lcd.move_to(0,1)
    lcd.putstr(str(magnet_sensor()) + " mV")
    lcd.move_to(8,1)
    lcd.putstr(str(magnet_counter()))
    await asyncio.sleep(0.4)

def motor_switch():
    init_state = 0
    curr_state = pin_switch.value()
    if curr_state != init_state:
        print("switch ON")
        motor_start()
    else:
        print("Switch OFF")
        motor_stop()
   
def motor_start():
    print("Start")
    duty_cycle = round(adc0.read_u16())
    pwm_signal.duty_u16(duty_cycle)
    pin_forward.high()
    pin_reverse.low()
    
def motor_stop():
    print("Stop")
    pin_forward.low()
    pin_reverse.low()
    

while True:
    asyncio.run(main_lcd())
    motor_switch()
    pass
        

        


    
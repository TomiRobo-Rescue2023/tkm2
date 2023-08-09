#!/usr/bin/env python3

import time
import math
import motoron
import busio
import board
import RPi.GPIO as GPIO 
import neopixel_spi as neopixel
import adafruit_amg88xx
import matplotlib.pyplot as plt
import numpy as np
from gpiozero import RotaryEncoder
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

mc = motoron.MotoronI2C()
#ポート番号の定義
Led_pin = 25                        #変数"Led_pin"に18を格納

#GPIOの設定
GPIO.setmode(GPIO.BCM)              #GPIOのモードを"GPIO.BCM"に設定
GPIO.setup(Led_pin, GPIO.OUT)       #GPIO18を出力モードに設定

PIN = 12
FREQ = 50

GPIO.setup(PIN, GPIO.OUT)
servo = GPIO.PWM(PIN, FREQ)

# I2Cバスの初期化
i2c_bus = busio.I2C(board.SCL, board.SDA)

# センサーの初期化
# アドレスを68に指定
sensor = adafruit_amg88xx.AMG88XX(i2c_bus, addr=0x68)

# センサーの初期化待ち
time.sleep(.1)

tr2 = 0
group = 0

# Reset the controller to its default settings, then disable CRC.  The bytes for
# each of these commands are shown here in case you want to implement them on
# your own without using the library.
mc.reinitialize()  # Bytes: 0x96 0x74
mc.disable_crc()   # Bytes: 0x8B 0x04 0x7B 0x43

# Clear the reset flag, which is set after the controller reinitializes and
# counts as an error.
mc.clear_reset_flag()  # Bytes: 0xA9 0x00 0x04

# Configure motor 1
mc.set_max_acceleration(1, 200)
mc.set_max_deceleration(1, 200)

# Configure motor 2
mc.set_max_acceleration(2, 200)
mc.set_max_deceleration(2, 200)

# Assigning parameter values
ppr = 800  # Pulses Per Revolution of the encoder
tstop = 20  # Loop execution duration (s)
tsample = 0.01  # Sampling period for code execution (s)
tdisp = 0.5  # Sampling period for values display (s)

# Creating encoder object using GPIO pins
encoder1 = RotaryEncoder(6, 5, max_steps=0)
encoder2 = RotaryEncoder(26, 16, max_steps=0)

# Initializing previous values and starting main clock
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
spi = board.SPI()

# The number of NeoPixels
NUM_PIXELS = 10

# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
PIXEL_ORDER = neopixel.GRB

pixels = neopixel.NeoPixel_SPI(
    spi, NUM_PIXELS, brightness=0.8, auto_write=False, pixel_order=PIXEL_ORDER, bit0=0b10000000
)
#init
servo.start(8.0)

def wheel(pos):
      # Input a value 0 to 255 to get a color value.
      # The colours are a transition r - g - b - back to r.
      if pos < 0 or pos > 255:
         r = g = b = 0
      elif pos < 85:
         r = int(pos * 3)
         g = int(255 - pos * 3)
         b = 0
      elif pos < 170:
         pos -= 85
         r = int(255 - pos * 3)
         g = 0
         b = int(pos * 3)
      else:
         pos -= 170
         r = 0
         g = int(pos * 3)
         b = int(255 - pos * 3)
      return (r, g, b) if PIXEL_ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)

def rainbow_cycle(wait):
   for j in range(255):
      for i in range(NUM_PIXELS):
         pixel_index = (i * 256 // NUM_PIXELS) + j
         pixels[i] = wheel(pixel_index & 255)
      pixels.show()
      time.sleep(wait)

class JoySubscriber(Node):
   def __init__(self):
        super().__init__("joy_subscriber")
        self.subscription_joy = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.subscription_joy

        self.joy_msgs = Joy()
    
   def joy_callback(self, joy_msg):
      self.joy_msgs = joy_msg

      global tr2,group,tprev

      # Pausing for `tsample` to give CPU time to process encoder signal
      time.sleep(tsample)
      # Getting current time (s)
      tcurr = time.perf_counter() - tstart
      # Getting angular position of the encoder
      # roughly every `tsample` seconds (deg.)
      anglecurr1 = 360 / ppr * encoder1.steps
      anglecurr2 = 360 / ppr * encoder2.steps
      distance1=anglecurr1 / 360 * 2 * math.pi * 72 / 1000
      distance2=anglecurr2 / 360 * 2 * math.pi * 72 / 1000
      # Printing angular position every `tdisp` seconds
      if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
         print("Dis_right = {:0.03f} deg Angle_right = {:0.03f} deg".format(distance1,anglecurr1))
      # Updating previous values
      tprev = tcurr

      if(joy_msg.buttons[0]==1):
         pixels.fill((0, 0, 0))
         pixels.show()
         tr2=0
         group=0
      
      if(joy_msg.buttons[1]==1):
         pixels.fill((0, 0, 255))
         pixels.show()
         tr2=1
         group=0

      if(joy_msg.buttons[2]==1):
         pixels.fill((0, 0, 0))
         pixels.show()
         tr2=0
         group=0
         
      if(joy_msg.buttons[3]==1):
         pixels.fill((255, 255, 255))
         pixels.show()
         tr2=0
         group=1
      
      if(joy_msg.buttons[9]==1):
         pixels.fill((0, 255, 255))
         pixels.show()
         while(distance1>-0.900):
            if(joy_msg.buttons[1]==1 or joy_msg.buttons[3]==1):
               distance1 = -2.0
            # Pausing for `tsample` to give CPU time to process encoder signal
            time.sleep(tsample)
            # Getting current time (s)
            tcurr = time.perf_counter() - tstart
            # Getting angular position of the encoder
            # roughly every `tsample` seconds (deg.)
            anglecurr1 = 360 / ppr * encoder1.steps
            anglecurr2 = 360 / ppr * encoder2.steps
            distance1=anglecurr1 / 360 * 2 * math.pi * 72 / 1000
            distance2=anglecurr2 / 360 * 2 * math.pi * 72 / 1000
            # Printing angular position every `tdisp` seconds
            if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
               print("Dis_right = {:0.03f} deg Angle_right = {:0.03f} deg".format(distance1,anglecurr1))
            # Updating previous values
            tprev = tcurr
            mc.set_speed(1, 100)
            mc.set_speed(2, -100)
            time.sleep(0.001)


         
         while(distance1>-0.912):
            if(joy_msg.buttons[1]==1 or joy_msg.buttons[3]==1):
               distance1 = -2.0
            # Pausing for `tsample` to give CPU time to process encoder signal
            time.sleep(tsample)
            # Getting current time (s)
            tcurr = time.perf_counter() - tstart
            # Getting angular position of the encoder
            # roughly every `tsample` seconds (deg.)
            anglecurr1 = 360 / ppr * encoder1.steps
            anglecurr2 = 360 / ppr * encoder2.steps
            distance1=anglecurr1 / 360 * 2 * math.pi * 72 / 1000
            distance2=anglecurr2 / 360 * 2 * math.pi * 72 / 1000
            # Printing angular position every `tdisp` seconds
            if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
               print("Dis_right = {:0.03f} deg Angle_right = {:0.03f} deg".format(distance1,anglecurr1))
            # Updating previous values
            tprev = tcurr
            mc.set_speed(1, -100)
            mc.set_speed(2, -100)
            time.sleep(0.001)

         while(distance1>-1.812):
            if(joy_msg.buttons[1]==1 or joy_msg.buttons[3]==1):
               distance1 = -2.0
            # Pausing for `tsample` to give CPU time to process encoder signal
            time.sleep(tsample)
            # Getting current time (s)
            tcurr = time.perf_counter() - tstart
            # Getting angular position of the encoder
            # roughly every `tsample` seconds (deg.)
            anglecurr1 = 360 / ppr * encoder1.steps
            anglecurr2 = 360 / ppr * encoder2.steps
            distance1=anglecurr1 / 360 * 2 * math.pi * 72 / 1000
            distance2=anglecurr2 / 360 * 2 * math.pi * 72 / 1000
            # Printing angular position every `tdisp` seconds
            if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
               print("Dis_right = {:0.03f} deg Angle_right = {:0.03f} deg".format(distance1,anglecurr1))
            # Updating previous values
            tprev = tcurr
            mc.set_speed(1, 100)
            mc.set_speed(2, -100)
            time.sleep(0.001)

      if(joy_msg.axes[4] > 0):
         servo.ChangeDutyCycle(8.0)
      if(joy_msg.axes[4] < 0):
         servo.ChangeDutyCycle(6.0)
      
      if(joy_msg.axes[5] > 0):
         GPIO.output(Led_pin, GPIO.HIGH)
      elif(joy_msg.axes[5]== 0):
         GPIO.output(Led_pin, GPIO.LOW)
      elif(joy_msg.axes[5] < 0):
         # データ取得
         sensordata = sensor.pixels
         # bicubic補間したデータ
         fig = plt.imshow(sensordata, cmap="inferno", interpolation="bicubic")
         plt.colorbar()
         plt.show()
      
      if(tr2==1 or group==1):
         if(tr2==1):
            pixels.fill((0, 0, 255))
            pixels.show()
         elif(group==1):
            pixels.fill((255,255,255))
            pixels.show()
         time.sleep(0.001)
         right_joy =-joy_msg.axes[3]
         left_joy =joy_msg.axes[0]
       
         move_duty=int(right_joy*1000)
         rotate_duty=int(left_joy*1000)
       
         if(joy_msg.axes[3]!=-0):
            mc.set_speed(1, -move_duty)
            mc.set_speed(2, move_duty)
            time.sleep(0.001)

         if(joy_msg.axes[0]!=-0):
            mc.set_speed(1, -rotate_duty)
            mc.set_speed(2, -rotate_duty)
            time.sleep(0.001)
       
         if(joy_msg.axes[0]==-0 and joy_msg.axes[3]==-0):
            mc.set_speed(1, 0)
            mc.set_speed(2, 0)
            time.sleep(0.001)

   def timer_callback(self):
      if (self.joy_msgs[] == 1)
      

def main(args=None):
    pixels.fill((0, 0, 0))
    pixels.show()
    rclpy.init(args=args)
    sub = JoySubscriber()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


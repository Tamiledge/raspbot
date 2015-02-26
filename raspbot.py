#! /usr/bin/python

# A Python command line tool for a badge leash robot
# By Greg Griffes http://yottametric.com
# GNU GPL V3 

# !!!!!!!!!!!!!!!!!
# remember to run this as root "sudo ./raspbot" so that DMA can be used for the servo
# !!!!!!!!!!!!!!!!!

# Jan 2015

import smbus
import sys
import getopt
import time 
import pigpio
import time
from webcolors import *
import pygame
from pygame.locals import *
import random
from omron_src import *		# contains omron functions

# Constants
RASPI_I2C_CHANNEL=1		# the /dev/i2c device
OMRON_1=0x0a 			# 7 bit I2C address of Omron MEMS Temp Sensor D6T-44L
OMRON_BUFFER_LENGTH=35		# Omron data buffer size
OMRON_DATA_LIST=16		# Omron data array - sixteen 16 bit words
HORIZ_LIST_SIZE=7		# The 4x4 array converted to 1x7 for temp tracking
MAX_VOLUME=1.0			# maximum speaker volume factor for pygame.mixer
TEMPMARGIN=3			# number of degrees F greater than room temp to detect a person
DEGREE_UNIT='F'			# F = Farenheit, C=Celcius
MEASUREMENT_WAIT_PERIOD=0.3     # time between Omron measurements
SERVO=1				# set this to 1 if the servo motor is wired up
SERVO_GPIO_PIN = 11		# GPIO number 
DEBUG=0				# set this to 1 to see debug messages on monitor
SCREEN_DIMENSIONS = [400, 400]	# setup the IR color window
MIN_TEMP = 0			# minimum expected temperature in Fahrenheit
MAX_TEMP = 200			# minimum expected temperature in Fahrenheit

# Colors


import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_GPIO_PIN,GPIO.OUT)
from RPIO import PWM		# for the servo motor

#hello_audio = "snd/20150201_zoe-hello1.mp3", "snd/20150201_zoe-hello2.mp3", "snd/20150201_chloe-higgg.mp3"
hello_audio = "snd/20150201_zoe-hello1.mp3", "snd/20150201_zoe-hello2.mp3"
after_hello_audio = "snd/20150201_zoe-boeing.mp3", "snd/20150201_chloe-boeing.mp3",  "snd/20150201_chloe-whosthat.mp3", "snd/20150201_chloe-yippee1.mp3"
#after_hello_audio = "snd/20150201_zoe-giggle1.mp3", "snd/20150201_zoe-boeing.mp3", "snd/20150201_zoe-candy1.mp3", "snd/20150201_zoe-dontworry1.mp3", "snd/20150201_chloe-boeing.mp3", "snd/20150201_chloe-candy1.mp3", "snd/20150201_chloe-dontworry1.mp3", "snd/20150201_chloe-dontworry2.mp3", "snd/20150201_chloe-whosthat.mp3", "snd/20150201_chloe-itslooking.mp3", "snd/20150201_chloe-yippee1.mp3"
byebye_audio = "snd/20150201_zoe-goodbye1.mp3", "snd/20150201_chloe-goodbye1.mp3"
after_byebye_audio = "snd/20150201_chloe-cry1.mp3", "snd/20150201_chloe-loveu.mp3", "snd/20150201_zoe-loveu.mp3"

# function to get the average value of a list
def avg(l):
    return sum(l, 0.0) / len(l)

def play_sound(volume, message):
   pygame.mixer.music.set_volume(volume)         
   pygame.mixer.music.load(message)
   pygame.mixer.music.play()
   while pygame.mixer.music.get_busy() == True:
      continue

#def set_servo(pi, servo_GPIO_pin, direction, delay_time):  
#   pw = direction + (servo_GPIO_pin*50)
#   if (pw != 0 and (pw < 500 or pw >2500)):
#      print 'Servo pulse width must be either 0 or between 500 and 2500: pw='+str(pw)
#      pw = 0
#   pi.set_servo_pulsewidth(servo_GPIO_pin, pw)
#   time.sleep(delay_time)
#   pi.set_servo_pulsewidth(servo_GPIO_pin, 0);	# set motor off

def print_temps(temp_list):
# Display each element's temperature in F
   print "%.1f"%temp_list[0]+' ',
   print "%.1f"%temp_list[1]+' ',
   print "%.1f"%temp_list[2]+' ',
   print "%.1f"%temp_list[3]+' ',
   print ''
   print "%.1f"%temp_list[4]+' ',
   print "%.1f"%temp_list[5]+' ',
   print "%.1f"%temp_list[6]+' ',
   print "%.1f"%temp_list[7]+' ',
   print ''
   print "%.1f"%temp_list[8]+' ',
   print "%.1f"%temp_list[9]+' ',
   print "%.1f"%temp_list[10]+' ',
   print "%.1f"%temp_list[11]+' ',
   print ''
   print "%.1f"%temp_list[12]+' ',
   print "%.1f"%temp_list[13]+' ',
   print "%.1f"%temp_list[14]+' ',
   print "%.1f"%temp_list[15]+' ',
   print ''

# Make a 1x7 horizontal array of temps based on a 4x4 array
# for use in head tracking of high temps
def make_horizontal(temp_list):
#   if DEBUG:
#      print 'Making 1x8 array'

   tlist=[0.0]*HORIZ_LIST_SIZE

   tlist[0] = max(temp_list[0], temp_list[4], temp_list[8], temp_list[12])
   tlist[2] = max(temp_list[1], temp_list[5], temp_list[9], temp_list[13])
   tlist[4] = max(temp_list[2], temp_list[6], temp_list[10], temp_list[14])
   tlist[6] = max(temp_list[3], temp_list[7], temp_list[11], temp_list[15])

   short_list=[tlist[0],tlist[2]]
   tlist[1] = avg(short_list)
   short_list=[tlist[2],tlist[4]]
   tlist[3] = avg(short_list)
   short_list=[tlist[4],tlist[6]]
   tlist[5] = avg(short_list)

#   if DEBUG:
#      print 'Horizontal Temperature string'
#      print "%.1f"%tlist[0]+' ',
#      print "%.1f"%tlist[1]+' ',
#      print "%.1f"%tlist[2]+' ',
#      print "%.1f"%tlist[3]+' ',
#      print "%.1f"%tlist[4]+' ',
#      print "%.1f"%tlist[5]+' ',
#      print "%.1f"%tlist[6]+' '

   return tlist

# function to calculate color from temperature
def fahrenheit_to_rgb(maxVal, minVal, actual):
   midVal = (maxVal - minVal)/2
   intR = 0
   intG = 0
   intB = 0

   if actual >= minVal or actual <= maxVal:
      if (actual >= midVal):
         intR = 255
         intG = round(255 * ((maxVal - actual) / (maxVal - midVal)))
      else:
         intG = 255
         intR = round(255 * ((actual - minVal) / (midVal - minVal)))

      if intR < 0:
         intR = 0
      if intR > 255:
         intR = 255
      if intG < 0:
         intG = 0
      if intG > 255:
         intG = 255
      if intB < 0:
         intB = 0
      if intB > 255:
         intB = 255

#      if DEBUG:
#         print 'Temp to RGB = ('+str(intR)+', '+str(intG)+', '+str(intB)+')'

   return ((intR, intG, intB))



def fahrenheit_to_kelvin(fahrenheit):
   kelvin = ((5*(fahrenheit - 32))/9) + 273
   if DEBUG:
      print "%.1f"%fahrenheit+' F = '+"%.1f"%kelvin+' K'
   return kelvin

def celsius_to_kelvin(celsius):
    return (celsius + 273)

###############################
#
# Start of main line program
#
###############################

# Handle command line arguments
if "-debug" in sys.argv:
   DEBUG=1				# set this to 1 to see debug messages on monitor

# Initialize variables
temperature_array=[0.0]*OMRON_DATA_LIST		# holds the recently measured temperature
temperature_previous=[0.0]*OMRON_DATA_LIST	# keeps track of the previous values
temperature_moving_ave=[0.0]*OMRON_DATA_LIST	# moving average of temperature
horiz_list=[0.0]*HORIZ_LIST_SIZE
fatal_error = 0
retries=0
played_hello=0
played_byebye=0
quadrant=[Rect]*OMRON_DATA_LIST		# quadrant of the display (x, y, width, height)
center=[(0,0)]*OMRON_DATA_LIST		# center of each quadrant
px=[0]*4
py=[0]*4

# Initialize screen
pygame.init()
font = pygame.font.Font(None, 36)

# setup the IR color window
screen = pygame.display.set_mode(SCREEN_DIMENSIONS,pygame.FULLSCREEN)
pygame.display.set_caption('IR temp array')

# initialize the window quadrant areas for displaying temperature
pixel_width = SCREEN_DIMENSIONS[0]/4
px = (pixel_width*3, pixel_width*2, pixel_width, 0)
pixel_height = SCREEN_DIMENSIONS[1]/4
py = (0, pixel_width, pixel_width*2, pixel_width*3)
for x in range(0,4):
   for y in range(0,4):
      quadrant[(x*4)+y] = (px[x], py[y], pixel_width, pixel_height)
      center[(x*4)+y] = (pixel_width/2+px[x], pixel_height/2+py[y])
if DEBUG:
   for i in range(0,OMRON_DATA_LIST):
      print 'Q['+str(i)+'] = '+str(quadrant[i])
      print 'c['+str(i)+'] = '+str(center[i])

try:
# Initialize i2c bus address
   i2c_bus = smbus.SMBus(1)
   time.sleep(0.05)				# Wait a short time

# make some space
   print ''
   if DEBUG:
      print 'DEBUG switch is on'
   if SERVO:
      print 'SERVO switch is on'
   else:
      print 'SERVO is off'


# intialize the pigpio library and socket connection to the daemon (pigpiod)
   pi = pigpio.pi()              # use defaults
   if DEBUG:
      version = pi.get_pigpio_version()
      print 'PiGPIO version = '+str(version)

# Initialize the selected Omron sensor
   if DEBUG:
      print 'Initializing Person Sensor'

   (omron1_handle, omron1_result) = omron_init(RASPI_I2C_CHANNEL, OMRON_1, pi, i2c_bus) # passing in the i2c address of the sensor

   if omron1_handle < 1:
      print 'I2C sensor not found!'
      sys.exit(0);

# servo motor inits
#   turnAway =  [1000, 0.14]		# [0] = pulse width (direction) [1] = time
#   facePerson= [2000, 0.1]		        # [0] = pulse width (direction) [1] = time

   if SERVO:
      if DEBUG:
         print 'Servo: initializing servo'
      GPIO.setmode(GPIO.BOARD)
      GPIO.setup(SERVO_GPIO_PIN,GPIO.OUT)
      servo = PWM.Servo()
      servo.set_servo(SERVO_GPIO_PIN, 1500)
      time.sleep(0.3)
      servo.stop_servo(SERVO_GPIO_PIN)

# initialze the music player
   pygame.mixer.init()

# Initialize servo position
   if SERVO:
      if DEBUG:
         print 'Servo: Turning away from person'
#   set_servo(pi, servo, turnAway[0], turnAway[1])

   print 'Looking for a person'

   Person = 0				# initialize the person tracker
   person_existed_last_time = 0
   first_time = 1

#############################
# Main while loop
#############################
   while True:			        # The main loop
      while True: 				# do this loop until a person shows up
         
         time.sleep(MEASUREMENT_WAIT_PERIOD)

         for event in pygame.event.get():
            if event.type == QUIT:
               pygame.quit()
               sys.exit()
            if event.type == KEYDOWN:
               if event.key == K_q or event.key == K_ESCAPE:
                  pygame.quit()
                  sys.exit()
               if event.key == (KMOD_LCTRL | K_c):
                  pygame.quit()
                  sys.exit()

# read the raw temperature data
# 
# save away the previous temp measurement so that a moving average can be kept
         for i in range(0,OMRON_DATA_LIST):
            temperature_previous[i] = temperature_array[i]
 
#         if DEBUG:
#            print 'Previous temperature measurement'
#            print_temps(temperature_previous)
#            print ''

# Format: (bytes_read, temperature_array, room_temp) = omron_read(sensor_handle, C/F, length of temperature array, pigpio socket handle)
# returns bytes_read - if not equal to length of temperature array, then sensor error
 
         (bytes_read, temperature_array, room_temp) = omron_read(omron1_handle, DEGREE_UNIT, OMRON_BUFFER_LENGTH, pi)

# Display each element's temperature in F
         if DEBUG:
            print 'New temperature measurement'
            print_temps(temperature_array)

         if bytes_read != OMRON_BUFFER_LENGTH: # sensor problem
            print ''
            print 'ERROR: Omron thermal sensor failure! Bytes read: '+str(bytes_read)
            print ''
            fatal_error = 1
            break

# Use the room temp value for the threshold to determine if a person is in front of the sensor
         for i in range(0,OMRON_DATA_LIST):
            list = [temperature_array[i], temperature_previous[i], temperature_moving_ave[i]]
            temperature_moving_ave[i] = avg(list)

# Display each element's temperature in F
#         if DEBUG:
#            print 'Temperature moving average'
#            print_temps(temperature_moving_ave)

         horiz_temp = make_horizontal(temperature_moving_ave)

# Display the Omron internal temperature (room temp - something to compare signals with)
         if DEBUG:
            print 'Omron D6T internal temp = '+"%.1f"%room_temp+' F'

# create the IR pixels
         for i in range(0,OMRON_DATA_LIST):
            screen.fill(fahrenheit_to_rgb(MAX_TEMP, MIN_TEMP, temperature_array[i]), quadrant[i])
# Display temp value
            if temperature_array[i] > room_temp+TEMPMARGIN:
               text = font.render("%.1f"%temperature_array[i], 1, name_to_rgb('red'))
            else:
               text = font.render("%.1f"%temperature_array[i], 1, name_to_rgb('navy'))
            textpos = text.get_rect()
            textpos.center = center[i]
            screen.blit(text, textpos)

# update the screen
         pygame.display.update()

         if max(temperature_array) > room_temp+TEMPMARGIN:    # Here is where a person is detected
            Person = 1
            break
         else:
            Person = 0
            break

#############################
# End main while loop
#############################

      if fatal_error:
         break

      if Person == 1:
         if person_existed_last_time == 0:			# person detected for the first time
            if DEBUG:
               print "Hello Person!"

# Move head
            if SERVO:
               if DEBUG:
                  print 'Servo: Facing Person'
#               set_servo(pi, servo, facePerson[0], facePerson[1])

# Play "hello" sound effect
            hello_message = random.choice(hello_audio)         
            play_sound(MAX_VOLUME, hello_message)
            person_existed_last_time = 1
            played_hello =1
         else:
            time.sleep(0.05)				# person remains in front of the device

      else:
         if person_existed_last_time == 1:			# person moved away from the device
            if DEBUG:
               print "Bye bye Person!"

# Play "bye bye" sound effect
            byebye_message = random.choice(byebye_audio)
            play_sound(MAX_VOLUME, byebye_message)
            played_byebye =1

# Move head back
            if SERVO:
               if DEBUG:
                  print 'Servo: Turning away from Person'
#               set_servo(pi, servo, turnAway[0], turnAway[1])

            person_existed_last_time = 0
         else:
            time.sleep(0.05)				# no one is in front of the device

#      if played_hello:
#         after_hello_message = random.choice(after_hello_audio)         
#         play_sound(MAX_VOLUME, after_hello_message)
#         played_hello=0

#      if played_byebye:
#         after_byebye_message = random.choice(after_byebye_audio)
#         play_sound(MAX_VOLUME, after_byebye_message)
#         played_byebye=0

   # end if

# end of main loop
except KeyboardInterrupt:
   print ''
   print 'Keyboard interrupt; cleaning up'
   pi.i2c_close(omron1_handle)

except IOError:
   print ''
   print 'I/O Error; quitting'



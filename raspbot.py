#! /usr/bin/python

# A Python command line tool for a badge leash robot
# By Greg Griffes http://yottametric.com
# GNU GPL V3 

# Jan 2015

import smbus
import sys
import getopt
import time 
import pigpio
import time
import pygame
import random
from omron_src import *		# contains omron functions

# Constants
RASPI_I2C_CHANNEL=1		# the /dev/i2c device
OMRON_1=0x0a 			# 7 bit I2C address of Omron MEMS Temp Sensor D6T-44L
OMRON_BUFFER_LENGTH=35		# Omron data buffer size
OMRON_DATA_LIST=16		# Omron data array - sixteen 16 bit words
MAX_VOLUME=1.0			# maximum speaker volume factor for pygame.mixer
TEMPMARGIN=3			# number of degrees F greater than room temp to detect a person
DEGREE_UNIT='F'			# F = Farenheit, C=Celcius
DEBUG=1				# set this to 1 to see debug messages on monitor

hello_audio = "snd/20150201_zoe-hello1.mp3", "snd/20150201_zoe-hello2.mp3", "snd/20150201_chloe-higgg.mp3"
after_hello_audio = "snd/20150201_zoe-giggle1.mp3", "snd/20150201_zoe-boeing.mp3", "snd/20150201_zoe-candy1.mp3", "snd/20150201_zoe-dontworry1.mp3", "snd/20150201_chloe-boeing.mp3", "snd/20150201_chloe-candy1.mp3", "snd/20150201_chloe-dontworry1.mp3", "snd/20150201_chloe-dontworry2.mp3", "snd/20150201_chloe-whosthat.mp3", "snd/20150201_chloe-itslooking.mp3", "snd/20150201_chloe-yippee1.mp3"
byebye_audio = "snd/20150201_zoe-goodbye1.mp3", "snd/20150201_chloe-goodbye1.mp3"
after_byebye_audio = "snd/20150201_chloe-cry1.mp3", "snd/20150201_chloe-loveu.mp3", "snd/20150201_zoe-loveu.mp3"

# function to get the average value of a list
def avg(l):
    return sum(l, 0.0) / len(l)

# Initialize variables
temperature_array=[0.0]*OMRON_DATA_LIST		# holds the recently measured temperature
temperature_previous=[0.0]*OMRON_DATA_LIST	# keeps track of the previous values
temperature_moving_ave=[0.0]*OMRON_DATA_LIST	# moving average of temperature
fatal_error = 0
retries=0
played_hello=0
played_byebye=0

# Initialize i2c bus address
i2c_bus = smbus.SMBus(1)
time.sleep(0.05)				# Wait a short time

# make some space
print ''

# intialize the pigpio library and socket connection to the daemon (pigpiod)
pi = pigpio.pi()              # use defaults
if DEBUG:
   version = pi.get_pigpio_version()
   print 'PiGPIO version = '+str(version)

# Initialize the selected Omron sensor
if DEBUG:
   print 'Initializing Person Sensor'

(omron1_handle, omron1_result) = omron_init(RASPI_I2C_CHANNEL, OMRON_1, pi, i2c_bus) # passing in the i2c address of the sensor

if omron1_handle==0:
   print 'I2C sensor not found!'
   fatal_error = 1
   sys.exit(0);

# servo motor inits
print '--- Moving servo'
servo = 4				# GPIO number 
turnAway =  [1000, 0.14]		# [0] = direction [1] = time
facePerson= [2000, 0.1]		# [0] = direction [1] = time
pi.set_servo_pulsewidth(servo, 0);	# set motor off

# initialze the music player
#print '--- Saying Hello'
pygame.mixer.init()
#pygame.mixer.music.load("snd/Zoe-giggle-01.mp3")
#pygame.mixer.music.play()
#while pygame.mixer.music.get_busy() == True:
#    continue

# Move head
#print 'Moving Head'
pw = turnAway[0] + (servo*50) 		# initialize head position
pi.set_servo_pulsewidth(servo, pw)
time.sleep(turnAway[1])
pi.set_servo_pulsewidth(servo, 0);	        # set motor off

print 'Looking for a person'
Person = 0				# initialize the person tracker
person_existed_last_time = 0
first_time = 1

while True:			        # The main loop
   while True: 				# do this loop until a person shows up
# read the raw temperature data
# 
# save away the previous temp measurement so that a moving average can be kept
      for i in range(0,OMRON_DATA_LIST):
         temperature_previous[i] = temperature_array[i]
 
# Format: (bytes_read, temperature_array, room_temp) = omron_read(sensor_handle, C/F, length of temperature array, pigpio socket handle)
# returns bytes_read - if not equal to length of temperature array, then sensor error
 
      (bytes_read, temperature_array, room_temp) = omron_read(omron1_handle, DEGREE_UNIT, OMRON_BUFFER_LENGTH, pi)

      if bytes_read != OMRON_BUFFER_LENGTH: # sensor problem
         print ''
         print 'ERROR: Omron thermal sensor failure! Bytes read: '+str(bytes_read)
         print ''
         fatal_error = 1
         break

# Use the room temp value for the threshold to determine if a person is in front of the sensor
      for i in range(0,OMRON_DATA_LIST):
         list = [temperature_array[i], temperature_previous[i]]
         temperature_moving_ave[i] = avg(list)

# Display each element's temperature in F
      if DEBUG:
         print 'Previous temperature measurement'
         print "%.1f"%temperature_previous[0]+' ',
         print "%.1f"%temperature_previous[1]+' ',
         print "%.1f"%temperature_previous[2]+' ',
         print "%.1f"%temperature_previous[3]+' ',
         print ''
         print "%.1f"%temperature_previous[4]+' ',
         print "%.1f"%temperature_previous[5]+' ',
         print "%.1f"%temperature_previous[6]+' ',
         print "%.1f"%temperature_previous[7]+' ',
         print ''
         print "%.1f"%temperature_previous[8]+' ',
         print "%.1f"%temperature_previous[9]+' ',
         print "%.1f"%temperature_previous[10]+' ',
         print "%.1f"%temperature_previous[11]+' ',
         print ''
         print "%.1f"%temperature_previous[12]+' ',
         print "%.1f"%temperature_previous[13]+' ',
         print "%.1f"%temperature_previous[14]+' ',
         print "%.1f"%temperature_previous[15]+' ',
         print ''

# Display each element's temperature in F
      if DEBUG:
         print 'New temperature measurment'
         print "%.1f"%temperature_array[0]+' ',
         print "%.1f"%temperature_array[1]+' ',
         print "%.1f"%temperature_array[2]+' ',
         print "%.1f"%temperature_array[3]+' ',
         print ''
         print "%.1f"%temperature_array[4]+' ',
         print "%.1f"%temperature_array[5]+' ',
         print "%.1f"%temperature_array[6]+' ',
         print "%.1f"%temperature_array[7]+' ',
         print ''
         print "%.1f"%temperature_array[8]+' ',
         print "%.1f"%temperature_array[9]+' ',
         print "%.1f"%temperature_array[10]+' ',
         print "%.1f"%temperature_array[11]+' ',
         print ''
         print "%.1f"%temperature_array[12]+' ',
         print "%.1f"%temperature_array[13]+' ',
         print "%.1f"%temperature_array[14]+' ',
         print "%.1f"%temperature_array[15]+' ',
         print ''

# Display each element's temperature in F
      if DEBUG:
         print 'Moving average'
         print "%.1f"%temperature_moving_ave[0]+' ',
         print "%.1f"%temperature_moving_ave[1]+' ',
         print "%.1f"%temperature_moving_ave[2]+' ',
         print "%.1f"%temperature_moving_ave[3]+' ',
         print ''
         print "%.1f"%temperature_moving_ave[4]+' ',
         print "%.1f"%temperature_moving_ave[5]+' ',
         print "%.1f"%temperature_moving_ave[6]+' ',
         print "%.1f"%temperature_moving_ave[7]+' ',
         print ''
         print "%.1f"%temperature_moving_ave[8]+' ',
         print "%.1f"%temperature_moving_ave[9]+' ',
         print "%.1f"%temperature_moving_ave[10]+' ',
         print "%.1f"%temperature_moving_ave[11]+' ',
         print ''
         print "%.1f"%temperature_moving_ave[12]+' ',
         print "%.1f"%temperature_moving_ave[13]+' ',
         print "%.1f"%temperature_moving_ave[14]+' ',
         print "%.1f"%temperature_moving_ave[15]+' ',
         print ''

# Display the Omron internal temperature (room temp - something to compare signals with)
      if DEBUG:
         print 'Omron D6T internal temp = '+"%.1f"%room_temp+' F'

      if max(temperature_array) > room_temp+TEMPMARGIN:    # Here is where a person is detected
         Person = 1
         break
      else:
         Person = 0
         break
      
# end of while loop

   if fatal_error:
      break

   if Person == 1:
      if person_existed_last_time == 0:			# person detected for the first time
         if DEBUG:
            print "Hello Person!"

# Move head
         pw = facePerson[0] + (servo*50) 		# change head position
         pi.set_servo_pulsewidth(servo, pw)
         time.sleep(facePerson[1])			# distance = time
         pi.set_servo_pulsewidth(servo, 0);	        # set motor off

# Play "hello" sound effect
         hello_message = random.choice(hello_audio)         
         pygame.mixer.music.set_volume(MAX_VOLUME)         
         pygame.mixer.music.load(hello_message)
         pygame.mixer.music.play()
         while pygame.mixer.music.get_busy() == True:
            continue
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
         pygame.mixer.music.set_volume(MAX_VOLUME)         
         pygame.mixer.music.load(byebye_message)
         pygame.mixer.music.play()
         while pygame.mixer.music.get_busy() == True:
            continue
         played_byebye =1

# Move head back
         pw = turnAway[0] + (servo*50) 		# change head position
         pi.set_servo_pulsewidth(servo, pw)
         time.sleep(turnAway[1])
         pi.set_servo_pulsewidth(servo, 0);	        # set motor off
         person_existed_last_time = 0
      else:
         time.sleep(0.05)				# no one is in front of the device

   if played_hello:
      pygame.mixer.music.set_volume(MAX_VOLUME)
      after_hello_message = random.choice(after_hello_audio)         
      pygame.mixer.music.load(after_hello_message)
      pygame.mixer.music.play()
      while pygame.mixer.music.get_busy() == True:
         continue
      played_hello=0

   if played_byebye:
      pygame.mixer.music.set_volume(MAX_VOLUME)         
      after_byebye_message = random.choice(after_byebye_audio)
      pygame.mixer.music.load(after_byebye_message)
      pygame.mixer.music.play()
      while pygame.mixer.music.get_busy() == True:
         continue
      played_byebye=0

   # end if

# end of main loop

pi.i2c_close(omron1_handle)



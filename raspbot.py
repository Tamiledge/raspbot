#! /usr/bin/python
"""
# A Python command line tool for a robot based on the Raspberry Pi computer
# By Greg Griffes http://yottametric.com
# GNU GPL V3 

# !!!!!!!!!!!!!!!!!
# remember to run this as root "sudo ./raspbot" so that DMA can be used
# for the servo
# !!!!!!!!!!!!!!!!!

# Jan 2015
"""
import smbus
import sys
import getopt
import pigpio
import time
from datetime import datetime
from webcolors import *
import pygame
from pygame.locals import *
import random
from omron_src import *     # contains omron functions
import urllib, pycurl, os   # needed for text to speech
import ping, socket

#The recipe gives simple implementation of a Discrete Proportional-Integral-
# Derivative (PID) controller. PID controller gives output value for error
# between desired reference input and measurement feedback to minimize error
# value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
####### Example #########
#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#

class PID:
    """
    Discrete PID control
    """
    def __init__(self, P = 1.0, I = 0.0, D = 1.0, Derivator = 0, Integrator = 0, Integrator_max = 500, Integrator_min = -500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

# Constants
RASPI_I2C_CHANNEL = 1     # the /dev/i2c device
OMRON_1 = 0x0a            # 7 bit I2C address of Omron MEMS Temp Sensor D6T-44L
OMRON_BUFFER_LENGTH = 35      # Omron data buffer size
OMRON_DATA_LIST = 16      # Omron data array - sixteen 16 bit words
MAX_VOLUME = 1.0          # maximum speaker volume factor for pygame.mixer
DEGREE_UNIT = 'F'         # F = Farenheit, C=Celcius
MEASUREMENT_WAIT_PERIOD = 0.3     # time between Omron measurements
SERVO = 1             # set this to 1 if the servo motor is wired up
SERVO_GPIO_PIN = 11     # GPIO number (GPIO 11 aka. SCLK)
DEBUG = 0             # set this to 1 to see debug messages on monitor
SCREEN_DIMENSIONS = [400, 600]  # setup the IR color window [0]= width [1]= height
MIN_TEMP = 0            # minimum expected temperature in Fahrenheit
MAX_TEMP = 200          # minimum expected temperature in Fahrenheit
ROAM = 0                        # if true, robot will "roam" looking for a heat signature 
BURN_HAZARD_TEMP = 100          # temperature at which a warning is given
TEMPMARGIN = 5            # number of degrees F greater than room temp to detect a person
PERSON_TEMP_THRESHOLD = 81      # degrees fahrenheit

# Servo positions
CENTER = 1500                  # Facing straign forward
POSVECT_MIN = 0                 # POSVECT is the vector position of the robot head
POSVECT_MAX = 2000              # POSVECT is a value between MIN and MAX where MIN is full clockwise and MAX is full CCW
POSVECT_OFFSET = 600            # When POSVECT is added to the offset, the number can be used to position the servo

MIN_SERVO_POSITION = 600
CTR_SERVO_POSITION = 1500
MAX_SERVO_POSITION = 2500
MINIMUM_SERVO_GRANULARITY = 10  # microseconds


SERVO_CUR_DIR_CW = 1            # Direction to move the servo next
SERVO_LIMIT_CW = MIN_SERVO_POSITION
SERVO_CUR_DIR_CCW = 2
SERVO_LIMIT_CCW = MAX_SERVO_POSITION
SERVO_GRANULARTY = 50           # minimum granularity of the servo is 10us

# Logfile
LOGFILE_NAME = 'Raspbot_logfile.txt'

import RPi.GPIO as GPIO
GPIO.setwarnings(False)         # if true, we get warnings about DMA channel in use
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
from RPIO import PWM        # for the servo motor

#HELLO_AUDIO = "snd/20150201_zoe-hello1.mp3", "snd/20150201_zoe-hello2.mp3", "snd/20150201_chloe-higgg.mp3"
HELLO_AUDIO = "snd/20150201_zoe-hello1.mp3", "snd/20150201_zoe-hello2.mp3"
AFTER_HELLO_AUDIO = "snd/20150201_zoe-boeing.mp3", "snd/20150201_chloe-boeing.mp3",  "snd/20150201_chloe-whosthat.mp3", "snd/20150201_chloe-yippee1.mp3"
#AFTER_HELLO_AUDIO = "snd/20150201_zoe-giggle1.mp3", "snd/20150201_zoe-boeing.mp3", "snd/20150201_zoe-candy1.mp3", "snd/20150201_zoe-dontworry1.mp3", "snd/20150201_chloe-boeing.mp3", "snd/20150201_chloe-candy1.mp3", "snd/20150201_chloe-dontworry1.mp3", "snd/20150201_chloe-dontworry2.mp3", "snd/20150201_chloe-whosthat.mp3", "snd/20150201_chloe-itslooking.mp3", "snd/20150201_chloe-yippee1.mp3"
BYEBYE_AUDIO = "snd/20150201_zoe-goodbye1.mp3", "snd/20150201_chloe-goodbye1.mp3"
AFTER_BYEBYE_AUDIO = "snd/20150201_chloe-cry1.mp3", "snd/20150201_chloe-loveu.mp3", "snd/20150201_zoe-loveu.mp3"

CONNECTED = 0           # true if connected to the internet

# function to get the average value of a list
def avg(incoming_list):
    """
    Calculates the average value of a list
    """
    return sum(incoming_list, 0.0) / len(incoming_list)

def play_sound(volume, message):
    """
    Play an mp3 file
    """
    pygame.mixer.music.set_volume(volume)         
    pygame.mixer.music.load(message)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continue

def print_temps(temp_list):
    """
    Display each element's temperature in F
    """
    print "%.1f"%temp_list[12]+' ',
    print "%.1f"%temp_list[8]+' ',
    print "%.1f"%temp_list[4]+' ',
    print "%.1f"%temp_list[0]+' ',
    print ''
    print "%.1f"%temp_list[13]+' ',
    print "%.1f"%temp_list[9]+' ',
    print "%.1f"%temp_list[5]+' ',
    print "%.1f"%temp_list[1]+' ',
    print ''
    print "%.1f"%temp_list[14]+' ',
    print "%.1f"%temp_list[10]+' ',
    print "%.1f"%temp_list[6]+' ',
    print "%.1f"%temp_list[2]+' ',
    print ''
    print "%.1f"%temp_list[15]+' ',
    print "%.1f"%temp_list[11]+' ',
    print "%.1f"%temp_list[7]+' ',
    print "%.1f"%temp_list[3]+' ',
    print ''

# function to calculate color from temperature
def fahrenheit_to_rgb(maxVal, minVal, actual):
    """
    Convert fahrenheit temperature to RGB color values
    """
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
    """
    Convert fahrenheit to kelvin
    """
    kelvin = ((5*(fahrenheit - 32))/9) + 273
    if DEBUG:
        print "%.1f"%fahrenheit+' F = '+"%.1f"%kelvin+' K'
    return kelvin

def celsius_to_kelvin(celsius):
    """
    Convert celsius to kelvin
    """
    return (celsius + 273)

def crash_and_burn(msg, pygame, servo, logfile):
    """
    Something bad happend; quit the program
    """
    if DEBUG:
        print msg
    if SERVO:
        servo.stop_servo(SERVO_GPIO_PIN)
    logfile.write(msg+' @ '+str(datetime.now()))
    logfile.close
    pygame.quit()
    sys.exit()

def set_servo_to_position (new_position):    # position across a line from 600 to 2500 points
    """
    Moves the servo to a new position
    """

    if SERVO:
    # make sure we don't go out of bounds
        if new_position == 0:
            new_position = ((MAX_SERVO_POSITION - MIN_SERVO_POSITION)/2) + MIN_SERVO_POSITION
        elif new_position < MIN_SERVO_POSITION:
            new_position = MIN_SERVO_POSITION
        elif new_position > MAX_SERVO_POSITION:
            new_position = MIN_SERVO_POSITION

        if (new_position >= MIN_SERVO_POSITION and new_position <= MAX_SERVO_POSITION):
            if (new_position%MINIMUM_SERVO_GRANULARITY < 5):        # if there is a remainder, then we need to make the value in 10us increments
                final_position = (new_position//MINIMUM_SERVO_GRANULARITY)*MINIMUM_SERVO_GRANULARITY
            else:
                final_position = ((new_position//MINIMUM_SERVO_GRANULARITY)+1)*MINIMUM_SERVO_GRANULARITY
            if DEBUG:
                print 'Setting servo to final position: '+str(final_position)

            servo.set_servo(SERVO_GPIO_PIN, final_position)
        else:
            if DEBUG:
                print 'ERROR: set_servo_to_position position out of range: '+str(new_position)+' min= '+str(MIN_SERVO_POSITION)+' max = '+str(MAX_SERVO_POSITION)

        return final_position

def person_position(room, t_array, s_position):
    """
    Algorithm to calculate the position of a person based on sensor data
    """

    X_DELTA_0 = 200
    X_DELTA_1 = 100
    X_DELTA_2 = -100
    X_DELTA_3 = -200

    hit_count=[0]*OMRON_DATA_LIST
    t_delta=[0.0]*OMRON_DATA_LIST       # holds the difference between threshold and actual
    x_delta=[0.0]*4                             # holds the sums of differences along the x axis
    p_delta=[0.0]*4                             # holds the sums of differences along multiplied by position
    h_delta=[0]*4

    for i in range(0,OMRON_DATA_LIST):
        if (t_array[i] > PERSON_TEMP_THRESHOLD and t_array[i] < BURN_HAZARD_TEMP):     # a person temperature
            t_delta[i] = t_array[i] - PERSON_TEMP_THRESHOLD
            hit_count[i] += 1
            
    if DEBUG:
        print 'Temperature deltas'
        print_temps(t_delta)
        print 'Hit count = '+str(hit_count)

    HIT_WEIGHT_PERCENT = 0.1
# use hit counts as a weighting factor: 1 hit = x percent increase
    h_delta[0] = hit_count[12]+hit_count[13]+hit_count[14]+hit_count[15] # add up the far left column
    h_delta[1] = hit_count[8]+hit_count[9]+hit_count[10]+hit_count[11] 
    h_delta[2] = hit_count[4]+hit_count[5]+hit_count[6]+hit_count[7] 
    h_delta[3] = hit_count[0]+hit_count[1]+hit_count[2]+hit_count[3] 

    if DEBUG:
        print 'hit count delta: ',
        print h_delta
        print ''

    x_delta[0] = (t_delta[12]+t_delta[13]+t_delta[14]+t_delta[15])*(1+(h_delta[0]*HIT_WEIGHT_PERCENT))
    x_delta[1] = (t_delta[8]+t_delta[9]+t_delta[10]+t_delta[11])*(1+(h_delta[1]*HIT_WEIGHT_PERCENT))          
    x_delta[2] = (t_delta[4]+t_delta[5]+t_delta[6]+t_delta[7])*(1+(h_delta[2]*HIT_WEIGHT_PERCENT))
    x_delta[3] = (t_delta[0]+t_delta[1]+t_delta[2]+t_delta[3])*(1+(h_delta[3]*HIT_WEIGHT_PERCENT))            # add up the far right column

    if DEBUG:
        print 'x axis delta sums: ',
        print x_delta
        print ''

    p_delta[0] = x_delta[0]*(s_position+X_DELTA_0)                      # convert the far left column to a position relative to the last position
    p_delta[1] = x_delta[1]*(s_position+X_DELTA_1)
    p_delta[2] = x_delta[2]*(s_position+X_DELTA_2)
    p_delta[3] = x_delta[3]*(s_position+X_DELTA_3)                      # far right

    if DEBUG:
        print 'p delta multiplicands: ',
        print p_delta
        print ''

    person_position = (p_delta[0]+p_delta[1]+p_delta[2]+p_delta[3])/(x_delta[0]+x_delta[1]+x_delta[2]+x_delta[3])       # estimate the location on x-axis

# make sure we don't go out of bounds
    if person_position == 0:
        person_position = CENTER
    elif person_position < MIN_SERVO_POSITION:
        person_position = MIN_SERVO_POSITION
    elif person_position > MAX_SERVO_POSITION:
        person_position = MAX_SERVO_POSITION

# make sure the result has a granularity of 10us
    if (person_position%MINIMUM_SERVO_GRANULARITY < 5):        # if there is a remainder, then we need to make the value in 10us increments
        person_position = (person_position//MINIMUM_SERVO_GRANULARITY)*MINIMUM_SERVO_GRANULARITY
    else:
        person_position = ((person_position//MINIMUM_SERVO_GRANULARITY)+1)*MINIMUM_SERVO_GRANULARITY

    if DEBUG:
        print 'person_position = '+str(person_position)

    return person_position
            
def person_detector(room, t_array):
    """
    Used to detect a persons presence using the sensor data
    """
    PERSON_TEMP_SUM_THRESHOLD = 3
    t_sum = 0
    hit_count = 0

    if DEBUG:
        print 'person temp threshold = '+str(PERSON_TEMP_THRESHOLD)

    for i in range(0,OMRON_DATA_LIST):
#        if (t_array[i] > room+TEMPMARGIN and t_array[i] < BURN_HAZARD_TEMP):     # a person temperature
        if (t_array[i] > PERSON_TEMP_THRESHOLD and t_array[i] < BURN_HAZARD_TEMP):     # a person temperature
            t_sum += t_array[i] - PERSON_TEMP_THRESHOLD
            hit_count += 1

    if DEBUG:
        print 'Temperature sum = '+str(t_sum)
        print 'Hit count = '+str(hit_count)

    if hit_count > 1 and t_sum > PERSON_TEMP_SUM_THRESHOLD:
        return True
    else:
        return False

def connected_to_internet():
    try:
        ping.verbose_ping('www.google.com', count = 3)
        if DEBUG:
            print "Connected to internet"
        return 1
    except:
        if DEBUG:
            print "Not connected to internet"
        return 0

def downloadFile(url, fileName):
    fp = open(fileName, "wb")
    curl = pycurl.Curl()
    curl.setopt(pycurl.URL, url)
    curl.setopt(pycurl.WRITEDATA, fp)
    curl.perform()
    curl.close()
    fp.close()

def getGoogleSpeechURL(phrase):
    googleTranslateURL = "http://translate.google.com/translate_tts?tl=en&"
    parameters = {'q': phrase}
    data = urllib.urlencode(parameters)
    googleTranslateURL = "%s%s" % (googleTranslateURL,data)
    return googleTranslateURL

def speakSpeechFromText(phrase, output_file_name):
    googleSpeechURL = getGoogleSpeechURL(phrase)
    downloadFile(googleSpeechURL, output_file_name)
#    pygame.mixer.music.load(output_file_name)
#    pygame.mixer.music.play()

###############################
#
# Start of main line program
#
###############################

# Handle command line arguments
if "-debug" in sys.argv:
    DEBUG=1             # set this to 1 to see debug messages on monitor
if "-noservo" in sys.argv:
    SERVO=0             # assume using servo is default
if "-roam" in sys.argv:
    ROAM=1              # set this to 1 to allow robot to roam looking for a person

if "-help" in sys.argv:
    print '-debug -noservo -roam -h -help'
    sys.exit()

# Initialize variables
temperature_array=[0.0]*OMRON_DATA_LIST     # holds the recently measured temperature
temperature_previous=[0.0]*OMRON_DATA_LIST  # keeps track of the previous values
temperature_moving_ave=[0.0]*OMRON_DATA_LIST    # moving average of temperature
left_far=[0.0]*4
left_ctr=[0.0]*4
right_ctr=[0.0]*4
right_far=[0.0]*4

fatal_error = 0
retries=0
played_hello=0
played_byebye=0
quadrant=[Rect]*OMRON_DATA_LIST     # quadrant of the display (x, y, width, height)
center=[(0,0)]*OMRON_DATA_LIST      # center of each quadrant
px=[0]*4
py=[0]*4
omron_error_count = 0
omron_read_count = 0
servo_position = CENTER                 # initialize the servo to face directly forward
servo_direction = SERVO_CUR_DIR_CCW     # initially start moving the servo CCW

# Open log file
logfile = open(LOGFILE_NAME, 'wb')
logfile.write('\r\nLog file opened at '+str(datetime.now()))

# Initialize screen
pygame.init()
font = pygame.font.Font(None, 36)

# setup the IR color window
screen = pygame.display.set_mode(SCREEN_DIMENSIONS)
#screen = pygame.display.set_mode(SCREEN_DIMENSIONS,pygame.FULLSCREEN)
pygame.display.set_caption('IR temp array')

# initialize the window quadrant areas for displaying temperature
pixel_width = SCREEN_DIMENSIONS[0]/4
px = (pixel_width*3, pixel_width*2, pixel_width, 0)
pixel_height = SCREEN_DIMENSIONS[0]/4               # using width here to keep an equal square; bottom section used for messages
py = (0, pixel_width, pixel_width*2, pixel_width*3)
for x in range(0,4):
    for y in range(0,4):
        quadrant[(x*4)+y] = (px[x], py[y], pixel_width, pixel_height)
        center[(x*4)+y] = (pixel_width/2+px[x], pixel_height/2+py[y])
#if DEBUG:
#   for i in range(0,OMRON_DATA_LIST):
#      print 'Q['+str(i)+'] = '+str(quadrant[i])
#      print 'c['+str(i)+'] = '+str(center[i])

# initialize the location of the message area
room_temp_area = (0, SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
room_temp_msg_xy = (SCREEN_DIMENSIONS[0]/2, (SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

message_area = (0, SCREEN_DIMENSIONS[0]+SCREEN_DIMENSIONS[0]/4, SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
message_area_xy = (SCREEN_DIMENSIONS[0]/2, (SCREEN_DIMENSIONS[1]/6)+(SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

try:
# Initialize i2c bus address
    logfile.write('\r\nInitializing smbus at '+str(datetime.now()))
    i2c_bus = smbus.SMBus(1)
    time.sleep(0.05)                # Wait a short time

# make some space
    print ''
    if DEBUG:
        print 'DEBUG switch is on'
    if SERVO:
# Initialize servo position
        if DEBUG:
            print 'Servo: initializing servo'
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_GPIO_PIN,GPIO.OUT)
        servo = PWM.Servo()
        servo.set_servo(SERVO_GPIO_PIN, CTR_SERVO_POSITION)
    else:
        print 'SERVO is off'


# intialize the pigpio library and socket connection to the daemon (pigpiod)
    pi = pigpio.pi()              # use defaults
    version = pi.get_pigpio_version()
    if DEBUG:
        print 'PiGPIO version = '+str(version)
    logfile.write('\r\nPiGPIO version = '+str(version))

# Initialize the selected Omron sensor
    if DEBUG:
        print 'Initializing Person Sensor'

    (omron1_handle, omron1_result) = omron_init(RASPI_I2C_CHANNEL, OMRON_1, pi, i2c_bus) # passing in the i2c address of the sensor

    if omron1_handle < 1:
        crash_msg = '\r\nI2C sensor not found!'
        crash_and_burn(crash_msg, pygame, servo, logfile)

# initialze the music player
    pygame.mixer.init()

    print 'Looking for a person'
    logfile.write('\r\nLooking for a person at '+str(datetime.now()))

    person = 0              # initialize the person tracker
    person_existed_last_time = 0
    first_time = 1
    burn_hazard = 0
################################
# initialize the PID controller
################################

#    pid_controller=PID(1.0,0.1,0.1)     # PID controller is the feedback loop controller for person following
    pid_controller=PID(1.0,0.1,0.1)     # PID controller is the feedback loop controller for person following
    SETTLE_TIME = 1.0                   # the time in seconds to allow the temperatures to settle once a person is found and the head has moved
    MINIMUM_ERROR_GRANULARITY = 50     # the number of microseconds - if the PID error is less than this, the head will stop moving
    previous_pid_error = 0

    CONNECTED = connected_to_internet
    HELLO_FILE_NAME = "hello_file.mp3"
    GOODBYE_FILE_NAME = "byebye_file.mp3"
    BADGE_FILE_NAME = "badge_file.mp3"
    MOVE_FILE_NAME = "move_file.mp3"
    BORED_FILE_NAME = "bored_file.mp3"
    BURN_FILE_NAME = "burn_file.mp3"
    
    if CONNECTED:
        speakSpeechFromText("Yeay,,,,,, we are connected to the Internet!", "intro.mp3")
        logfile.write('\r\nConnected to the Internet')
        play_sound(MAX_VOLUME, "intro.mp3")
        speakSpeechFromText("Hello!", HELLO_FILE_NAME)
        speakSpeechFromText("Good bye", GOODBYE_FILE_NAME)
        speakSpeechFromText("STOP!,,,, Don't forget your badge!", BADGE_FILE_NAME)
        speakSpeechFromText("Yo!", MOVE_FILE_NAME)
        speakSpeechFromText("I'm so bored...", BORED_FILE_NAME)
        speakSpeechFromText("WARNING: Somethings very HOT! Don't get burned!", BURN_FILE_NAME)
        
    else:
        logfile.write('\r\nNOT connected to the Internet')        

#############################
# Main while loop
#############################
    while True:                 # The main loop
        while True:                 # do this loop until a person shows up
         
            time.sleep(MEASUREMENT_WAIT_PERIOD)

            for event in pygame.event.get():
                if event.type == QUIT:
                    crash_msg = '\r\npygame event QUIT'
                    crash_and_burn(crash_msg, pygame, servo, logfile)
                if event.type == KEYDOWN:
                    if event.key == K_q or event.key == K_ESCAPE:
                        crash_msg = '\r\npygame event: keyboard q or esc pressed'
                        crash_and_burn(crash_msg, pygame, servo, logfile)
                    if event.key == (KMOD_LCTRL | K_c):
                        crash_msg = '\r\npygame event: keyboard ^c pressed'
                        crash_and_burn(crash_msg, pygame, servo, logfile)

# read the raw temperature data
# 
# save away the previous temp measurement so that a moving average can be kept
            for i in range(0,OMRON_DATA_LIST):
                temperature_previous[i] = temperature_array[i]
 
            if DEBUG:
                print 'Previous temperature measurement'
                print_temps(temperature_previous)
                print ''

# Format: (bytes_read, temperature_array, room_temp) = omron_read(sensor_handle, C/F, length of temperature array, pigpio socket handle)
# returns bytes_read - if not equal to length of temperature array, then sensor error
 
            (bytes_read, temperature_array, room_temp) = omron_read(omron1_handle, DEGREE_UNIT, OMRON_BUFFER_LENGTH, pi)
            omron_read_count += 1
         
# Display each element's temperature in F
            if DEBUG:
                print 'New temperature measurement'
            print_temps(temperature_array)

            if bytes_read != OMRON_BUFFER_LENGTH: # sensor problem
                omron_error_count += 1
                print ''
                print 'ERROR: Omron thermal sensor failure! Bytes read: '+str(bytes_read)
                print ''
                logfile.write('\r\nOmron sensor failure count: '+str(omron_error_count)+' out of : '+str(omron_read_count)+'. Bytes read = '+str(bytes_read)+'at '+str(datetime.now()))
                fatal_error = 1
                break

            for i in range(0,OMRON_DATA_LIST):
                temp_list = [temperature_array[i], temperature_previous[i], temperature_moving_ave[i]]
                temperature_moving_ave[i] = avg(temp_list)

# Display each element's temperature in F
            if DEBUG:
                print 'Temperature moving average'
                print_temps(temperature_moving_ave)

# Display the Omron internal temperature (room temp - something to compare signals with)
            if DEBUG:
                print 'Omron D6T internal temp = '+"%.1f"%room_temp+' F'

# create the IR pixels
            for i in range(0,OMRON_DATA_LIST):
# This fills each little array square with a background color that matches the temp
                screen.fill(fahrenheit_to_rgb(MAX_TEMP, MIN_TEMP, temperature_array[i]), quadrant[i])
# Display temp value
                if temperature_array[i] > PERSON_TEMP_THRESHOLD:
                    text = font.render("%.1f"%temperature_array[i], 1, name_to_rgb('red'))
                else:
                    text = font.render("%.1f"%temperature_array[i], 1, name_to_rgb('navy'))
                textpos = text.get_rect()
                textpos.center = center[i]
                screen.blit(text, textpos)

# Create an area to display the room temp and messages
            screen.fill(fahrenheit_to_rgb(MAX_TEMP, MIN_TEMP, room_temp), room_temp_area)
            text = font.render("Room: %.1f"%room_temp, 1, name_to_rgb('navy'))
            textpos = text.get_rect()
            textpos.center = room_temp_msg_xy
            screen.blit(text, textpos)

# update the screen
            pygame.display.update()

###########################
# Burn Hazard Detected !
###########################
            if max(temperature_array) > BURN_HAZARD_TEMP:
                screen.fill(name_to_rgb('red'), message_area)
                text = font.render("WARNING! Burn danger!", 1, name_to_rgb('yellow'))
                textpos = text.get_rect()
                textpos.center = message_area_xy
                screen.blit(text, textpos)
# update the screen
                pygame.display.update()
                play_sound(MAX_VOLUME, BURN_FILE_NAME)
                if CONNECTED:
                    speakSpeechFromText("The temperature is "+"%.1f"%max(temperature_array)+" degrees fahrenheit", "mtemp.mp3")
                    play_sound(MAX_VOLUME, "mtemp.mp3")

                burn_hazard = 1
                break

###########################
# Person Detected !
###########################
            elif person_detector(room_temp, temperature_array):    # Here is where a person is detected
                screen.fill(name_to_rgb('white'), message_area)
                text = font.render("Hello!", 1, name_to_rgb('red'))
                textpos = text.get_rect()
                textpos.center = message_area_xy
                screen.blit(text, textpos)
# update the screen
                pygame.display.update()

                if SERVO:
# face the servo twoards the heat
                    p_pos = person_position(room_temp, temperature_array, servo_position)
                    pid_controller.setPoint(p_pos)                      # setpoint is the desired position
                    pid_error = pid_controller.update(servo_position)         # process variable is current position
                    if DEBUG:
                        print 'Des Pos: '+str(p_pos),
                        print ' Cur Pos: '+str(servo_position),
                        print ' PID Error: '+str(pid_error)

# make the robot turn its head to the person
# if previous error is the same absolute value as the current error, then we are oscillating - stop it
                    if abs(pid_error) > MINIMUM_ERROR_GRANULARITY:
                        previous_pid_error = pid_error
                        servo_position += pid_error
                        servo_position = set_servo_to_position(servo_position)
                        play_sound(MAX_VOLUME, MOVE_FILE_NAME)
                        if (pid_error < -100):
                            play_sound(MAX_VOLUME, BADGE_FILE_NAME)

                        time.sleep(MEASUREMENT_WAIT_PERIOD*SETTLE_TIME)                 #let the temp's settle

                person = 1
                burn_hazard = 0
                break

###########################
# Nobody Detected !
###########################
            else:
                screen.fill(name_to_rgb('white'), message_area)
                text = font.render("Waiting...", 1, name_to_rgb('blue'))
                textpos = text.get_rect()
                textpos.center = message_area_xy
                screen.blit(text, textpos)
# update the screen
                pygame.display.update()
                person = 0
                burn_hazard = 0

# put servo in roaming mode

            if SERVO and ROAM:

                if (servo_position >= SERVO_LIMIT_CCW):
                    if DEBUG:
                        print 'CCW limit hit, changing direction'
                    servo_direction = SERVO_CUR_DIR_CW
                    #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                if (servo_position <= SERVO_LIMIT_CW):
                    if DEBUG:
                        print 'CW limit hit, changing direction'
                    servo_direction = SERVO_CUR_DIR_CCW
                    #play_sound(MAX_VOLUME, BORED_FILE_NAME)
               
                if DEBUG:
                    print 'Servo: Roaming. Position: '+str(servo_position),
                if servo_direction == SERVO_CUR_DIR_CCW:
                    if DEBUG:
                        print ' Direction: CCW'
                    servo_position += MINIMUM_SERVO_GRANULARITY
                if servo_direction == SERVO_CUR_DIR_CW:
                    if DEBUG:
                        print ' Direction: CW'
                    servo_position -= MINIMUM_SERVO_GRANULARITY
                  
                servo_position = set_servo_to_position(servo_position)

# End of inner While loop
            break

#############################
# End main while loop
#############################

        if fatal_error:
            break

        if person == 1:
            if person_existed_last_time == 0:           # person detected for the first time

                screen.fill(name_to_rgb('white'), message_area)
                text = font.render("Hello!", 1, name_to_rgb('red'))
                textpos = text.get_rect()
                textpos.center = message_area_xy
                screen.blit(text, textpos)
# update the screen
                pygame.display.update()

                if DEBUG:
                    print '************************** Hello Person! **************************'

# Move head
#            if SERVO:
#               if DEBUG:
#                  print 'Servo: Facing Person'
#               servo.set_servo(SERVO_GPIO_PIN, CCW_HALF)
#               time.sleep(0.5)         # Wait for the temps to normalize

# Play "hello" sound effect
                play_sound(MAX_VOLUME, HELLO_FILE_NAME)

                if CONNECTED:
                    speakSpeechFromText("The room temperature is "+"%.1f"%room_temp+" degrees fahrenheit", "rtemp.mp3")
                    play_sound(MAX_VOLUME, "rtemp.mp3")

                    #speakSpeechFromText("and your temperature is "+"%.1f"%max(temperature_array)+" degrees fahrenheit", "mtemp.mp3")
                    #play_sound(MAX_VOLUME, "mtemp.mp3")
                    

                person_existed_last_time = 1
                played_hello =1

        else:
            if person_existed_last_time == 1:           # person moved away from the device

                screen.fill(name_to_rgb('white'), message_area)
                text = font.render("Bye bye!", 1, name_to_rgb('red'))
                textpos = text.get_rect()
                textpos.center = message_area_xy
                screen.blit(text, textpos)
# update the screen
                pygame.display.update()

                if DEBUG:
                    print '************************** Bye Bye Person! **************************'

# Move head
#            if SERVO:
#               if DEBUG:
#                  print 'Servo: Facing AWAY'
#               servo.set_servo(SERVO_GPIO_PIN, CW_HALF)
#               time.sleep(0.5)         # Wait for the temps to normalize

# Play "bye bye" sound effect
                #byebye_message = random.choice(BYEBYE_FILE_NAME)
                play_sound(MAX_VOLUME, BADGE_FILE_NAME)
                play_sound(MAX_VOLUME, GOODBYE_FILE_NAME)

                played_byebye =1
                person_existed_last_time = 0

#      if played_hello:
#         after_hello_message = random.choice(AFTER_HELLO_AUDIO)         
#         play_sound(MAX_VOLUME, after_hello_message)
#         played_hello=0

#      if played_byebye:
#         after_byebye_message = random.choice(AFTER_BYEBYE_AUDIO)
#         play_sound(MAX_VOLUME, after_byebye_message)
#         played_byebye=0

   # end if

# end of main loop

except KeyboardInterrupt:
    crash_msg = '\r\nKeyboard interrupt; quitting'
    crash_and_burn(crash_msg, pygame, servo, logfile)

except IOError:
    crash_msg = '\r\nI/O Error; quitting'
    crash_and_burn(crash_msg, pygame, servo, logfile)


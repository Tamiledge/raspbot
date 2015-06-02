#! /usr/bin/python
"""
# A Python command line tool for a robot based on the Raspberry Pi
# By Greg Griffes http://yottametric.com
# GNU GPL V3
#
# This file is automatically run at boot time using the following method
# edit the /etc/rc.local file using sudo nano /etc/rc.local
# add these two lines at the end before "exit 0"
# sudo pigpiod # starts the pigpio daemon
# sudo python /home/pi/<path>/raspbot.py -nomonitor -roam &
#
# The log file created by this program when running independently is
# located at root (/) about every five minutes the log file is closed
# and reopened.
#
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
from omron_src import omron_init     # contains omron functions
from omron_src import omron_read     # contains omron functions
import urllib, pycurl, os   # needed for text to speech
from pid import PID
from raspbot_functions import *

def debugPrint(message):
    nowString = str(datetime.now())
    if DEBUG and MONITOR:
        print nowString+': '+message
    logfile.write('\r\n'+nowString+': '+message)
    
def print_temps(temp_list):
    """
    Display each element's temperature in F
    """
    debugPrint("%.1f"%temp_list[12]+' '+"%.1f"%temp_list[8]+ \
               ' '+"%.1f"%temp_list[4]+' '+"%.1f"%temp_list[0]+' ')
    debugPrint("%.1f"%temp_list[13]+' '+"%.1f"%temp_list[9]+ \
               ' '+"%.1f"%temp_list[5]+' '+"%.1f"%temp_list[1]+' ')
    debugPrint("%.1f"%temp_list[14]+' '+"%.1f"%temp_list[10]+ \
               ' '+"%.1f"%temp_list[6]+' '+"%.1f"%temp_list[2]+' ')
    debugPrint("%.1f"%temp_list[15]+' '+"%.1f"%temp_list[11]+ \
               ' '+"%.1f"%temp_list[7]+' '+"%.1f"%temp_list[3]+' ')

def get_hit_array(room, t_array, s_position):
    """
    Used to fill a 4x4 array with a person "hit" temperature logical
    """
    hitCount = 0

    hit_count=[0]*OMRON_DATA_LIST
    # t_delta holds the difference between threshold and actual
    t_delta=[0.0]*OMRON_DATA_LIST 
    h_delta=[0]*4

    for i in range(0,OMRON_DATA_LIST):
        if (t_array[i] > PERSON_TEMP_THRESHOLD and \
            t_array[i] < BURN_HAZARD_TEMP):     # a person temperature
            
            t_delta[i] = t_array[i] - PERSON_TEMP_THRESHOLD
            hit_count[i] += 1
            hitCount += 1
            
#    debugPrint('Hit count = '+str(hit_count))
#    debugPrint('Temperature deltas')
#    print_temps(t_delta)
     
# use hit counts as a weighting factor: 1 hit = x percent increase
    # far left column
    h_delta[0] = hit_count[12]+hit_count[13]+hit_count[14]+hit_count[15]
    h_delta[1] = hit_count[8]+hit_count[9]+hit_count[10]+hit_count[11] 
    h_delta[2] = hit_count[4]+hit_count[5]+hit_count[6]+hit_count[7] 
    # far right column
    h_delta[3] = hit_count[0]+hit_count[1]+hit_count[2]+hit_count[3] 

    debugPrint('hit_count_delta: '+str(h_delta[0])+str(h_delta[1])+ \
               str(h_delta[2])+str(h_delta[3]))

    return h_delta, hitCount

def set_servo_to_position (new_position):
    """
    Moves the servo to a new position
    """

    if SERVO:
    # make sure we don't go out of bounds
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            if new_position == 0:
                new_position = CTR_SERVO_POSITION
            elif new_position < MAX_SERVO_POSITION:
                new_position = MAX_SERVO_POSITION
            elif new_position > MIN_SERVO_POSITION:
                new_position = MIN_SERVO_POSITION
        else:
            if new_position == 0:
                new_position = CTR_SERVO_POSITION
            elif new_position < MIN_SERVO_POSITION:
                new_position = MIN_SERVO_POSITION
            elif new_position > MAX_SERVO_POSITION:
                new_position = MAX_SERVO_POSITION

#        debugPrint('Desired servo position: '+str(new_position))
            
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            if (new_position >= MAX_SERVO_POSITION and \
                new_position <= MIN_SERVO_POSITION):
                # if there is a remainder, make 10us increments
                if (new_position%MINIMUM_SERVO_GRANULARITY < 5):
                    final_position = \
                    (new_position//MINIMUM_SERVO_GRANULARITY) \
                    *MINIMUM_SERVO_GRANULARITY
                else:
                    final_position = \
                    ((new_position//MINIMUM_SERVO_GRANULARITY)+1) \
                    *MINIMUM_SERVO_GRANULARITY
                servo.set_servo(SERVO_GPIO_PIN, final_position)
            else:
                debugPrint \
                ('ERROR: set_servo_to_position L-H=CW out of range: ' \
                 +str(new_position)+' min= '+str(MIN_SERVO_POSITION)+ \
                 ' max = '+str(MAX_SERVO_POSITION))
        else:
            if (new_position >= MIN_SERVO_POSITION and \
                new_position <= MAX_SERVO_POSITION):
                # if there is a remainder, make 10us increments
                if (new_position%MINIMUM_SERVO_GRANULARITY < 5):
                    final_position = \
                    (new_position//MINIMUM_SERVO_GRANULARITY) \
                    *MINIMUM_SERVO_GRANULARITY
                else:
                    final_position = \
                    ((new_position//MINIMUM_SERVO_GRANULARITY)+1) \
                    *MINIMUM_SERVO_GRANULARITY
                servo.set_servo(SERVO_GPIO_PIN, final_position)
            else:
                debugPrint \
                ('ERROR: set_servo_to_position L-H=CCW out of range: ' \
                 +str(new_position)+ ' min= '+str(MIN_SERVO_POSITION)+ \
                 ' max = '+str(MAX_SERVO_POSITION))

            debugPrint('SERVO_MOVE: '+str(final_position))

        return final_position

# number of microseconds to move the servo if person is not near cetner
FAR = 220
# number of microseconds to move the servo if person is near cetner
NEAR = 120

def person_position_2_hit(room, t_array, s_position):
    """
    Detect a persons presence using the "greater than two algorithm"
    returns (TRUE if person detected, approximate person position)
    """

    hit_array, hitCnt = get_hit_array(room, t_array, s_position)

# First, look for > two hits in a single column
    if (hit_array[1] >= 2 and hit_array[2] >= 2):
        # stop
        return (True, s_position)
    elif (hit_array[0] >= 2 and hit_array[1] <= 1 and \
          hit_array[2] <= 1 and hit_array[3] <= 1):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + FAR)
        else:
            return (True, s_position - FAR)
    elif (hit_array[1] >= 2 and hit_array[2] <= 1 and \
          hit_array[3] <= 1):
        # move CW 15
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR)
        else:
            return (True, s_position - NEAR)
    elif (hit_array[2] >= 2 and hit_array[0] <= 1 and \
          hit_array[1] <= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR)
        else:
            return (True, s_position + NEAR)
    elif (hit_array[3] >= 2 and hit_array[0] <= 1 and \
          hit_array[1] <= 1 and hit_array[2] <= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - FAR)
        else:
            return (True, s_position + FAR)
    else:
        # no person detected
        return (False, s_position)

def person_position_quad_hit(room, t_array, s_position):
    """
    Detect a persons presence using "look for hits in a 2x2 quadrant"
    returns (TRUE if person detected, approximate person position)
    """

    hit_array, hitCnt = get_hit_array(room, t_array, s_position)

# First, look for center quad hits
    if (hit_array[1] >= 2 and hit_array[2] >= 2):
        # stop
        return (True, s_position)
    elif (hit_array[0] >= 2 and hit_array[0] >= 2):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR)
        else:
            return (True, s_position - NEAR)
    elif (hit_array[2] >= 2 and hit_array[3] >= 2):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR)
        else:
            return (True, s_position + NEAR)
    else:
        # no person detected
        return (False, s_position)

FAR_ONE = 240       
NEAR_ONE = 100      
FAR_TWO = 170       
NEAR_THREE = 30     
# 0001 hits = FAR_ONE CCW
# 0010 hits = NEAR_ONE CCW
# 0100 hits = NEAR_ONE CW
# 1000 hits = FAR_ONE CW
#
# 0011 hits = FAR_TWO CCW
# 0110 hits = no change
# 1100 hits = FAR_TWO CW
#
# 0111 hits = NEAR_THREE CCW
# 1110 hits = NEAR_THREE CW
# 1111 hits = no change

def person_position_1_hit(room, t_array, s_position):
    """
    Detect a persons presence using "greater than one algorithm"
    returns (TRUE if person detected, approximate person position)
    """

    hit_array, hitCnt = get_hit_array(room, t_array, s_position)

    if ((hit_array[0] >= 1 or hit_array[0] == 0) and \
        hit_array[1] >= 1 and hit_array[2] >= 1 and \
        (hit_array[3] >= 1 or hit_array[3] == 0)):
        # no change
        return (True, s_position)
    elif (hit_array[0] == 0 and hit_array[1] == 0 and \
          hit_array[2] == 0 and hit_array[3] >= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - FAR_ONE)
        else:
            return (True, s_position + FAR_ONE)
    elif (hit_array[0] == 0 and hit_array[1] == 0 and \
          hit_array[2] >= 1 and hit_array[3] == 0):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR_ONE)
        else:
            return (True, s_position + NEAR_ONE)
    elif (hit_array[0] == 0 and hit_array[1] >= 1 and \
          hit_array[2] == 0 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR_ONE)
        else:
            return (True, s_position - NEAR_ONE)
    elif (hit_array[0] >= 1 and hit_array[1] == 0 and \
          hit_array[2] == 0 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + FAR_ONE)
        else:
            return (True, s_position - FAR_ONE)
    elif (hit_array[0] == 0 and hit_array[1] == 0 and \
          hit_array[2] >= 1 and hit_array[3] >= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - FAR_TWO)
        else:
            return (True, s_position + FAR_TWO)
    elif (hit_array[0] >= 1 and hit_array[1] >= 1 and \
          hit_array[2] == 0 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + FAR_TWO)
        else:
            return (True, s_position - FAR_TWO)
    elif (hit_array[0] == 0 and hit_array[1] >= 1 and \
          hit_array[2] >= 1 and hit_array[3] >= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR_THREE)
        else:
            return (True, s_position + NEAR_THREE)
    elif (hit_array[0] >= 1 and hit_array[1] >= 1 and \
          hit_array[2] >= 1 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR_THREE)
        else:
            return (True, s_position - NEAR_THREE)
    else:
        # no person detected
        return (False, s_position)

def moveHead(position, servo_pos):
    if SERVO:
# face the servo twoards the heat
        # setpoint is the desired position
        pid_controller.setPoint(position)
        # process variable is current position
        pid_error = pid_controller.update(servo_pos)
        debugPrint('Des Pos: '+str(position)+ \
                   ' Cur Pos: '+str(servo_pos)+ \
                   ' PID Error: '+str(pid_error))

# make the robot turn its head to the person
# if previous error is the same absolute value as the current error,
# then we are oscillating - stop it
        if abs(pid_error) > MINIMUM_ERROR_GRANULARITY:
            previous_pid_error = pid_error
            if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                servo_pos += pid_error
            else:
                servo_pos -= pid_error
                           
        new_servo_pos = set_servo_to_position(servo_pos)

        #let the temp's settle
        time.sleep(MEASUREMENT_WAIT_PERIOD*SETTLE_TIME)

        return new_servo_pos

def sayHello():
    if MONITOR:
        screen.fill(name_to_rgb('white'), message_area)
        text = font.render("Hello!", 1, name_to_rgb('red'))
        textpos = text.get_rect()
        textpos.center = message_area_xy
        screen.blit(text, textpos)
# update the screen
        pygame.display.update()

    debugPrint('**************************')
    debugPrint(' Hello Person! ')
    debugPrint('**************************')

# Play "hello" sound effect
    debugPrint('Playing hello audio')
    play_sound(MAX_VOLUME, HELLO_FILE_NAME)
#    time.sleep(1)
#    play_sound(MAX_VOLUME, AFTER_HELLO_FILE_NAME)
#    debugPrint('Played after hello audio')

def sayGoodBye():
    if MONITOR:
        screen.fill(name_to_rgb('white'), message_area)
        text = font.render("Good Bye!", 1, name_to_rgb('red'))
        textpos = text.get_rect()
        textpos.center = message_area_xy
        screen.blit(text, textpos)
# update the screen
        pygame.display.update()

    debugPrint('**************************')
    debugPrint(' Good Bye Person! ')
    debugPrint('**************************')

# Play "bye bye" sound effect
    #byebye_message = random.choice(BYEBYE_FILE_NAME)
    debugPrint('Playing badge audio')
    play_sound(MAX_VOLUME, BADGE_FILE_NAME)

    debugPrint('Playing good bye audio')
    play_sound(MAX_VOLUME, GOODBYE_FILE_NAME)


def play_sound(volume, message):
    """
    Play an mp3 file
    """
    pygame.mixer.music.set_volume(volume)         
    pygame.mixer.music.load(message)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continue

def crash_and_burn(msg, pygame, servo, logfile):
    """
    Something bad happend; quit the program
    """
    debugPrint(msg)
    if SERVO:
        servo.stop_servo(SERVO_GPIO_PIN)
    LED_state = False
    GPIO.output(LED_GPIO_PIN, LED_state)
    pygame.quit()
    logfile.write(msg+' @ '+str(datetime.now()))
    logfile.close
    sys.exit()

# Constants
RASPI_I2C_CHANNEL = 1       # the /dev/i2c device
OMRON_1 = 0x0a              # 7 bit I2C address of Omron Sensor D6T-44L
OMRON_BUFFER_LENGTH = 35    # Omron data buffer size
OMRON_DATA_LIST = 16        # Omron data array - sixteen 16 bit words
MAX_VOLUME = 1.0            # maximum speaker volume for pygame.mixer
DEGREE_UNIT = 'F'           # F = Farenheit, C=Celcius
MEASUREMENT_WAIT_PERIOD = 0.3     # time between Omron measurements
SERVO = 1               # set this to 1 if the servo motor is wired up
SERVO_GPIO_PIN = 11     # GPIO number (GPIO 11 aka. SCLK)
LED_GPIO_PIN = 7    # GPIO number that the LED is connected to
                    # (BCM GPIO_04 (Pi Hat) is the same as BOARD pin 7)
                    # See "Raspberry Pi B+ J8 Header" diagram
DEBUG = 0           # set this to 1 to see debug messages on monitor
SCREEN_DIMENSIONS = [400, 600]  # setup IR window [0]= width [1]= height
MIN_TEMP = 0            # minimum expected temperature in Fahrenheit
MAX_TEMP = 200          # minimum expected temperature in Fahrenheit
ROAM = 0                # if true, robot will look for a heat signature
ROAM_MAX = 600          # Max number of times to roam between person
                        # detections (roughly 0.5 seconds between roams
LOG_MAX = 1200
RAND = 0                # Causes random head movement when idle
BURN_HAZARD_TEMP = 100  # temperature at which a warning is given
TEMPMARGIN = 5          # degrees > than room temp to detect person
PERSON_TEMP_THRESHOLD = 81      # degrees fahrenheit
MONITOR = 1             # assume a monitor is attached
# Servo positions
# Weirdness factor: Some servo's I used go in the reverse direction
# from other servos. Therefore, this next constant is used to change the
# software to use the appropriate servo. The HiTEC HS-55 Feather servo.

LOW_TO_HIGH_IS_COUNTERCLOCKWISE = 0
LOW_TO_HIGH_IS_CLOCKWISE = 1
HITEC_HS55 = LOW_TO_HIGH_IS_CLOCKWISE
SERVO_TYPE = HITEC_HS55

CTR_SERVO_POSITION = 1500
MINIMUM_SERVO_GRANULARITY = 10  # microseconds
SERVO_CUR_DIR_CW = 1            # Direction to move the servo next
SERVO_CUR_DIR_CCW = 2
ROAMING_GRANULARTY = 50
HIT_WEIGHT_PERCENT = 0.1
PERSON_TEMP_SUM_THRESHOLD = 3
DETECT_COUNT_THRESH = 3

# Strange things happen: Some servos move CW and others move CCW for the
# same number. # it is possible that the "front" of the servo might be
# treated differently and it seams that the colors of the wires on the
# servo might indicate different servos:
# brown, red, orange seems to be HIGH_TO_LOW is clockwise
# (2400 is full CCW and 600 is full CW)
# black, red, yellos seems to be LOW_TO_HIGH is clockwise
# (2400 is full CW and 600 is full CCW)
if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
    MIN_SERVO_POSITION = 2300
    MAX_SERVO_POSITION = 600
    SERVO_LIMIT_CW = MIN_SERVO_POSITION
    SERVO_LIMIT_CCW = MAX_SERVO_POSITION
    X_DELTA_0 = 200
    X_DELTA_1 = 100
    X_DELTA_2 = -100
    X_DELTA_3 = -200
else:
    MIN_SERVO_POSITION = 600
    MAX_SERVO_POSITION = 2300
    SERVO_LIMIT_CW = MAX_SERVO_POSITION
    SERVO_LIMIT_CCW = MIN_SERVO_POSITION
    X_DELTA_0 = -200
    X_DELTA_1 = -100
    X_DELTA_2 = 100
    X_DELTA_3 = 200

# Logfile
LOGFILE_NAME = 'raspbot_logfile.txt'

import RPi.GPIO as GPIO
GPIO.setwarnings(False) # turn off warnings about DMA channel in use
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
GPIO.setup(LED_GPIO_PIN, GPIO.OUT)
from RPIO import PWM        # for the servo motor

CONNECTED = 0           # true if connected to the internet

###############################
#
# Start of main line program
#
###############################

# Handle command line arguments
if "-debug" in sys.argv:
    DEBUG=1         # set this to 1 to see debug messages on monitor

if "-noservo" in sys.argv:
    SERVO=0         # assume using servo is default

if "-nomonitor" in sys.argv:
    MONITOR=0       # assume using servo is default

if "-roam" in sys.argv:
    ROAM=1          # set this to 1 to roam looking for a person

if "-rand" in sys.argv:
    RAND=1          # set this to 1 to randomize looking for a person

if "-help" in sys.argv:
    print 'IMPORTANT: run as superuser (sudo) to allow DMA access'
    print '-debug:   print debug info to console'
    print '-nomonitor run without producing the pygame temp display'
    print '-noservo: do not use the servo motor'
    print '-roam:    when no person turn head slowly 180 degrees'
    print '-rand:    when roaming randomize the head movement'
    sys.exit()

# Initialize variables
# holds the recently measured temperature
temperature_array=[0.0]*OMRON_DATA_LIST
# keeps track of the previous values
temperature_previous=[0.0]*OMRON_DATA_LIST
left_far=[0.0]*4
left_ctr=[0.0]*4
right_ctr=[0.0]*4
right_far=[0.0]*4

LED_state = True
# keep track of head roams so that we can turn it off
roam_count = 0
fatal_error = 0
retries=0
played_hello=0
played_byebye=0
# quadrant of the display (x, y, width, height)
quadrant=[Rect]*OMRON_DATA_LIST
center=[(0,0)]*OMRON_DATA_LIST      # center of each quadrant
px=[0]*4
py=[0]*4
omron_error_count = 0
omron_read_count = 0
previousHitCnt = 0
hitCnt = 0
hit_array=[0]*4
# initialize the servo to face directly forward
servo_position = CTR_SERVO_POSITION
# set initial direction
if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
    servo_direction = SERVO_CUR_DIR_CW
else:
    servo_direction = SERVO_CUR_DIR_CCW

# Initialize screen
pygame.init()
font = pygame.font.Font(None, 36)

try:
# Initialize i2c bus address
    i2c_bus = smbus.SMBus(1)
    time.sleep(0.1)                # Wait

# make some space
    print ''
    if DEBUG:
        print 'DEBUG switch is on'
    if SERVO:
# Initialize servo position
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_GPIO_PIN,GPIO.OUT)
        servo = PWM.Servo()
        servo.set_servo(SERVO_GPIO_PIN, CTR_SERVO_POSITION)
        time.sleep(10.0)            # Wait a sec before starting
    else:
        print 'SERVO is off'
    LED_state = True
    GPIO.output(LED_GPIO_PIN, LED_state)

# intialize pigpio library and socket connection for daemon (pigpiod)
    pi = pigpio.pi()              # use defaults
    version = pi.get_pigpio_version()

# Initialize the selected Omron sensor

    (omron1_handle, omron1_result) = \
        omron_init(RASPI_I2C_CHANNEL, OMRON_1, pi, i2c_bus)

    if omron1_handle < 1:
        if SERVO:
            servo.stop_servo(SERVO_GPIO_PIN)
        pygame.quit()
        sys.exit()

# Open log file

    logfile = open(LOGFILE_NAME, 'wb')
    logfile_open_string = '\r\nStartup log file opened at ' \
                          +str(datetime.now())
    logfile_args_string = '\r\nDEBUG: '+str(DEBUG)+' SERVO: ' \
                          +str(SERVO)+' MONITOR: '+str(MONITOR)+ \
                          ' ROAM: '+str(ROAM)+' RAND: '+str(RAND)
    logfile.write(logfile_open_string)
    logfile.write(logfile_args_string)

    CPUtemp = getCPUtemperature()
    logfile_temp_string = '\r\nInitial CPU Temperature = '+str(CPUtemp)
    logfile.write(logfile_temp_string)
        
    if DEBUG:
        print 'Opening log file: '+LOGFILE_NAME
        print 'CPU temperature = '+str(CPUtemp)

    logfile.write('\r\nPiGPIO version = '+str(version))
    debugPrint('PiGPIO version = '+str(version))

# setup the IR color window
    if MONITOR:
        screen = pygame.display.set_mode(SCREEN_DIMENSIONS)
        pygame.display.set_caption('IR temp array')

# initialize the window quadrant areas for displaying temperature
        pixel_width = SCREEN_DIMENSIONS[0]/4
        px = (pixel_width*3, pixel_width*2, pixel_width, 0)
# using width here to keep an equal square; bottom section for messages
        pixel_height = SCREEN_DIMENSIONS[0]/4
        py = (0, pixel_width, pixel_width*2, pixel_width*3)
        for x in range(0,4):
            for y in range(0,4):
                quadrant[(x*4)+y] = \
                    (px[x], py[y], pixel_width, pixel_height)
                center[(x*4)+y] = \
                    (pixel_width/2+px[x], pixel_height/2+py[y])

    # initialize the location of the message area
        room_temp_area = (0, SCREEN_DIMENSIONS[0], \
                          SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
        room_temp_msg_xy = (SCREEN_DIMENSIONS[0]/2, \
                        (SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

        message_area = (0, SCREEN_DIMENSIONS[0]+SCREEN_DIMENSIONS[0]/4,\
                        SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
        message_area_xy = (SCREEN_DIMENSIONS[0]/2, \
                        (SCREEN_DIMENSIONS[1]/6)+ \
                        (SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

# initialze the music player
    pygame.mixer.init()

    debugPrint('Looking for a person')

    no_person_count = 0
    p_detect = False

# Used to lessen the number of repeat "hello" and "goodbye" messages.
# Once a person is detected, assume they will be there for a short time
# so this is used to wait a while before saying goodbye
                            
    p_detect_count = 0
    person = 0              # initialize the person tracker
    person_existed_last_time = 0
    first_time = 1
    burn_hazard = 0
################################
# initialize the PID controller
################################

# PID controller is the feedback loop controller for person following
    pid_controller=PID(1.0,0.1,0.1)
# seconds to allow temps to settle once the head has moved
    SETTLE_TIME = 1.0
# minimum microseconds if PID error is less than this head will stop
    MINIMUM_ERROR_GRANULARITY = 50
    previous_pid_error = 0

    HELLO_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/20150201_zoe-hello1.mp3"
    AFTER_HELLO_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-sorry.mp3"
    GOODBYE_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/20150201_chloe-goodbye1.mp3"
    BADGE_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-badge1.mp3"
    BURN_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-warning.mp3"
    CPU_105_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-105a.mp3"
    CPU_110_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-110a.mp3"
    CPU_115_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-115a.mp3"
    CPU_120_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-120a.mp3"
    CPU_125_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-125a.mp3"
# the CPU can reach 105 easily, so, normally this is turned off    
    CPU_105_ON = False

##    if CONNECTED:
##    try:
##        speakSpeechFromText("Connected to the Internet!", "intro.mp3")
##        print "Connected to internet"
##        logfile.write('\r\nConnected to the Internet')
##        play_sound(MAX_VOLUME, "intro.mp3")
##        CONNECTED = 1
##    except:
##        print "Not connected to internet"
##        logfile.write('\r\nNOT connected to the Internet')        
##        CONNECTED = 0
        
###########################
# Analyze sensor data
###########################
#
# Sensor data is hard to evaluate. Sometimes there is a weak signal
#     that appears to light up single array cells. In addition, the
#     cells are not a perfect 4x4 array. So it seems that each sensor
#     has a detection area lobe reaching out from the sensor.
#     As a result of these "lobes", there are dead spots inbetween
#     sensors. Also, the lobes are not perfectly symetrical; measured
#     10% offset from an adjacent lobe at 10" away from the sensor.
#     Hot spot of one lobe was off by 1" compared to an adjacent lobe.
#
# In addition, the further away an object is the lower its temperature 
#     Therefore, what temperature threshold is considered a person?
#     The room temp sensor is used as a baseline threshold. Anything
#     below the room temp is considered "background radiation" because
#     if there is no person or heat source, the sensors measure lower
#     than room temp (e.g. room temp = 70F, sensors are around 66F).
#     As a person appears, sensors start measuring above room temp. So,
#     who knows if room temp is a good threshold or not? I add
#     a fudge factor to room temp which requires a person to get closer.
#     Therefore, room temp plus fudge factor results in what I call a
#     "hit".
#
# Now, other complicating factors. A person's clothing will shield
#     temperature, so, the sensors mainly "see" face and hands.
#     A coffee cup, light bulb, candle, or other odd heat source light
#     up one of the sensors and if close enough, can trigger a burn
#     hazard. Therefore, another threshold, over the person temperature
#     which is used to say that this is not a person, it must be a fire.
#     Burn threshold is about 100F.
#
# As a result of this behavior, it is hard to say when a person is there
#     much less, where the person is (to the right or to the left)?
#     Using the raw threshold to say hello or goodbye results in false
#     positives and true negatives.
#
    STATE_NOTHING = 0
    STATE_POSSIBLE = 1
    STATE_LIKELY = 2
    STATE_PROBABLE = 3
    STATE_DETECTED = 4
    personState = STATE_NOTHING
    prevPersonState = STATE_NOTHING

#############################
# Main while loop
#############################
    loop_count = 0
    while True:                 # The main loop
        loop_count += 1
        CPUtemp = getCPUtemperature()
        debugPrint('\r\n*************************** MAIN_WHILE_LOOP: ' \
                   +str(loop_count)+'\r\nPcount: '+str(p_detect_count)+\
                   ' Max: '+"%.1f"%max(temperature_array)+ \
                   ' Servo: '+str(servo_position)+' CPU: '+str(CPUtemp))
# Check for overtemp
        if (CPUtemp >= 105.0):
            if CPU_105_ON:
                play_sound(MAX_VOLUME, CPU_105_FILE_NAME)
#                    debugPrint('Played 105 audio')
        elif (CPUtemp >= 110.0):
            play_sound(MAX_VOLUME, CPU_110_FILE_NAME)
#                debugPrint('Played 110 audio')
        elif (CPUtemp >= 115.0):
            play_sound(MAX_VOLUME, CPU_115_FILE_NAME)
#                debugPrint('Played 115 audio')
        elif (CPUtemp >= 120.0):
            play_sound(MAX_VOLUME, CPU_120_FILE_NAME)
#                debugPrint('Played 120 audio')
        elif (CPUtemp >= 125.0):
            play_sound(MAX_VOLUME, CPU_125_FILE_NAME)
#                debugPrint('Played 125 audio')

# periododically, write the log file to disk
        if loop_count >= LOG_MAX:
            debugPrint('\r\nLoop count max reached (' \
                +str(loop_count)+' at '+str(datetime.now()))
            loop_count = 0      # reset the counter
            debugPrint('\r\nClosing log file at '+str(datetime.now()))
            logfile.close       # for forensic analysis
            logfile = open(LOGFILE_NAME, 'wb')
            debugPrint('\r\nLog file re-opened at '+str(datetime.now()))
            debugPrint(logfile_open_string)
            debugPrint(logfile_args_string)
            debugPrint(logfile_temp_string)
            debugPrint('person temp threshold = ' \
                       +str(PERSON_TEMP_THRESHOLD))
# Display the Omron internal temperature
            debugPrint('Omron D6T internal temp = ' \
                       +"%.1f"%room_temp+' F')
            debugPrint('Servo Type: '+str(SERVO_TYPE))

# start roaming again            
            no_person_count = 0
            p_detect_count  = 0
            roam_count = 0

        while True: # do this loop until a person shows up
         
            time.sleep(MEASUREMENT_WAIT_PERIOD)

            for event in pygame.event.get():
                if event.type == QUIT:
                    crash_msg = '\r\npygame event QUIT'
                    crash_and_burn(crash_msg, pygame, servo, logfile)
                if event.type == KEYDOWN:
                    if event.key == K_q or event.key == K_ESCAPE:
                        crash_msg = \
                        '\r\npygame event: keyboard q or esc pressed'
                        crash_and_burn(crash_msg, pygame, \
                                       servo, logfile)
                    if event.key == (KMOD_LCTRL | K_c):
                        crash_msg = \
                        '\r\npygame event: keyboard ^c pressed'
                        crash_and_burn(crash_msg, pygame, \
                                       servo, logfile)

# read the raw temperature data
# 
            (bytes_read, temperature_array, room_temp) = \
                omron_read(omron1_handle, DEGREE_UNIT, \
                OMRON_BUFFER_LENGTH, pi)
            omron_read_count += 1
         
# Display each element's temperature in F
#            debugPrint('New temperature measurement')
            print_temps(temperature_array)

            if bytes_read != OMRON_BUFFER_LENGTH: # sensor problem
                omron_error_count += 1
                debugPrint( \
                    'ERROR: Omron thermal sensor failure! Bytes read: '\
                    +str(bytes_read))
                fatal_error = 1
                break

            if MONITOR:
# create the IR pixels
                for i in range(0,OMRON_DATA_LIST):
# This fills each array square with a color that matches the temp
                    screen.fill(fahrenheit_to_rgb(MAX_TEMP, MIN_TEMP, \
                                temperature_array[i]), quadrant[i])
# Display temp value
                    if temperature_array[i] > PERSON_TEMP_THRESHOLD:
                        text = font.render("%.1f"%temperature_array[i],\
                                           1, name_to_rgb('red'))
                    else:
                        text = font.render("%.1f"%temperature_array[i],\
                                           1, name_to_rgb('navy'))
                    textpos = text.get_rect()
                    textpos.center = center[i]
                    screen.blit(text, textpos)

# Create an area to display the room temp and messages
                screen.fill(fahrenheit_to_rgb(MAX_TEMP, MIN_TEMP, \
                            room_temp), room_temp_area)
                text = font.render("Room: %.1f"%room_temp, 1, \
                                    name_to_rgb('navy'))
                textpos = text.get_rect()
                textpos.center = room_temp_msg_xy
                screen.blit(text, textpos)

# update the screen
                pygame.display.update()

###########################
# Analyze sensor data
###########################

            previousHitCnt = hitCnt
            hit_array, hitCnt = get_hit_array(room_temp, \
                    temperature_array, servo_position)

###########################
# Burn Hazard Detected !
###########################
            if max(temperature_array) > BURN_HAZARD_TEMP:
                roam_count = 0
                burn_hazard = 1
                LED_state = True
                GPIO.output(LED_GPIO_PIN, LED_state)
                if MONITOR:
                    screen.fill(name_to_rgb('red'), message_area)
                    text = font.render("WARNING! Burn danger!", 1, \
                            name_to_rgb('yellow'))
                    textpos = text.get_rect()
                    textpos.center = message_area_xy
                    screen.blit(text, textpos)
# update the screen
                    pygame.display.update()

                play_sound(MAX_VOLUME, BURN_FILE_NAME)
                debugPrint('Played Burn warning audio')
                if CONNECTED:
                    try:
                        speakSpeechFromText("The temperature is "+ \
                            "%.1f"%max(temperature_array)+ \
                            " degrees fahrenheit", "mtemp.mp3")
                        play_sound(MAX_VOLUME, "mtemp.mp3")
                    except:
                        continue
                debugPrint('\r\n'+"Burn hazard temperature is " \
                           +"%.1f"%max(temperature_array)+" degrees")
                
                break

###########################
# No Person Detected
###########################
# State 0: NOTHING - no heat source in view
#     Event 0: No change - outcome: continue waiting for a person
#     Event 1: One or more sensors cross the person threshold
#
            elif (personState == STATE_NOTHING):
                debugPrint('STATE: NOTHING: No Person cnt: ' \
                           +str(no_person_count)+' ROAM_COUNT = ' \
                           +str(roam_count))
                no_person_count += 1
                p_detect_count = 0
                person = 0
                burn_hazard = 0
                LED_state = False
                GPIO.output(LED_GPIO_PIN, LED_state)
                if MONITOR:
                    screen.fill(name_to_rgb('white'), message_area)
                    text = font.render("Waiting...", 1, \
                                       name_to_rgb('blue'))
                    textpos = text.get_rect()
                    textpos.center = message_area_xy
                    screen.blit(text, textpos)
    # update the screen
                    pygame.display.update()
                if (hitCnt == 0 or previousHitCnt == 0):
                    personState = STATE_NOTHING
                else:
                    personState = STATE_POSSIBLE

    # put servo in roaming mode

                roam_count += 1
                if SERVO and (ROAM or RAND) and roam_count <= ROAM_MAX:
                    if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                        if (servo_position <= SERVO_LIMIT_CCW and \
                            servo_direction == SERVO_CUR_DIR_CCW):
                            debugPrint('CCW limit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                    else:
                        if (servo_position >= SERVO_LIMIT_CCW and \
                            servo_direction == SERVO_CUR_DIR_CCW):
                            debugPrint('CCW limit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                        
                    if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                        if (servo_position >= SERVO_LIMIT_CW and \
                            servo_direction == SERVO_CUR_DIR_CW):
                            debugPrint('CW limit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CCW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                    else:
                        if (servo_position <= SERVO_LIMIT_CW and \
                            servo_direction == SERVO_CUR_DIR_CW):
                            debugPrint('CW limit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CCW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                        
                    if RAND:
                        debugPrint('SERVO_RAND Pos: ' \
                                   +str(servo_position)+' Dir: ' \
                                   +str(servo_direction))
                        servo_position = \
                            random.randint(MAX_SERVO_POSITION, \
                                           MIN_SERVO_POSITION)
                    else:
                        if servo_direction == SERVO_CUR_DIR_CCW:
                            debugPrint('SERVO_ROAM Pos: '+ \
                                str(servo_position)+' Direction: CCW')
                            if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                                servo_position -= ROAMING_GRANULARTY
                            else:
                                servo_position += ROAMING_GRANULARTY
                        if servo_direction == SERVO_CUR_DIR_CW:
                            debugPrint('SERVO_ROAM Pos: '+ \
                                str(servo_position)+' Direction: CW')
                            if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                                servo_position += ROAMING_GRANULARTY
                            else:
                                servo_position -= ROAMING_GRANULARTY
                      
                    servo_position = \
                        set_servo_to_position(servo_position)

                else:
                    if (LED_state == False):
                        LED_state = True
#                        debugPrint('Turning LED on')
                        GPIO.output(LED_GPIO_PIN, LED_state)
                    else:
                        LED_state = False
#                        debugPrint('Turning LED off')
                        GPIO.output(LED_GPIO_PIN, LED_state)
                    time.sleep(0.5)
# Start roaming again if no action
                    if SERVO and ROAM and roam_count >= ROAM_MAX*2:
                        roam_count = 0

                prevPersonState = STATE_NOTHING
                    
###########################
# Possible Person Detected
###########################
# State 1: Possible person in view - one or more sensors had a hit
#     Event 0: No hits - blip, move to State 0
#     Event 1: One hit - move head to try to center on the hit
#     Event 2: More than one hit - state 2
#
            elif (personState == STATE_POSSIBLE):
                debugPrint('STATE: POSSIBLE: No Person cnt: ' \
                           +str(no_person_count))
                no_person_count += 1
                if (hitCnt == 0 or previousHitCnt == 0):
                    personState = STATE_NOTHING
                elif (hitCnt == 1 and previousHitCnt >= 1):
                    p_detect, p_pos = person_position_1_hit(room_temp, \
                        temperature_array, servo_position)
#                    servo_position = moveHead(p_pos, servo_position)
#                    personState = STATE_POSSIBLE
                else:
                    personState = STATE_LIKELY

                prevPersonState = STATE_POSSIBLE
                
###########################
# Likely Person Detected
###########################
# State 2: Likely person in view - more than one sensor had a hit
#     Event 0: No hits - blip, move to State 1
#     Event 1: One hit - noise, no change
#     Event 2: more than one sensor still has a hit, move head, State 3
#
            elif (personState == STATE_LIKELY):
                debugPrint('STATE: LIKELY: No Person cnt: ' \
                            +str(no_person_count))
                no_person_count += 1
                if (hitCnt == 0 or previousHitCnt == 0):
                    personState = STATE_POSSIBLE
                elif (hitCnt == 1 and previousHitCnt >= 1):
                    p_detect, p_pos = person_position_1_hit(room_temp, \
                        temperature_array, servo_position)
                    servo_position = moveHead(p_pos, servo_position)
#                    personState = STATE_LIKELY
                else:
                    p_detect, p_pos = person_position_2_hit(room_temp, \
                        temperature_array, servo_position)
                    servo_position = moveHead(p_pos, servo_position)
                    personState = STATE_PROBABLE

                prevPersonState = STATE_LIKELY

###########################
# Probable Person Detected
###########################
# State 3: Probably a person in view
#     Event 0: No hits - noise, move to State 2
#     Event 1: One hit - noise, move to state 2
#     Event 2: more than one sensor has a hit, move head, say hello
#
            elif (personState == STATE_PROBABLE):
                debugPrint('STATE: PROBABLE: No Person cnt: ' \
                            +str(no_person_count))
                if (hitCnt == 0 or previousHitCnt == 0):
                    personState = STATE_LIKELY
                elif (hitCnt == 1 and previousHitCnt >= 1):
                    p_detect, p_pos = person_position_1_hit(room_temp, \
                        temperature_array, servo_position)
                    servo_position = moveHead(p_pos, servo_position)
#                    personState = STATE_LIKELY
                else:
                    p_detect, p_pos = person_position_2_hit(room_temp, \
                        temperature_array, servo_position)
                    servo_position = moveHead(p_pos, servo_position)
                    sayHello()
                    personState = STATE_DETECTED

                prevPersonState = STATE_PROBABLE

###########################
# Person Detected !
###########################
# State 4: Person detected
#     Event 0: No hits - person left, say goodbye, move to state 0
#     Event 1: One hit - person left, say goodbye, move to state 1
#     Event 2: more than one sensor, move head to position, stay
#     
            elif (personState == STATE_DETECTED):
                debugPrint('STATE: DETECTED: No Person cnt: ' \
                           +str(no_person_count))
                roam_count = 0
                no_person_count = 0
                burn_hazard = 0
                LED_state = True
                GPIO.output(LED_GPIO_PIN, LED_state)
                p_detect_count += 1
                CPUtemp = getCPUtemperature()
                debugPrint('Person_count: '+str(p_detect_count)+ \
                           ' Max: '+"%.1f"%max(temperature_array)+ \
                           ' Servo: '+str(servo_position)+' CPU: ' \
                           +str(CPUtemp))
                if (hitCnt == 0 or previousHitCnt == 0):
                    sayGoodBye()
                    personState = STATE_PROBABLE
                elif (hitCnt == 1 and previousHitCnt >= 1):
                    p_detect, p_pos = person_position_1_hit(room_temp, \
                        temperature_array, servo_position)
                    servo_position = moveHead(p_pos, servo_position)
#                    sayGoodBye()
#                    personState = STATE_NOTHING
                else:
                    p_detect, p_pos = person_position_2_hit(room_temp, \
                        temperature_array, servo_position)
                    servo_position = moveHead(p_pos, servo_position)
#                    personState = STATE_DETECTED

                prevPersonState = STATE_DETECTED

###########################
# Invalid state
###########################
            else:
                personState = STATE_NOTHING
                
# End of inner While loop
            break

        if fatal_error:
            logfile.write('\r\nFatal error at '+str(datetime.now()))
            break

#############################
# End main while loop
#############################

except KeyboardInterrupt:
    crash_msg = '\r\nKeyboard interrupt; quitting'
    crash_and_burn(crash_msg, pygame, servo, logfile)

except IOError:
    # do not close the logfile here
    # allows the previous logfile to stay intact for a forensic analysis
    crash_msg = '\r\nI/O Error; quitting'
    debugPrint(crash_msg)
    if SERVO:
        servo.stop_servo(SERVO_GPIO_PIN)
    pygame.quit()
    sys.exit()


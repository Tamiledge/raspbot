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
# sudo python /home/pi/projects_ggg/raspbot/raspbot.py -nomonitor -roam
# >> /home/pi/projects_ggg/raspbot/raspbot_python.log 2>&1 &
# The above command will route all regular and error messages to a log file
# The log file is appended each time, so it can get large and should be
# deleted every once in a while.
#
# There is another log file created by this program when running
# idependently is created in the raspbot directory about every
# five minutes the log file is closed and reopened. This is probably
# redundant but it contains more debug info than the python log.
#
# !!!!!!!!!!!!!!!!!
# remember to run this as root "sudo ./raspbot -debug -roam" so that
# DMA can be used for the servo and the GPIO pins can be used
# !!!!!!!!!!!!!!!!!

# Jan 2015
"""
import smbus
import sys
#import getopt
import pigpio
import time
from datetime import datetime
from webcolors import name_to_rgb
import pygame
from pygame.locals import Rect, QUIT, KEYDOWN, K_q, K_ESCAPE
import random
from omron_src import omron_init    # contains omron functions
from omron_src import omron_read    # contains omron functions
#import urllib, pycurl, os           # needed for text to speech
from pid import PID
from raspbot_functions import getCPUtemperature, fahrenheit_to_rgb, speakSpeechFromText

# GPIO assignments for the hit LEDs (three colors, red, yellow, green)
#   red = burn hazard (hit_array[x] > 4
#   yellow = possible person (hit_array[x] == 1
#   green = person probable (hit_array[x] >=2 <= 4)
#   dark = no hit detected (hit_array[x] = 0)
#
#   LEDs are connected to ground on one pin and the other three pins
#       (RGY) take +3v through a 1k resistor. The LEDs are LUMEX
#       SSL-LX5097 or DigiKey 67-2184-ND
#
#   LED0 (_RED, _YEL, _GRN) = hit_array[0] 
#   LED1 (_RED, _YEL, _GRN) = hit_array[1] 
#   LED2 (_RED, _YEL, _GRN) = hit_array[2] 
#   LED3 (_RED, _YEL, _GRN) = hit_array[3] 
LED0_RED = 11   # AKA: BCM GPIO 17
LED0_YEL = 12   # AKA: BCM GPIO 18
LED0_GRN = 13   # AKA: BCM GPIO 27

LED1_RED = 15   # AKA: BCM GPIO 22
LED1_YEL = 16   # AKA: BCM GPIO 23
LED1_GRN = 18   # AKA: BCM GPIO 24

LED2_RED = 22   # AKA: BCM GPIO 25
LED2_YEL = 29   # AKA: BCM GPIO 5
LED2_GRN = 31   # AKA: BCM GPIO 6

LED3_RED = 32   # AKA: BCM GPIO 12
LED3_YEL = 35   # AKA: BCM GPIO 19
LED3_GRN = 36   # AKA: BCM GPIO 16

LED_ON = True
LED_OFF = False

def get_uptime():
    f = open("/proc/uptime", "r");
    t = float(f.read().split()[0])
    f.close()
    return t

def debug_print(message):
    """
    Debug messages are printed to display and log file using this
    """
    now_string = str(datetime.now())
    if DEBUG and MONITOR:
        print now_string+': '+message
    LOGFILE_HANDLE.write('\r\n'+now_string+': '+message)
    
def print_temps(temp_list):
    """
    Display each element's temperature in F
    """
    debug_print("%.1f"%temp_list[12]+' '+"%.1f"%temp_list[8]+ \
               ' '+"%.1f"%temp_list[4]+' '+"%.1f"%temp_list[0]+' ')
    debug_print("%.1f"%temp_list[13]+' '+"%.1f"%temp_list[9]+ \
               ' '+"%.1f"%temp_list[5]+' '+"%.1f"%temp_list[1]+' ')
    debug_print("%.1f"%temp_list[14]+' '+"%.1f"%temp_list[10]+ \
               ' '+"%.1f"%temp_list[6]+' '+"%.1f"%temp_list[2]+' ')
    debug_print("%.1f"%temp_list[15]+' '+"%.1f"%temp_list[11]+ \
               ' '+"%.1f"%temp_list[7]+' '+"%.1f"%temp_list[3]+' ')

def set_servo_to_position (new_position):
    """
    Moves the servo to a new position
    """

    if SERVO_ENABLED:
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

        # if there is a remainder, make 10us increments
        if (new_position%MINIMUM_SERVO_GRANULARITY < 5):
            final_position = \
            (new_position//MINIMUM_SERVO_GRANULARITY) \
            *MINIMUM_SERVO_GRANULARITY
        else:
            final_position = \
            ((new_position//MINIMUM_SERVO_GRANULARITY)+1) \
            *MINIMUM_SERVO_GRANULARITY

        debug_print('SERVO_MOVE: '+str(final_position))
        SERVO_HANDLE.set_servo(SERVO_GPIO_PIN, final_position)
           
        return final_position

# all the servo constants
LOW_TO_HIGH_IS_COUNTERCLOCKWISE = 0
LOW_TO_HIGH_IS_CLOCKWISE = 1
HITEC_HS55 = LOW_TO_HIGH_IS_CLOCKWISE
SERVO_TYPE = HITEC_HS55
CTR_SERVO_POSITION = 1500
MINIMUM_SERVO_GRANULARITY = 10  # microseconds
SERVO_CUR_DIR_CW = 1            # Direction to move the servo next
SERVO_CUR_DIR_CCW = 2
ROAMING_GRANULARTY = 50
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
else:
    MIN_SERVO_POSITION = 600
    MAX_SERVO_POSITION = 2300
    SERVO_LIMIT_CW = MAX_SERVO_POSITION
    SERVO_LIMIT_CCW = MIN_SERVO_POSITION

MOVE_DIST_CLOSE = 30     
MOVE_DIST_SHORT = 100      
MOVE_DIST_MEDIUM = 170       
MOVE_DIST_FAR = 240       

def resolve_new_position(move_cw, servo_pos, move_distance):
        if (move_cw):
            if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                new_position = servo_pos + move_distance
            else:
                new_position = servo_pos - move_distance
        else:
            if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                new_position = servo_pos - move_distance
            else:
                new_position = servo_pos + move_distance

        return new_position
    
def person_position_1_hit(hit_array_1, s_position):
    """
    Detect a persons presence using "greater than one algorithm"
    returns (TRUE if person detected, approximate person position)
    """
    person_det_1 = True
    person_pos_1 = s_position
    move_dist_1 = 0
    move_cw_1 = True

    if (hit_array_1[1] >= 1 and hit_array_1[2] >= 1):
        # person is centered
        move_dist_1 = 0
    elif (hit_array_1[0] == 0 and hit_array_1[1] == 0 and \
          hit_array_1[2] == 0 and hit_array_1[3] >= 1):
        move_dist_1 = MOVE_DIST_FAR
        move_cw_1 = False
    elif (hit_array_1[0] == 0 and hit_array_1[1] == 0 and \
          hit_array_1[2] >= 1 and hit_array_1[3] == 0):
        move_dist_1 = MOVE_DIST_SHORT
        move_cw_1 = False
    elif (hit_array_1[0] == 0 and hit_array_1[1] >= 1 and \
          hit_array_1[2] == 0 and hit_array_1[3] == 0):
        move_dist_1 = MOVE_DIST_SHORT
        move_cw_1 = True
    elif (hit_array_1[0] >= 1 and hit_array_1[1] == 0 and \
          hit_array_1[2] == 0 and hit_array_1[3] == 0):
        move_dist_1 = MOVE_DIST_FAR
        move_cw_1 = True
    elif (hit_array_1[0] == 0 and hit_array_1[1] == 0 and \
          hit_array_1[2] >= 1 and hit_array_1[3] >= 1):
        move_dist_1 = MOVE_DIST_MEDIUM
        move_cw_1 = False
    elif (hit_array_1[0] >= 1 and hit_array_1[1] >= 1 and \
          hit_array_1[2] == 0 and hit_array_1[3] == 0):
        move_dist_1 = MOVE_DIST_MEDIUM
        move_cw_1 = True
    elif (hit_array_1[0] == 0 and hit_array_1[1] >= 1 and \
          hit_array_1[2] >= 1 and hit_array_1[3] >= 1):
        move_dist_1 = MOVE_DIST_CLOSE
        move_cw_1 = False
    elif (hit_array_1[0] >= 1 and hit_array_1[1] >= 1 and \
          hit_array_1[2] >= 1 and hit_array_1[3] == 0):
        move_dist_1 = MOVE_DIST_CLOSE
        move_cw_1 = True
    else:
        # no person detected
        person_det_1 = False

    if (move_dist_1 > 0):
        person_pos_1 = resolve_new_position(move_cw_1, s_position, move_dist_1)

    debug_print('person_position_1: Pos: '+str(person_pos_1)+ \
                ' Det: '+str(person_det_1))

    return (person_det_1, person_pos_1)

def person_position_2_hit(hit_array_2, s_position):
    """
    Detect a persons presence using the "greater than two algorithm"
    returns (TRUE if person detected, approximate person position)
    """
    person_det_2 = True
    person_pos_2 = s_position
    move_dist_2 = 0
    move_cw_2 = True

# First, look for > two hits in a single column
    if (hit_array_2[1] >= 2 and hit_array_2[2] >= 2):
        # person already in center
        move_dist_2 = 0
    elif (hit_array_2[0] >= 2 and hit_array_2[1] <= 1 and \
          hit_array_2[2] <= 1 and hit_array_2[3] <= 1):
        move_dist_2 = MOVE_DIST_FAR
        move_cw_2 = True
# Sometimes a stationary person can show up 0200 and 0020 alternatively
# without moving causing the robot to oscillate
##    elif (hit_array_2[0] <= 1 and hit_array_2[1] >= 2 and \
##          hit_array_2[2] <= 1 and hit_array_2[3] <= 1):
##        move_dist_2 = MOVE_DIST_SHORT
##        move_cw_2 = True
##    elif (hit_array_2[0] <= 1 and hit_array_2[1] <= 1 and \
##          hit_array_2[2] >= 2 and hit_array_2[3] <= 1):
##        move_dist_2 = MOVE_DIST_SHORT
##        move_cw_2 = False
    elif (hit_array_2[0] <= 1 and hit_array_2[1] <= 1 and \
          hit_array_2[2] <= 1 and hit_array_2[3] >= 2):
        move_dist_2 = MOVE_DIST_FAR
        move_cw_2 = False
    elif (hit_array_2[0] >= 2 and hit_array_2[1] >= 2 and \
          hit_array_2[2] <= 1 and hit_array_2[3] <= 1):
        move_dist_2 = MOVE_DIST_CLOSE
        move_cw_2 = True
    elif (hit_array_2[0] <= 1 and hit_array_2[1] <= 1 and \
          hit_array_2[2] >= 2 and hit_array_2[3] >= 2):
        move_dist_2 = MOVE_DIST_CLOSE
        move_cw_2 = False
    else:
        # no person detected
        person_det_2 = False

    if (move_dist_2 > 0):
        person_pos_2 = resolve_new_position(move_cw_2, s_position, move_dist_2)

    debug_print('person_position_2: Pos: '+str(person_pos_2)+ \
                ' Det: '+str(person_det_2))

    return (person_det_2, person_pos_2)

def move_head(position, servo_pos):
    """
    Move the robot head to a specific position
    """
    if SERVO_ENABLED:
# face the servo twoards the heat
        # setpoint is the desired position
        PID_CONTROLLER.setPoint(position)
        # process variable is current position
        pid_error = PID_CONTROLLER.update(servo_pos)
        debug_print('Des Pos: '+str(position)+ \
                   ' Cur Pos: '+str(servo_pos)+ \
                   ' PID Error: '+str(pid_error))

# make the robot turn its head to the person
# if previous error is the same absolute value as the current error,
# then we are oscillating - stop it
        if abs(pid_error) > MINIMUM_ERROR_GRANULARITY:
            if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                servo_pos += pid_error
            else:
                servo_pos -= pid_error
                           
        new_servo_pos = set_servo_to_position(servo_pos)

        #let the temp's settle
        time.sleep(MEASUREMENT_WAIT_PERIOD*SETTLE_TIME)

        return new_servo_pos

LAST_KNOWN_LED_POS = 0  # counter keeps track of which LED to light
LED_POS_MAX = 4
LIT_LED = LED0_RED
LED0_COLOR_SET = [LED0_RED, LED0_YEL, LED0_GRN]
LED1_COLOR_SET = [LED1_RED, LED1_YEL, LED1_GRN]
LED2_COLOR_SET = [LED2_RED, LED2_YEL, LED2_GRN]
LED3_COLOR_SET = [LED3_RED, LED3_YEL, LED3_GRN]
def servo_roam(roam_cnt, servo_pos, servo_dir, last_led, lit):
    """
    Puts the servo in roaming (person searching) mode
    """
    roam_cnt += 1
    GPIO.output(lit, LED_OFF)
    debug_print('Roam count = '+str(roam_cnt))

    if roam_cnt <= ROAM_MAX:
        
        # determine next servo direction
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            if (servo_pos <= SERVO_LIMIT_CCW and \
                servo_dir == SERVO_CUR_DIR_CCW):
#                debug_print('CCW -> CW')
                servo_dir = SERVO_CUR_DIR_CW
            elif (servo_pos >= SERVO_LIMIT_CW and \
                servo_dir == SERVO_CUR_DIR_CW):
#                debug_print('CW -> CCW')
                servo_dir = SERVO_CUR_DIR_CCW
        else:
            if (servo_pos >= SERVO_LIMIT_CCW and \
                servo_dir == SERVO_CUR_DIR_CCW):
#                debug_print('CCW -> CW')
                servo_dir = SERVO_CUR_DIR_CW
            elif (servo_pos <= SERVO_LIMIT_CW and \
                servo_dir == SERVO_CUR_DIR_CW):
#                debug_print('CW -> CCW')
                servo_dir = SERVO_CUR_DIR_CCW

        # determine next servo position    
        if RAND:
            debug_print('SERVO_RAND Pos: ' \
                       +str(servo_pos)+' Dir: ' \
                       +str(servo_dir))
            servo_pos = \
                random.randint(MAX_SERVO_POSITION, \
                               MIN_SERVO_POSITION)

        elif ROAM:
            if servo_dir == SERVO_CUR_DIR_CCW:
                debug_print('SERVO_ROAM Pos: '+ \
                    str(servo_pos)+' Direction: CCW')
                if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                    servo_pos -= ROAMING_GRANULARTY
                else:
                    servo_pos += ROAMING_GRANULARTY
            if servo_dir == SERVO_CUR_DIR_CW:
                debug_print('SERVO_ROAM Pos: '+ \
                    str(servo_pos)+' Direction: CW')
                if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                    servo_pos += ROAMING_GRANULARTY
                else:
                    servo_pos -= ROAMING_GRANULARTY

        servo_pos = \
            set_servo_to_position(servo_pos)

        # randomly select a color and light it up
        if (last_led == 0):
            lit = LED0_YEL
        elif (last_led == 1):
            lit = LED1_YEL
        elif (last_led == 2):
            lit = LED2_YEL
        else:
            lit = LED3_YEL

        GPIO.output(lit, LED_ON)

        if (servo_dir == SERVO_CUR_DIR_CW):
            last_led -= 1
            if (last_led < 0):
                last_led = LED_POS_MAX-1
        else:
            last_led += 1
            if (last_led >= LED_POS_MAX):
                last_led = 0

        debug_print('last LED = '+str(last_led)+' lit LED = '+str(lit))

    else:
# center the servo when roam max is hit
        servo_pos = \
            set_servo_to_position(CTR_SERVO_POSITION)

        time.sleep(0.3)
        if SERVO_ENABLED:
            SERVO_HANDLE.stop_servo(SERVO_GPIO_PIN)

        GPIO.output(LED0_RED, LED_OFF)
        GPIO.output(LED1_RED, LED_OFF)
        GPIO.output(LED2_RED, LED_OFF)
        GPIO.output(LED3_RED, LED_OFF)
        GPIO.output(LED0_YEL, LED_OFF)
        GPIO.output(LED1_YEL, LED_OFF)
        GPIO.output(LED2_YEL, LED_OFF)
        GPIO.output(LED3_YEL, LED_OFF)
        GPIO.output(LED0_GRN, LED_OFF)
        GPIO.output(LED1_GRN, LED_OFF)
        GPIO.output(LED2_GRN, LED_OFF)
        GPIO.output(LED3_GRN, LED_OFF)

# Start roaming again if no action
        if roam_cnt >= ROAM_MAX*20:
            roam_cnt = 0
            GPIO.output(LED0_GRN, LED_OFF)
            GPIO.output(LED1_GRN, LED_OFF)
            GPIO.output(LED2_GRN, LED_OFF)
            GPIO.output(LED3_GRN, LED_OFF)

    return roam_cnt, servo_pos, servo_dir, last_led, lit

def say_hello():
    """
    Causes the robot to say hello
    """
    if MONITOR:
        SCREEN_DISPLAY.fill(name_to_rgb('white'), MESSAGE_AREA)
        txt = FONT.render("Hello!", 1, name_to_rgb('red'))
        txtpos = SCREEN_TEXT.get_rect()
        txtpos.center = MESSAGE_AREA_XY
        SCREEN_DISPLAY.blit(txt, txtpos)
# update the screen
        pygame.display.update()

    debug_print('\r\n**************************\r\n     Hello Person!\r\n**************************')

# Play "hello" sound effect
    debug_print('Playing hello audio')
    play_sound(MAX_VOLUME, HELLO_FILE_NAME)
#    time.sleep(1)
#    play_sound(MAX_VOLUME, AFTER_HELLO_FILE_NAME)
#    debug_print('Played after hello audio')

def say_goodbye():
    """
    Causes the robot to say good bye
    """
    if MONITOR:
        SCREEN_DISPLAY.fill(name_to_rgb('white'), MESSAGE_AREA)
        txt = FONT.render("Good Bye!", 1, name_to_rgb('red'))
        txtpos = SCREEN_TEXT.get_rect()
        txtpos.center = MESSAGE_AREA_XY
        SCREEN_DISPLAY.blit(txt, txtpos)
# update the screen
        pygame.display.update()

    debug_print('\r\n**************************\r\n      Goodbye Person!\r\n**************************')

# Play "bye bye" sound effect
    #byebye_message = random.choice(BYEBYE_FILE_NAME)
    debug_print('Playing badge audio')
    play_sound(MAX_VOLUME, BADGE_FILE_NAME)

    debug_print('Playing good bye audio')
    play_sound(MAX_VOLUME, GOODBYE_FILE_NAME)


def play_sound(volume, message):
    """
    Play an mp3 file
    """
# commented next line thinking that it might be causing the garbling
    # pygame.mixer.music.set_volume(volume)         
    pygame.mixer.music.load(message)
    pygame.mixer.music.play()
# this is not causing the garbling
    while pygame.mixer.music.get_busy() == True:
        continue

def crash_and_burn(msg, py_game, servo_in, log_file):
    """
    Something bad happend; quit the program
    """
# doing a print here makes sure that the stdout gets a message
    print(msg)
    debug_print(msg)
    if SERVO_ENABLED:
        servo_in.stop_servo(SERVO_GPIO_PIN)
    GPIO.output(LED_GPIO_PIN, LED_OFF)
    GPIO.output(LED0_RED, LED_OFF)
    GPIO.output(LED0_YEL, LED_OFF)
    GPIO.output(LED0_GRN, LED_OFF)
    GPIO.output(LED1_RED, LED_OFF)
    GPIO.output(LED1_YEL, LED_OFF)
    GPIO.output(LED1_GRN, LED_OFF)
    GPIO.output(LED2_RED, LED_OFF)
    GPIO.output(LED2_YEL, LED_OFF)
    GPIO.output(LED2_GRN, LED_OFF)
    GPIO.output(LED3_RED, LED_OFF)
    GPIO.output(LED3_YEL, LED_OFF)
    GPIO.output(LED3_GRN, LED_OFF)
    py_game.quit()
    PWM.cleanup()
    log_file.write(msg+' @ '+str(datetime.now()))
    log_file.close
    sys.exit()

def panic():
# doing a print here makes sure that the stdout gets a message
    print('panic!')
    debug_print('Panic!')
    GPIO.output(LED0_GRN, LED_OFF)
    GPIO.output(LED1_GRN, LED_OFF)
    GPIO.output(LED2_GRN, LED_OFF)
    GPIO.output(LED3_GRN, LED_OFF)
    GPIO.output(LED0_YEL, LED_OFF)
    GPIO.output(LED1_YEL, LED_OFF)
    GPIO.output(LED2_YEL, LED_OFF)
    GPIO.output(LED3_YEL, LED_OFF)
    if SERVO_ENABLED:
        SERVO_HANDLE.stop_servo(SERVO_GPIO_PIN)
    while True:
        GPIO.output(LED0_RED, LED_ON)
        GPIO.output(LED1_RED, LED_ON)
        GPIO.output(LED2_RED, LED_ON)
        GPIO.output(LED3_RED, LED_ON)
        time.sleep(0.3)
        GPIO.output(LED0_RED, LED_OFF)
        GPIO.output(LED1_RED, LED_OFF)
        GPIO.output(LED2_RED, LED_OFF)
        GPIO.output(LED3_RED, LED_OFF)
        time.sleep(0.3)
        for event in pygame.event.get():
            if event.type == QUIT:
                CRASH_MSG = '\r\npygame event QUIT'
                crash_and_burn(CRASH_MSG, pygame, SERVO_HANDLE, \
                               LOGFILE_HANDLE)
            if event.type == KEYDOWN:
                if event.key == K_q or event.key == K_ESCAPE:
                    CRASH_MSG = \
                    '\r\npygame event: keyboard q or esc pressed'
                    crash_and_burn(CRASH_MSG, pygame, \
                                   SERVO_HANDLE, LOGFILE_HANDLE)


# Constants
RASPI_I2C_CHANNEL = 1       # the /dev/i2c device
OMRON_1 = 0x0a              # 7 bit I2C address of Omron Sensor D6T-44L
OMRON_BUFFER_LENGTH = 35    # Omron data buffer size
OMRON_DATA_LIST = 16        # Omron data array - sixteen 16 bit words
MAX_VOLUME = 1.0            # maximum speaker volume for pygame.mixer
DEGREE_UNIT = 'F'           # F = Farenheit, C=Celcius
MEASUREMENT_WAIT_PERIOD = 0.3   # time between Omron measurements
SERVO_ENABLED = 1   # set this to 1 if the servo motor is wired up
SERVO_GPIO_PIN = 11 # GPIO number (GPIO 11 aka. SCLK)
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
BURN_HAZARD_CNT = 0     # number of times burn hazard detected
BURN_HAZARD_HIT = 10    # Number used in Hit array to indicate hazard
TEMPMARGIN = 5          # degrees > than room temp to detect person
PERSON_TEMP_THRESHOLD = 79  # degrees fahrenheit
MONITOR = 1             # assume a monitor is attached
# Servo positions
# Weirdness factor: Some servo's I used go in the reverse direction
# from other servos. Therefore, this next constant is used to change the
# software to use the appropriate servo. The HiTEC HS-55 Feather servo.

HIT_WEIGHT_PERCENT = 0.1
PERSON_TEMP_SUM_THRESHOLD = 3
DETECT_COUNT_THRESH = 3
PERSON_HIT_COUNT = 4
PROBABLE_PERSON_THRESH = 3  # used to determine when to say hello

# Logfile
LOGFILE_NAME = "/home/pi/projects_ggg/raspbot/raspbot.log"

import RPi.GPIO as GPIO
GPIO.setwarnings(False) # turn off warnings about DMA channel in use
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
GPIO.setup(LED_GPIO_PIN, GPIO.OUT)
from RPIO import PWM        # for the servo motor
PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS) # turn off debug msgs

CONNECTED = 0           # true if connected to the internet

###############################
#
# Start of main line program
#
###############################

# Handle command line arguments
if "-debug" in sys.argv:
    DEBUG = 1         # set this to 1 to see debug messages on monitor

if "-noservo" in sys.argv:
    SERVO_ENABLED = 0         # assume using servo is default

if "-nomonitor" in sys.argv:
    MONITOR = 0       # assume using servo is default

if "-roam" in sys.argv:
    ROAM = 1          # set this to 1 to roam looking for a person

if "-rand" in sys.argv:
    RAND = 1          # set this to 1 to randomize looking for a person

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
TEMPERATURE_ARRAY = [0.0]*OMRON_DATA_LIST
LED_STATE = True
# keep track of head roams so that we can turn it off
ROAM_COUNT = 0
# QUADRANT of the display (x, y, width, height)
QUADRANT = [Rect]*OMRON_DATA_LIST
CENTER = [(0, 0)]*OMRON_DATA_LIST      # center of each QUADRANT
PX = [0]*4
PY = [0]*4
OMRON_ERROR_COUNT = 0
OMRON_READ_COUNT = 0
PREVIOUS_HIT_COUNT = 0
HIT_COUNT = 0
HIT_ARRAY_TEMP = [0]*OMRON_DATA_LIST
HIT_ARRAY = [0]*4
# initialize the servo to face directly forward
SERVO_POSITION = CTR_SERVO_POSITION
# set initial direction
if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
    SERVO_DIRECTION = SERVO_CUR_DIR_CW
else:
    SERVO_DIRECTION = SERVO_CUR_DIR_CCW

# Initialize screen
pygame.init()
FONT = pygame.font.Font(None, 36)

try:
# Initialize i2c bus address
    I2C_BUS = smbus.SMBus(1)
    time.sleep(0.1)                # Wait

# make some space
    print ''

    if SERVO_ENABLED:
# Initialize servo position
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
        SERVO_HANDLE = PWM.Servo()
        SERVO_HANDLE.set_servo(SERVO_GPIO_PIN, CTR_SERVO_POSITION)
        print('')
    else:
        print('SERVO is off')

    print ''
    if DEBUG:
        print('DEBUG switch is on, initializing Pigpio...')

# intialize pigpio library and socket connection for daemon (pigpiod)
    PIGPIO_HANDLE = pigpio.pi()              # use defaults
    PIGPIO_VERSION = PIGPIO_HANDLE.get_pigpio_version()

# initialize LEDs
    LED_STATE = True
    GPIO.setup(LED_GPIO_PIN, GPIO.OUT)
    GPIO.output(LED_GPIO_PIN, LED_STATE)
    GPIO.setup(LED0_RED, GPIO.OUT)
    GPIO.output(LED0_RED, LED_OFF)
    GPIO.setup(LED0_YEL, GPIO.OUT)
    GPIO.output(LED0_YEL, LED_OFF)
    GPIO.setup(LED0_GRN, GPIO.OUT)
    GPIO.output(LED0_GRN, LED_OFF)
    GPIO.setup(LED1_RED, GPIO.OUT)
    GPIO.output(LED1_RED, LED_OFF)
    GPIO.setup(LED1_YEL, GPIO.OUT)
    GPIO.output(LED1_YEL, LED_OFF)
    GPIO.setup(LED1_GRN, GPIO.OUT)
    GPIO.output(LED1_GRN, LED_OFF)
    GPIO.setup(LED2_RED, GPIO.OUT)
    GPIO.output(LED2_RED, LED_OFF)
    GPIO.setup(LED2_YEL, GPIO.OUT)
    GPIO.output(LED2_YEL, LED_OFF)
    GPIO.setup(LED2_GRN, GPIO.OUT)
    GPIO.output(LED2_GRN, LED_OFF)
    GPIO.setup(LED3_RED, GPIO.OUT)
    GPIO.output(LED3_RED, LED_OFF)
    GPIO.setup(LED3_YEL, GPIO.OUT)
    GPIO.output(LED3_YEL, LED_OFF)
    GPIO.setup(LED3_GRN, GPIO.OUT)
    GPIO.output(LED3_GRN, LED_OFF)

# Initialize the selected Omron sensor

    GPIO.output(LED0_GRN, LED_ON)
    if DEBUG:
        print('DEBUG switch is on, initializing Omron sensor...')

    (OMRON1_HANDLE, OMRON1_RESULT) = \
        omron_init(RASPI_I2C_CHANNEL, OMRON_1, PIGPIO_HANDLE, I2C_BUS)

    if OMRON1_HANDLE < 1:
        GPIO.output(LED0_GRN, LED_OFF)
        GPIO.output(LED0_RED, LED_ON)
        panic()

# Open log file

    LOGFILE_HANDLE = open(LOGFILE_NAME, 'wb')
    LOGFILE_OPEN_STRING = '\r\nStartup log file opened at ' \
                          +str(datetime.now())
    LOGFILE_ARGS_STRING = '\r\nDEBUG: '+str(DEBUG)+' SERVO: ' \
                          +str(SERVO_ENABLED)+' MONITOR: ' \
                          +str(MONITOR)+ \
                          ' ROAM: '+str(ROAM)+' RAND: '+str(RAND)
# doing a print here makes sure that the stdout gets a message
    print('Log file name '+str(LOGFILE_NAME)+LOGFILE_OPEN_STRING)
    LOGFILE_HANDLE.write(LOGFILE_OPEN_STRING)
    print LOGFILE_ARGS_STRING
    LOGFILE_HANDLE.write(LOGFILE_ARGS_STRING)

    CPU_TEMP = getCPUtemperature()
    LOGFILE_TEMP_STRING = '\r\nInitial CPU Temperature = '+str(CPU_TEMP)
    print LOGFILE_TEMP_STRING
    LOGFILE_HANDLE.write(LOGFILE_TEMP_STRING)
        
    LOGFILE_HANDLE.write('\r\nPiGPIO version = '+str(PIGPIO_VERSION))
    debug_print('PiGPIO version = '+str(PIGPIO_VERSION))
    debug_print('Omron 1 sensor result = '+str(OMRON1_RESULT))

# initialze the music player
    pygame.mixer.init()

    if SERVO_ENABLED:
        debug_print('SERVO is on - you have 20 seconds to calibrate the bot head')
        for g in range(0, 19):
            debug_print(str(g))
            if (g % 2):
                GPIO.output(LED0_RED, LED_ON)
                GPIO.output(LED0_YEL, LED_ON)
                GPIO.output(LED0_GRN, LED_ON)
                GPIO.output(LED3_RED, LED_ON)
                GPIO.output(LED3_YEL, LED_ON)
                GPIO.output(LED3_GRN, LED_ON)
                GPIO.output(LED1_RED, LED_OFF)
                GPIO.output(LED1_YEL, LED_OFF)
                GPIO.output(LED1_GRN, LED_OFF)
                GPIO.output(LED2_RED, LED_OFF)
                GPIO.output(LED2_YEL, LED_OFF)
                GPIO.output(LED2_GRN, LED_OFF)
            else:
                GPIO.output(LED1_RED, LED_ON)
                GPIO.output(LED1_YEL, LED_ON)
                GPIO.output(LED1_GRN, LED_ON)
                GPIO.output(LED2_RED, LED_ON)
                GPIO.output(LED2_YEL, LED_ON)
                GPIO.output(LED2_GRN, LED_ON)
                GPIO.output(LED0_RED, LED_OFF)
                GPIO.output(LED0_YEL, LED_OFF)
                GPIO.output(LED0_GRN, LED_OFF)
                GPIO.output(LED3_RED, LED_OFF)
                GPIO.output(LED3_YEL, LED_OFF)
                GPIO.output(LED3_GRN, LED_OFF)
            time.sleep(1.0)

    debug_print('Looking for a person')

    NO_PERSON_COUNT = 0
    P_DETECT = False

# Used to lessen the number of repeat "hello" and "goodbye" messages.
# Once a person is detected, assume they will be there for a short time
# so this is used to wait a while before saying goodbye
                            
    P_DETECT_COUNT = 0
################################
# initialize the PID controller
################################

# PID controller is the feedback loop controller for person following
    PID_CONTROLLER = PID(1.0, 0.1, 0.0)
# seconds to allow temps to settle once the head has moved
    SETTLE_TIME = 1.0
# minimum microseconds if PID error is less than this head will stop
    MINIMUM_ERROR_GRANULARITY = 20

    HELLO_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/20150201_zoe-hello1.mp3"
    AFTER_HELLO_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-sorry.mp3"
    GOODBYE_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/20150201_chloe-goodbye1.mp3"
    BADGE_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/badge_file.mp3"
    BURN_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/girl-warning.mp3"
    STRETCH_FILE_NAME = \
        "/home/pi/projects_ggg/raspbot/snd/stretch.mp3"                      
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

    if CONNECTED:
        speakSpeechFromText("Now might be a good time to stand up and stretch", "stretch.mp3")
        debug_print("Connected to internet")
        LOGFILE_HANDLE.write('\r\nConnected to the Internet')
        play_sound(MAX_VOLUME, "stretch.mp3")
    else:
        debug_print("Not connected to internet")
        LOGFILE_HANDLE.write('\r\nNOT connected to the Internet')   
        
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
    STATE_BURN = 5
    PERSON_STATE = STATE_NOTHING
    PREV_PERSON_STATE = STATE_NOTHING
    POSSIBLE_PERSON_MAX = 10 # after 10 one-hits, move head
    POSSIBLE_PERSON = 0
    PROBABLE_PERSON = 0
    EXERSIZE_TIMEOUT = 1200   # seconds between exersize reminders
    EXERSIZE_TIMEOUT_BLINKS = 10    # number of times to blink LEDs
                                    # each blink takes 2 seconds
    
# setup the IR color window
    if MONITOR:
        SCREEN_DISPLAY = pygame.display.set_mode(SCREEN_DIMENSIONS)
        pygame.display.set_caption('IR temp array')

# initialize the window QUADRANT areas for displaying temperature
        PIXEL_WIDTH = SCREEN_DIMENSIONS[0]/4
        PX = (PIXEL_WIDTH*3, PIXEL_WIDTH*2, PIXEL_WIDTH, 0)
# using width here to keep an equal square; bottom section for messages
        PIXEL_HEIGHT = SCREEN_DIMENSIONS[0]/4
        PY = (0, PIXEL_WIDTH, PIXEL_WIDTH*2, PIXEL_WIDTH*3)
        for x in range(0, 4):
            for y in range(0, 4):
                QUADRANT[(x*4)+y] = \
                    (PX[x], PY[y], PIXEL_WIDTH, PIXEL_HEIGHT)
                CENTER[(x*4)+y] = \
                    (PIXEL_WIDTH/2+PX[x], PIXEL_HEIGHT/2+PY[y])

    # initialize the location of the message area
        ROOM_TEMP_AREA = (0, SCREEN_DIMENSIONS[0], \
                          SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
        ROOM_TEMP_MSG_XY = (SCREEN_DIMENSIONS[0]/2, \
                        (SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

        MESSAGE_AREA = (0, \
                        SCREEN_DIMENSIONS[0]+SCREEN_DIMENSIONS[0]/4, \
                        SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
        MESSAGE_AREA_XY = (SCREEN_DIMENSIONS[0]/2, \
                        (SCREEN_DIMENSIONS[1]/6)+ \
                        (SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

#############################
# Main while loop
#############################
    MAIN_LOOP_COUNT = 0
    while True:                 # The main loop
        MAIN_LOOP_COUNT += 1
        CPU_TEMP = getCPUtemperature()
        debug_print('\r\n^^^^^^^^^^^^^^^^^^^^\r\n    MAIN_WHILE_LOOP: '\
                    +str(MAIN_LOOP_COUNT)+' Pcount: ' \
                    +str(P_DETECT_COUNT)+ \
                    ' Servo: '+str(SERVO_POSITION)+' CPU: '+ \
                    str(CPU_TEMP)+' Uptime(sec) = '+str(get_uptime())+ \
                    '\r\n^^^^^^^^^^^^^^^^^^^^')
# Check for overtemp
        if (CPU_TEMP >= 105.0):
            if CPU_105_ON:
                play_sound(MAX_VOLUME, CPU_105_FILE_NAME)
#                    debug_print('Played 105 audio')
        elif (CPU_TEMP >= 110.0):
            play_sound(MAX_VOLUME, CPU_110_FILE_NAME)
#                debug_print('Played 110 audio')
        elif (CPU_TEMP >= 115.0):
            play_sound(MAX_VOLUME, CPU_115_FILE_NAME)
#                debug_print('Played 115 audio')
        elif (CPU_TEMP >= 120.0):
            play_sound(MAX_VOLUME, CPU_120_FILE_NAME)
#                debug_print('Played 120 audio')
        elif (CPU_TEMP >= 125.0):
            play_sound(MAX_VOLUME, CPU_125_FILE_NAME)
#                debug_print('Played 125 audio')

# periododically, write the log file to disk
        if MAIN_LOOP_COUNT >= LOG_MAX:
            debug_print('\r\nLoop count max reached (' \
                +str(MAIN_LOOP_COUNT)+' at '+str(datetime.now()))
            MAIN_LOOP_COUNT = 0      # reset the counter
            debug_print('\r\nClosing log file at '+str(datetime.now()))
            LOGFILE_HANDLE.close       # for forensic analysis

            LOGFILE_HANDLE = open(LOGFILE_NAME, 'wb')
            debug_print('\r\nLog file re-opened at ' \
                        +str(datetime.now()))
            debug_print(LOGFILE_OPEN_STRING)
            debug_print(LOGFILE_ARGS_STRING)
            debug_print(LOGFILE_TEMP_STRING)
            debug_print('person temp threshold = ' \
                       +str(PERSON_TEMP_THRESHOLD))
# Display the Omron internal temperature
            debug_print('Servo Type: '+str(SERVO_TYPE))

# reinitialize the mixer; for some reason the audio drops out
# after extended periods of operating time. See if this fixes
            pygame.mixer.init()

# start roaming again            
            NO_PERSON_COUNT = 0
            P_DETECT_COUNT  = 0
            ROAM_COUNT = 0

        if (LED_STATE == False):
            LED_STATE = True
#                debug_print('Turning LED on')
            GPIO.output(LED_GPIO_PIN, LED_STATE)
        else:
            LED_STATE = False
#                debug_print('Turning LED off')
            GPIO.output(LED_GPIO_PIN, LED_STATE)
            
        time.sleep(MEASUREMENT_WAIT_PERIOD)

        for event in pygame.event.get():
            if event.type == QUIT:
                CRASH_MSG = '\r\npygame event QUIT'
                crash_and_burn(CRASH_MSG, pygame, SERVO_HANDLE, \
                               LOGFILE_HANDLE)
            if event.type == KEYDOWN:
                if event.key == K_q or event.key == K_ESCAPE:
                    CRASH_MSG = \
                    '\r\npygame event: keyboard q or esc pressed'
                    crash_and_burn(CRASH_MSG, pygame, \
                                   SERVO_HANDLE, LOGFILE_HANDLE)

# read the raw temperature data
# 
        (BYTES_READ, TEMPERATURE_ARRAY, ROOM_TEMP) = \
            omron_read(OMRON1_HANDLE, DEGREE_UNIT, \
            OMRON_BUFFER_LENGTH, PIGPIO_HANDLE)
        OMRON_READ_COUNT += 1
     
# Display each element's temperature in F
#            debug_print('New temperature measurement')
#            print_temps(TEMPERATURE_ARRAY)

        if BYTES_READ != OMRON_BUFFER_LENGTH: # sensor problem
            OMRON_ERROR_COUNT += 1
            debug_print( \
                'ERROR: Omron thermal sensor failure! Bytes read: '\
                +str(BYTES_READ))
            panic()

        if MONITOR:
# create the IR pixels
            for i in range(0, OMRON_DATA_LIST):
# This fills each array square with a color that matches the temp
                SCREEN_DISPLAY.fill(fahrenheit_to_rgb(MAX_TEMP, \
                            MIN_TEMP, TEMPERATURE_ARRAY[i]), \
                            QUADRANT[i])
# Display temp value
                if TEMPERATURE_ARRAY[i] > PERSON_TEMP_THRESHOLD:
                    SCREEN_TEXT = \
                        FONT.render("%.1f"%TEMPERATURE_ARRAY[i], \
                                    1, name_to_rgb('red'))
                else:
                    SCREEN_TEXT = \
                        FONT.render("%.1f"%TEMPERATURE_ARRAY[i], \
                                    1, name_to_rgb('navy'))
                SCREEN_TEXT_POS = SCREEN_TEXT.get_rect()
                SCREEN_TEXT_POS.center = CENTER[i]
                SCREEN_DISPLAY.blit(SCREEN_TEXT, SCREEN_TEXT_POS)

# Create an area to display the room temp and messages
            SCREEN_DISPLAY.fill(fahrenheit_to_rgb(MAX_TEMP, \
                                MIN_TEMP, ROOM_TEMP), \
                                ROOM_TEMP_AREA)
            SCREEN_TEXT = FONT.render("Room: %.1f"%ROOM_TEMP, 1, \
                                name_to_rgb('navy'))
            SCREEN_TEXT_POS = SCREEN_TEXT.get_rect()
            SCREEN_TEXT_POS.center = ROOM_TEMP_MSG_XY
            SCREEN_DISPLAY.blit(SCREEN_TEXT, SCREEN_TEXT_POS)

# update the screen
            pygame.display.update()

# testing panic
#        panic()
        
###########################
# Analyze sensor data
###########################

        PREVIOUS_HIT_COUNT = HIT_COUNT
        HIT_COUNT = 0
        HIT_ARRAY = [0, 0, 0, 0]
        HIT_ARRAY_TEMP = \
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # go through each array element to find person "hits"
        # max hit count is 4 unless there is a burn hazard
        for element in range(0, OMRON_DATA_LIST):
            if (TEMPERATURE_ARRAY[element] > \
                BURN_HAZARD_TEMP):
                HIT_ARRAY_TEMP[element] = BURN_HAZARD_HIT
                HIT_COUNT += 1
                
            elif (TEMPERATURE_ARRAY[element] > \
                  PERSON_TEMP_THRESHOLD):
                HIT_ARRAY_TEMP[element] += 1
                HIT_COUNT += 1

            else:
                HIT_ARRAY_TEMP[element] = 0
                
        # far left column
        HIT_ARRAY[0] = HIT_ARRAY_TEMP[12]+HIT_ARRAY_TEMP[13]+ \
                     HIT_ARRAY_TEMP[14]+HIT_ARRAY_TEMP[15]
        HIT_ARRAY[1] = HIT_ARRAY_TEMP[8]+HIT_ARRAY_TEMP[9]+ \
                     HIT_ARRAY_TEMP[10]+HIT_ARRAY_TEMP[11] 
        HIT_ARRAY[2] = HIT_ARRAY_TEMP[4]+HIT_ARRAY_TEMP[5]+ \
                     HIT_ARRAY_TEMP[6]+HIT_ARRAY_TEMP[7] 
        # far right column
        HIT_ARRAY[3] = HIT_ARRAY_TEMP[0]+HIT_ARRAY_TEMP[1]+ \
                     HIT_ARRAY_TEMP[2]+HIT_ARRAY_TEMP[3] 

        GPIO.output(LED0_RED, LED_OFF)
        GPIO.output(LED0_YEL, LED_OFF)
        GPIO.output(LED0_GRN, LED_OFF)
        if (HIT_ARRAY[0] == 1):
            GPIO.output(LED0_YEL, LED_ON)
        elif (HIT_ARRAY[0] >= 2 and HIT_ARRAY[0] <= 4):
            GPIO.output(LED0_GRN, LED_ON)
        elif (HIT_ARRAY[0] > 4):
            GPIO.output(LED0_RED, LED_ON)
            
        GPIO.output(LED1_RED, LED_OFF)
        GPIO.output(LED1_YEL, LED_OFF)
        GPIO.output(LED1_GRN, LED_OFF)
        if (HIT_ARRAY[1] == 1):
            GPIO.output(LED1_YEL, LED_ON)
        elif (HIT_ARRAY[1] >= 2 and HIT_ARRAY[1] <= 4):
            GPIO.output(LED1_GRN, LED_ON)
        elif (HIT_ARRAY[1] > 4):
            GPIO.output(LED1_RED, LED_ON)
            
        GPIO.output(LED2_RED, LED_OFF)
        GPIO.output(LED2_YEL, LED_OFF)
        GPIO.output(LED2_GRN, LED_OFF)
        if (HIT_ARRAY[2] == 1):
            GPIO.output(LED2_YEL, LED_ON)
        elif (HIT_ARRAY[2] >= 2 and HIT_ARRAY[2] <= 4):
            GPIO.output(LED2_GRN, LED_ON)
        elif (HIT_ARRAY[2] > 4):
            GPIO.output(LED2_RED, LED_ON)
            
        GPIO.output(LED3_RED, LED_OFF)
        GPIO.output(LED3_YEL, LED_OFF)
        GPIO.output(LED3_GRN, LED_OFF)
        if (HIT_ARRAY[3] == 1):
            GPIO.output(LED3_YEL, LED_ON)
        elif (HIT_ARRAY[3] >= 2 and HIT_ARRAY[3] <= 4):
            GPIO.output(LED3_GRN, LED_ON)
        elif (HIT_ARRAY[3] > 4):
            GPIO.output(LED3_RED, LED_ON)
                             
        debug_print('\r\n-----------------------\r\nhit array: '+\
                    str(HIT_ARRAY[0])+str(HIT_ARRAY[1])+ \
                    str(HIT_ARRAY[2])+str(HIT_ARRAY[3])+ \
                    '\r\nhit count: '+str(HIT_COUNT)+ \
                    '\r\n-----------------------')

        if max(TEMPERATURE_ARRAY) > BURN_HAZARD_TEMP:
            PERSON_STATE = STATE_BURN

###########################
# Burn Hazard Detected !
###########################
        if (PERSON_STATE == STATE_BURN):
            debug_print('STATE: BURN: Burn Hazard cnt: ' \
                       +str(BURN_HAZARD_CNT)+' ROAM_COUNT = ' \
                       +str(ROAM_COUNT))
            ROAM_COUNT = 0
            POSSIBLE_PERSON = 0
            BURN_HAZARD_CNT += 1
            LED_STATE = True
            GPIO.output(LED_GPIO_PIN, LED_STATE)
            if MONITOR:
                SCREEN_DISPLAY.fill(name_to_rgb('red'), \
                                    MESSAGE_AREA)
                SCREEN_TEXT = FONT.render("WARNING! Burn danger!", \
                                          1, name_to_rgb('yellow'))
                SCREEN_TEXT_POS = SCREEN_TEXT.get_rect()
                SCREEN_TEXT_POS.center = MESSAGE_AREA_XY
                SCREEN_DISPLAY.blit(SCREEN_TEXT, SCREEN_TEXT_POS)
# update the screen
                pygame.display.update()

            debug_print('\r\n'+"Burn hazard temperature is " \
                       +"%.1f"%max(TEMPERATURE_ARRAY)+" degrees")
            
            # play this only once, otherwise, its too annoying
            if (BURN_HAZARD_CNT == 1):
                play_sound(MAX_VOLUME, BURN_FILE_NAME)
                debug_print('Played Burn warning audio')

            MOVE_DIST = 0
            MOVE_CW = True
            HAZARD_POSITION = 0
            
            if (HIT_ARRAY[0] > BURN_HAZARD_HIT and \
                HIT_ARRAY[1] < BURN_HAZARD_HIT and \
                HIT_ARRAY[2] < BURN_HAZARD_HIT and \
                HIT_ARRAY[3] < BURN_HAZARD_HIT):
                    MOVE_DIST = MOVE_DIST_SHORT
                    MOVE_CW = True

            elif (HIT_ARRAY[0] < BURN_HAZARD_HIT and \
                  HIT_ARRAY[1] < BURN_HAZARD_HIT and \
                  HIT_ARRAY[2] < BURN_HAZARD_HIT and \
                  HIT_ARRAY[3] > BURN_HAZARD_HIT):
                      MOVE_DIST = MOVE_DIST_SHORT
                      MOVE_CW = False

            if (MOVE_DIST > 0):
                HAZARD_POSITION = \
                    resolve_new_position(MOVE_CW, \
                                         SERVO_POSITION, \
                                         MOVE_DIST)
                debug_print('hazard_position: Pos: '+ \
                            str(HAZARD_POSITION))
                SERVO_POSITION = move_head(HAZARD_POSITION, \
                                           SERVO_POSITION)

            if max(TEMPERATURE_ARRAY) > BURN_HAZARD_TEMP:
                PERSON_STATE = STATE_BURN
            else:
# Drop back to looking for a person
                PERSON_STATE = STATE_NOTHING

##                if CONNECTED:
##                    try:
##                        speakSpeechFromText("The temperature is "+ \
##                            "%.1f"%max(TEMPERATURE_ARRAY)+ \
##                            " degrees fahrenheit", "mtemp.mp3")
##                        play_sound(MAX_VOLUME, "mtemp.mp3")
##                    except:
##                        continue

###########################
# No Person Detected
###########################
# State 0: NOTHING - no heat source in view
#     Event 0: No change - outcome: continue waiting for a person
#     Event 1: One or more sensors cross the person threshold
#
        elif (PERSON_STATE == STATE_NOTHING):
            debug_print('STATE: NOTHING: No Person cnt: ' \
                       +str(NO_PERSON_COUNT)+' ROAM_COUNT = ' \
                       +str(ROAM_COUNT))
            NO_PERSON_COUNT += 1
            P_DETECT_COUNT = 0
            POSSIBLE_PERSON = 0
            PROBABLE_PERSON = 0
            BURN_HAZARD_CNT = 0
            if MONITOR:
                SCREEN_DISPLAY.fill(name_to_rgb('white'), \
                                    MESSAGE_AREA)
                SCREEN_TEXT = FONT.render("Waiting...", 1, \
                                   name_to_rgb('blue'))
                SCREEN_TEXT_POS = SCREEN_TEXT.get_rect()
                SCREEN_TEXT_POS.center = MESSAGE_AREA_XY
                SCREEN_DISPLAY.blit(SCREEN_TEXT, SCREEN_TEXT_POS)
# update the screen
                pygame.display.update()
            if (HIT_COUNT == 0 or PREVIOUS_HIT_COUNT == 0):
                PERSON_STATE = STATE_NOTHING
            else:
                PERSON_STATE = STATE_POSSIBLE

            (ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
             LAST_KNOWN_LED_POS, LIT_LED) = \
            servo_roam(ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
                       LAST_KNOWN_LED_POS, LIT_LED)
            
            PREV_PERSON_STATE = STATE_NOTHING
                
###########################
# Possible Person Detected
###########################
# State 1: Possible person in view - one or more sensors had a hit
#     Event 0: No hits - blip, move to State 0
#     Event 1: One hit - move head to try to center on the hit
#     Event 2: More than one hit - state 2
#
        elif (PERSON_STATE == STATE_POSSIBLE):
            BURN_HAZARD_CNT = 0
            debug_print('STATE: POSSIBLE: Possible Person cnt: ' \
                       +str(POSSIBLE_PERSON))
            NO_PERSON_COUNT += 1
            if (HIT_COUNT == 0 or PREVIOUS_HIT_COUNT == 0):
                PERSON_STATE = STATE_NOTHING
            elif (HIT_COUNT == 1 and PREVIOUS_HIT_COUNT >= 1):
                P_DETECT, PERSON_POSITION = \
                    person_position_1_hit(HIT_ARRAY, SERVO_POSITION)
                # stay in possible state
                if (P_DETECT):
                    POSSIBLE_PERSON += 1
                    if (POSSIBLE_PERSON > POSSIBLE_PERSON_MAX):
                        POSSIBLE_PERSON = 0
                        SERVO_POSITION = \
                            move_head(PERSON_POSITION, \
                                      SERVO_POSITION)
                        ROAM_COUNT = 0
                else:
                    PERSON_STATE = STATE_NOTHING
            else:
                PERSON_STATE = STATE_LIKELY

            (ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
             LAST_KNOWN_LED_POS, LIT_LED) = \
            servo_roam(ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
                       LAST_KNOWN_LED_POS, LIT_LED)
            
            PREV_PERSON_STATE = STATE_POSSIBLE
            
###########################
# Likely Person Detected
###########################
# State 2: Likely person in view - more than one sensor had a hit
#     Event 0: No hits - blip, move to State 1
#     Event 1: One hit - noise, no change
#     Event 2: more than one sensor still has a hit, move head, State 3
#
        elif (PERSON_STATE == STATE_LIKELY):
            BURN_HAZARD_CNT = 0
            debug_print('STATE: LIKELY: No Person cnt: ' \
                        +str(NO_PERSON_COUNT))
            POSSIBLE_PERSON = 0
            NO_PERSON_COUNT += 1
            if (HIT_COUNT == 0 or PREVIOUS_HIT_COUNT == 0):
                PERSON_STATE = STATE_NOTHING
            else:
                P_DETECT, PERSON_POSITION = \
                          person_position_2_hit(HIT_ARRAY, \
                                                SERVO_POSITION)
                if (not P_DETECT):
                    PERSON_STATE = STATE_POSSIBLE
                else:
                    SERVO_POSITION = move_head(PERSON_POSITION, \
                                               SERVO_POSITION)
                    ROAM_COUNT = 0
                    
                if (HIT_COUNT > PERSON_HIT_COUNT):
                    PERSON_STATE = STATE_PROBABLE
            
            PREV_PERSON_STATE = STATE_LIKELY

###########################
# Probable Person Detected
###########################
# State 3: Probably a person in view
#     Event 0: No hits - noise, move to State 2
#     Event 1: One hit - noise, move to state 2
#     Event 2: more than one sensor has a hit, move head, say hello
#
        elif (PERSON_STATE == STATE_PROBABLE):
            BURN_HAZARD_CNT = 0
            POSSIBLE_PERSON = 0
            debug_print('STATE: PROBABLE: Probable Person cnt: ' \
                        +str(PROBABLE_PERSON))
            if (HIT_COUNT == 0 or PREVIOUS_HIT_COUNT == 0):
                PERSON_STATE = STATE_LIKELY
            elif (HIT_COUNT == 1 and PREVIOUS_HIT_COUNT >= 1):
                P_DETECT, PERSON_POSITION = \
                    person_position_1_hit(HIT_ARRAY, \
                                          SERVO_POSITION)
                if (P_DETECT):
                    SERVO_POSITION = move_head(PERSON_POSITION, \
                                               SERVO_POSITION)
                    ROAM_COUNT = 0
                else:
                    PERSON_STATE = STATE_LIKELY
            else:
                P_DETECT, PERSON_POSITION = \
                          person_position_2_hit(HIT_ARRAY, \
                                                SERVO_POSITION)
                if (P_DETECT):
                    SERVO_POSITION = move_head(PERSON_POSITION, \
                                               SERVO_POSITION)
                    ROAM_COUNT = 0
                    PROBABLE_PERSON += 1
                    if (PROBABLE_PERSON > PROBABLE_PERSON_THRESH \
                        and HIT_COUNT > 5):
                        say_hello()
                        detected_time_stamp = get_uptime()
                        debug_print('Person detected at '+str(detected_time_stamp))
                        PERSON_STATE = STATE_DETECTED
                        PROBABLE_PERSON = 0
                    else:
                        PERSON_STATE = STATE_PROBABLE
                else:
                    PERSON_STATE = STATE_LIKELY

            PREV_PERSON_STATE = STATE_PROBABLE

###########################
# Person Detected !
###########################
# State 4: Person detected
#     Event 0: No hits - person left, say goodbye, move to state 0
#     Event 1: One hit - person left, say goodbye, move to state 1
#     Event 2: more than one sensor, move head to position, stay
#     
        elif (PERSON_STATE == STATE_DETECTED):
            BURN_HAZARD_CNT = 0
            debug_print('STATE: DETECTED: detect cnt: ' \
                       +str(P_DETECT_COUNT))
            ROAM_COUNT = 0
            NO_PERSON_COUNT = 0
            POSSIBLE_PERSON = 0
            LED_STATE = True
            GPIO.output(LED_GPIO_PIN, LED_STATE)
            P_DETECT_COUNT += 1
            CPU_TEMP = getCPUtemperature()
            debug_print('Person_count: '+str(P_DETECT_COUNT)+ \
                       ' Max: '+"%.1f"%max(TEMPERATURE_ARRAY)+ \
                       ' Servo: '+str(SERVO_POSITION)+' CPU: ' \
                       +str(CPU_TEMP))

# every 20 minutes that a person is detected, have the bot remind
# the person to get some excersize.

            uptime_now = get_uptime()
            if (uptime_now - detected_time_stamp) >= EXERSIZE_TIMEOUT:
                play_sound(MAX_VOLUME, STRETCH_FILE_NAME)
                detected_time_stamp = uptime_now    # reset
                for b in range(0, EXERSIZE_TIMEOUT_BLINKS):
                    GPIO.output(LED0_GRN, LED_OFF)
                    GPIO.output(LED1_GRN, LED_OFF)
                    GPIO.output(LED2_GRN, LED_OFF)
                    GPIO.output(LED3_GRN, LED_OFF)
                    GPIO.output(LED0_YEL, LED_OFF)
                    GPIO.output(LED1_YEL, LED_OFF)
                    GPIO.output(LED2_YEL, LED_OFF)
                    GPIO.output(LED3_YEL, LED_OFF)
                    GPIO.output(LED0_RED, LED_ON)
                    GPIO.output(LED1_RED, LED_ON)
                    GPIO.output(LED2_RED, LED_ON)
                    GPIO.output(LED3_RED, LED_ON)
                    time.sleep(0.3)
                    GPIO.output(LED0_RED, LED_OFF)
                    GPIO.output(LED1_RED, LED_OFF)
                    GPIO.output(LED2_RED, LED_OFF)
                    GPIO.output(LED3_RED, LED_OFF)
                    time.sleep(0.3)
                    GPIO.output(LED0_YEL, LED_ON)
                    GPIO.output(LED1_YEL, LED_ON)
                    GPIO.output(LED2_YEL, LED_ON)
                    GPIO.output(LED3_YEL, LED_ON)
                    time.sleep(0.3)
                    GPIO.output(LED0_YEL, LED_OFF)
                    GPIO.output(LED1_YEL, LED_OFF)
                    GPIO.output(LED2_YEL, LED_OFF)
                    GPIO.output(LED3_YEL, LED_OFF)
                    time.sleep(0.3)
                    GPIO.output(LED0_GRN, LED_ON)
                    GPIO.output(LED1_GRN, LED_ON)
                    GPIO.output(LED2_GRN, LED_ON)
                    GPIO.output(LED3_GRN, LED_ON)
                    time.sleep(0.3)
                    GPIO.output(LED0_GRN, LED_OFF)
                    GPIO.output(LED1_GRN, LED_OFF)
                    GPIO.output(LED2_GRN, LED_OFF)
                    GPIO.output(LED3_GRN, LED_OFF)
                    time.sleep(0.3)

            if (HIT_COUNT == 0 or PREVIOUS_HIT_COUNT == 0):
                say_goodbye()
                PERSON_STATE = STATE_NOTHING
            elif (HIT_COUNT >= 1 and \
                  HIT_COUNT <= PERSON_HIT_COUNT ):
                say_goodbye()
                PERSON_STATE = STATE_POSSIBLE
# hit count needs to be above PERSON_HIT_COUNT to validate a person
            else:
                P_DETECT, PERSON_POSITION = \
                          person_position_2_hit(HIT_ARRAY, \
                                                SERVO_POSITION)
                if (P_DETECT):
                    SERVO_POSITION = move_head(PERSON_POSITION, \
                                               SERVO_POSITION)
                else:
                    PERSON_STATE = STATE_LIKELY
                    say_goodbye()

            PREV_PERSON_STATE = STATE_DETECTED

###########################
# Invalid state
###########################
        else:
            PERSON_STATE = STATE_NOTHING
            (ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
             LAST_KNOWN_LED_POS, LIT_LED) = \
            servo_roam(ROAM_COUNT, SERVO_POSITION, SERVO_DIRECTION, \
                       LAST_KNOWN_LED_POS, LIT_LED)
                       
#############################
# End main while loop
#############################

except KeyboardInterrupt:
    print 'Keyboard Interrupt Exception!'
    CRASH_MSG = '\r\nKeyboard interrupt; quitting'
    crash_and_burn(CRASH_MSG, pygame, SERVO_HANDLE, LOGFILE_HANDLE)

except IOError:
    print 'I/O Error Exception!'
    GPIO.output(LED0_GRN, LED_OFF)
    GPIO.output(LED0_RED, LED_ON)
    # do not close the logfile here
    # allows the previous logfile to stay intact for a forensic analysis
    debug_print('\r\nI/O Error; quitting')
    if SERVO_ENABLED:
        SERVO_HANDLE.stop_servo(SERVO_GPIO_PIN)
    panic()

#! /usr/bin/python
"""
# A Python command line tool for a robot based on the Raspberry Pi computer
# By Greg Griffes http://yottametric.com
# GNU GPL V3
#
# This file is automatically run at boot time using the following method:
# edit the /etc/rc.local file using sudo nano /etc/rc.local
# add these two lines at the end before "exit 0"
# sudo pigpiod # starts the pigpio daemon
# sudo python /home/pi/projects_ggg/raspbot/raspbot.py -nomonitor -roam &
#
# The log file created by this program when running independently is located at root (/)
# about every five minutes the log file is closed and reopened.
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
from omron_src import *     # contains omron functions
import urllib, pycurl, os   # needed for text to speech

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
LED_GPIO_PIN = 7       # GPIO number that the LED is connected to (BCM GPIO_04 (Pi Hat) is the same as BOARD pin 7)
                        # See "Raspberry Pi B+ J8 Header" diagram
DEBUG = 0             # set this to 1 to see debug messages on monitor
SCREEN_DIMENSIONS = [400, 600]  # setup the IR color window [0]= width [1]= height
MIN_TEMP = 0            # minimum expected temperature in Fahrenheit
MAX_TEMP = 200          # minimum expected temperature in Fahrenheit
ROAM = 0                        # if true, robot will "roam" looking for a heat signature 
ROAM_MAX = 600          # Max number of times to roam between person detections (roughly 0.5 seconds between roams
LOG_MAX = 1200
RAND = 0                # Causes random head movement when idle
BURN_HAZARD_TEMP = 100          # temperature at which a warning is given
TEMPMARGIN = 5            # number of degrees F greater than room temp to detect a person
PERSON_TEMP_THRESHOLD = 81      # degrees fahrenheit
MONITOR = 1             # assume a monitor is attached
# Servo positions
# Weirdness factor: Some servo's I used go in the reverse direction from other servos. Therefore, this
# next constant is used to change the software to use the appropriate servo. The HiTEC HS-55 Feather servo.

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

# Strange things happen: Some servos move CW and others move CCW for the same number.
# it is possible that the "front" of the servo might be treated differently
# and it seams that the colors of the wires on the servo might indicate different servos:
# brown, red, orange seems to be HIGH_TO_LOW is clockwise (2400 is full CCW and 600 is full CW)
# black, red, yellos seems to be LOW_TO_HIGH is clockwise (2400 is full CW and 600 is full CCW)
if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
    MIN_SERVO_POSITION = 2400
    MAX_SERVO_POSITION = 600
    SERVO_LIMIT_CW = MIN_SERVO_POSITION
    SERVO_LIMIT_CCW = MAX_SERVO_POSITION
    X_DELTA_0 = 200
    X_DELTA_1 = 100
    X_DELTA_2 = -100
    X_DELTA_3 = -200
else:
    MIN_SERVO_POSITION = 600
    MAX_SERVO_POSITION = 2400
    SERVO_LIMIT_CW = MAX_SERVO_POSITION
    SERVO_LIMIT_CCW = MIN_SERVO_POSITION
    X_DELTA_0 = -200
    X_DELTA_1 = -100
    X_DELTA_2 = 100
    X_DELTA_3 = 200

# Logfile
LOGFILE_NAME = 'raspbot_logfile.txt'

import RPi.GPIO as GPIO
GPIO.setwarnings(False)         # if true, we get warnings about DMA channel in use
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_GPIO_PIN, GPIO.OUT)
GPIO.setup(LED_GPIO_PIN, GPIO.OUT)
from RPIO import PWM        # for the servo motor

CONNECTED = 0           # true if connected to the internet

# function for celcius to farenheiht conversion
def c2f (c):
   return 9.0*c/5.0 + 32

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
    debugPrint("%.1f"%temp_list[12]+' '+"%.1f"%temp_list[8]+' '+"%.1f"%temp_list[4]+' '+"%.1f"%temp_list[0]+' ')
    debugPrint("%.1f"%temp_list[13]+' '+"%.1f"%temp_list[9]+' '+"%.1f"%temp_list[5]+' '+"%.1f"%temp_list[1]+' ')
    debugPrint("%.1f"%temp_list[14]+' '+"%.1f"%temp_list[10]+' '+"%.1f"%temp_list[6]+' '+"%.1f"%temp_list[2]+' ')
    debugPrint("%.1f"%temp_list[15]+' '+"%.1f"%temp_list[11]+' '+"%.1f"%temp_list[7]+' '+"%.1f"%temp_list[3]+' ')

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

    return ((intR, intG, intB))

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

        debugPrint('Desired servo position: '+str(new_position))
            
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            if (new_position >= MAX_SERVO_POSITION and new_position <= MIN_SERVO_POSITION):
                if (new_position%MINIMUM_SERVO_GRANULARITY < 5):        # if there is a remainder, then we need to make the value in 10us increments
                    final_position = (new_position//MINIMUM_SERVO_GRANULARITY)*MINIMUM_SERVO_GRANULARITY
                else:
                    final_position = ((new_position//MINIMUM_SERVO_GRANULARITY)+1)*MINIMUM_SERVO_GRANULARITY
                servo.set_servo(SERVO_GPIO_PIN, final_position)
            else:
                debugPrint('ERROR: set_servo_to_position L-H=CW position out of range: '+str(new_position)+' min= '+str(MIN_SERVO_POSITION)+' max = '+str(MAX_SERVO_POSITION))
        else:
            if (new_position >= MIN_SERVO_POSITION and new_position <= MAX_SERVO_POSITION):
                if (new_position%MINIMUM_SERVO_GRANULARITY < 5):        # if there is a remainder, then we need to make the value in 10us increments
                    final_position = (new_position//MINIMUM_SERVO_GRANULARITY)*MINIMUM_SERVO_GRANULARITY
                else:
                    final_position = ((new_position//MINIMUM_SERVO_GRANULARITY)+1)*MINIMUM_SERVO_GRANULARITY
                servo.set_servo(SERVO_GPIO_PIN, final_position)
            else:
                debugPrint('ERROR: set_servo_to_position L-H=CCW position out of range: '+str(new_position)+' min= '+str(MIN_SERVO_POSITION)+' max = '+str(MAX_SERVO_POSITION))

            debugPrint('Setting servo to position: '+str(final_position))

        return final_position

def get_hit_array(room, t_array, s_position):
    """
    Used to fill a 4x4 array with a person "hit" temperature logical value
    """
    hit_count = 0

    debugPrint('person temp threshold = '+str(PERSON_TEMP_THRESHOLD))

    hit_count=[0]*OMRON_DATA_LIST
    t_delta=[0.0]*OMRON_DATA_LIST       # holds the difference between threshold and actual
    h_delta=[0]*4

    for i in range(0,OMRON_DATA_LIST):
        if (t_array[i] > PERSON_TEMP_THRESHOLD and t_array[i] < BURN_HAZARD_TEMP):     # a person temperature
            t_delta[i] = t_array[i] - PERSON_TEMP_THRESHOLD
            hit_count[i] += 1
            
#    debugPrint('Hit count = '+str(hit_count))
    debugPrint('Temperature deltas')
    print_temps(t_delta)
     
# use hit counts as a weighting factor: 1 hit = x percent increase
    h_delta[0] = hit_count[12]+hit_count[13]+hit_count[14]+hit_count[15] # add up the far left column
    h_delta[1] = hit_count[8]+hit_count[9]+hit_count[10]+hit_count[11] 
    h_delta[2] = hit_count[4]+hit_count[5]+hit_count[6]+hit_count[7] 
    h_delta[3] = hit_count[0]+hit_count[1]+hit_count[2]+hit_count[3] 

    debugPrint('hit count delta: '+str(h_delta[0])+str(h_delta[1])+str(h_delta[2])+str(h_delta[3]))

    return h_delta

FAR = 220       # the number of microseconds to move the servo when the person is not near cetner
NEAR = 120      # the number of microseconds to move the servo when the person is near the center
def person_position_2_hit(room, t_array, s_position):
    """
    Used to detect a persons presence using the "greater than two algorithm"
    returns (TRUE/FALSE - person detected, whole integer - approximate person position)
    """

    hit_array = get_hit_array(room, t_array, s_position)

# First, look for > two hits in a single column
    if (hit_array[1] >= 2 and hit_array[2] >= 2):
        # stop
        return (True, s_position)
    elif (hit_array[0] >= 2 and hit_array[1] <= 1 and hit_array[2] <= 1 and hit_array[3] <= 1):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + FAR)
        else:
            return (True, s_position - FAR)
    elif (hit_array[1] >= 2 and hit_array[2] <= 1 and hit_array[3] <= 1):
        # move CW 15
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR)
        else:
            return (True, s_position - NEAR)
    elif (hit_array[2] >= 2 and hit_array[0] <= 1 and hit_array[1] <= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR)
        else:
            return (True, s_position + NEAR)
    elif (hit_array[3] >= 2 and hit_array[0] <= 1 and hit_array[1] <= 1 and hit_array[2] <= 1):
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
    Used to detect a persons presence using the "look for hits in a 2x2 quadrant algorithm"
    returns (TRUE/FALSE - person detected, whole integer - approximate person position)
    """

    hit_array = get_hit_array(room, t_array, s_position)

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
    Used to detect a persons presence using the "greater than one algorithm"
    returns (TRUE/FALSE - person detected, whole integer - approximate person position)
    """

    hit_array = get_hit_array(room, t_array, s_position)

    if ((hit_array[0] >= 1 or hit_array[0] == 0) and hit_array[1] >= 1 and hit_array[2] >= 1 and (hit_array[3] >= 1 or hit_array[3] == 0)):
        # no change
        return (True, s_position)
    elif (hit_array[0] == 0 and hit_array[1] == 0 and hit_array[2] == 0 and hit_array[3] >= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - FAR_ONE)
        else:
            return (True, s_position + FAR_ONE)
    elif (hit_array[0] == 0 and hit_array[1] == 0 and hit_array[2] >= 1 and hit_array[3] == 0):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR_ONE)
        else:
            return (True, s_position + NEAR_ONE)
    elif (hit_array[0] == 0 and hit_array[1] >= 1 and hit_array[2] == 0 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR_ONE)
        else:
            return (True, s_position - NEAR_ONE)
    elif (hit_array[0] >= 1 and hit_array[1] == 0 and hit_array[2] == 0 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + FAR_ONE)
        else:
            return (True, s_position - FAR_ONE)
    elif (hit_array[0] == 0 and hit_array[1] == 0 and hit_array[2] >= 1 and hit_array[3] >= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - FAR_TWO)
        else:
            return (True, s_position + FAR_TWO)
    elif (hit_array[0] >= 1 and hit_array[1] >= 1 and hit_array[2] == 0 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + FAR_TWO)
        else:
            return (True, s_position - FAR_TWO)
    elif (hit_array[0] == 0 and hit_array[1] >= 1 and hit_array[2] >= 1 and hit_array[3] >= 1):
        # move CCW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position - NEAR_THREE)
        else:
            return (True, s_position + NEAR_THREE)
    elif (hit_array[0] >= 1 and hit_array[1] >= 1 and hit_array[2] >= 1 and hit_array[3] == 0):
        # move CW
        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
            return (True, s_position + NEAR_THREE)
        else:
            return (True, s_position - NEAR_THREE)
    else:
        # no person detected
        return (False, s_position)

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

def getCPUtemperature():
#    res = os.popen('vcgencmd measure temp').readline()
#    return(res.replace("temp=","").replace("'C\n",""))
#    process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
#    output, _error = process.communicate()
#    return float(output[output.index('=') + 1:output.index("'")])
    fo = open("/sys/class/thermal/thermal_zone0/temp")
    temp_str = fo.read();
    fo.close()
    temp_degC = float(temp_str)/1000
    temp_degF = c2f(temp_degC)
#    temp1 = temp_str[5:9]
#    temp2 = eval(temp1)
    return temp_degF

def debugPrint(message):
    now_string = str(datetime.now())
    if DEBUG and MONITOR:
        print now_string+': '+message
    logfile.write('\r\n'+now_string+': '+message)
    

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

if "-nomonitor" in sys.argv:
    MONITOR=0             # assume using servo is default

if "-roam" in sys.argv:
    ROAM=1              # set this to 1 to allow robot to roam looking for a person

if "-rand" in sys.argv:
    RAND=1              # set this to 1 to allow robot to roam looking for a person

if "-help" in sys.argv:
    print 'IMPORTANT: run as superuser (sudo) to allow DMA access'
    print '-debug:   print debug info to console'
    print '-nomonitor run without producing the pygame temp display'
    print '-noservo: do not use the servo motor'
    print '-roam:    when no person detected, turn head slowly 180 degrees'
    print '-rand:    when roaming randomize the head movement'
    sys.exit()

# Initialize variables
temperature_array=[0.0]*OMRON_DATA_LIST     # holds the recently measured temperature
temperature_previous=[0.0]*OMRON_DATA_LIST  # keeps track of the previous values
temperature_moving_ave=[0.0]*OMRON_DATA_LIST    # moving average of temperature
left_far=[0.0]*4
left_ctr=[0.0]*4
right_ctr=[0.0]*4
right_far=[0.0]*4

LED_state = True
roam_count = 0                      # keeps track of how many times the head roams so that we can turn it off
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
servo_position = CTR_SERVO_POSITION                 # initialize the servo to face directly forward
if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
    servo_direction = SERVO_CUR_DIR_CW     # initially start moving the servo CCW
else:
    servo_direction = SERVO_CUR_DIR_CCW     # initially start moving the servo CCW

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
        time.sleep(5.0)                # Wait a sec before starting
    else:
        print 'SERVO is off'
    LED_state = True
    GPIO.output(LED_GPIO_PIN, LED_state)

# intialize the pigpio library and socket connection to the daemon (pigpiod)
    pi = pigpio.pi()              # use defaults
    version = pi.get_pigpio_version()

# Initialize the selected Omron sensor

    (omron1_handle, omron1_result) = omron_init(RASPI_I2C_CHANNEL, OMRON_1, pi, i2c_bus) # passing in the i2c address of the sensor

    if omron1_handle < 1:
#        debugPrint('\r\nI2C sensor not found! Aborting! Quitting! Stopping! Good bye.')
        if SERVO:
            servo.stop_servo(SERVO_GPIO_PIN)
        pygame.quit()
        sys.exit()

# Open log file

    logfile = open(LOGFILE_NAME, 'wb')
    logfile_open_string = '\r\nStartup log file opened at '+str(datetime.now())
    logfile_args_string = '\r\nDEBUG: '+str(DEBUG)+' SERVO: '+str(SERVO)+' MONITOR: '+str(MONITOR)+' ROAM: '+str(ROAM)+' RAND: '+str(RAND)
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

    # initialize the location of the message area
        room_temp_area = (0, SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
        room_temp_msg_xy = (SCREEN_DIMENSIONS[0]/2, (SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

        message_area = (0, SCREEN_DIMENSIONS[0]+SCREEN_DIMENSIONS[0]/4, SCREEN_DIMENSIONS[0], SCREEN_DIMENSIONS[0]/4)
        message_area_xy = (SCREEN_DIMENSIONS[0]/2, (SCREEN_DIMENSIONS[1]/6)+(SCREEN_DIMENSIONS[1]/12)+SCREEN_DIMENSIONS[0])

# initialze the music player
    pygame.mixer.init()

    debugPrint('Looking for a person')

    no_person_count = 99
    p_detect = False
    p_detect_count = 0      # This is used to lessen the number of repeat "hello" and "goodbye" messages.
                            # Once a person is detected, we assume they will be there for a short time,
                            # so this is used to wait a while before saying goodbye
                            
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

    HELLO_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/20150201_zoe-hello1.mp3"
    AFTER_HELLO_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-sorry.mp3"
    GOODBYE_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/20150201_chloe-goodbye1.mp3"
    BADGE_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-badge1.mp3"
    BURN_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-warning.mp3"
    CPU_105_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-105a.mp3"
    CPU_110_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-110a.mp3"
    CPU_115_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-115a.mp3"
    CPU_120_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-120a.mp3"
    CPU_125_FILE_NAME = "/home/pi/projects_ggg/raspbot/snd/girl-125a.mp3"
    
    CPU_105_ON = False       # set to true to test CPU temp warnings, otherwise set to false

##    if CONNECTED:
##    try:
##        speakSpeechFromText("Yeay,,,,,, we are connected to the Internet!", "intro.mp3")
##        print "Connected to internet"
##        logfile.write('\r\nConnected to the Internet')
##        play_sound(MAX_VOLUME, "intro.mp3")
##        CONNECTED = 1
##    except:
##        print "Not connected to internet"
##        logfile.write('\r\nNOT connected to the Internet')        
##        CONNECTED = 0
        
#############################
# Main while loop
#############################
    loop_count = 0
    while True:                 # The main loop
        loop_count += 1
        debugPrint('loop_count = '+str(loop_count))
        if loop_count >= LOG_MAX:   # periododically, write the log file to disk

# Check for overtemp
            CPUtemp = getCPUtemperature()
            if (CPUtemp >= 105.0):
                if CPU_105_ON:
                    play_sound(MAX_VOLUME, CPU_105_FILE_NAME)
                    debugPrint('Played 105 audio')
            elif (CPUtemp >= 110.0):
                play_sound(MAX_VOLUME, CPU_110_FILE_NAME)
                debugPrint('Played 110 audio')
            elif (CPUtemp >= 115.0):
                play_sound(MAX_VOLUME, CPU_115_FILE_NAME)
                debugPrint('Played 115 audio')
            elif (CPUtemp >= 120.0):
                play_sound(MAX_VOLUME, CPU_120_FILE_NAME)
                debugPrint('Played 120 audio')
            elif (CPUtemp >= 125.0):
                play_sound(MAX_VOLUME, CPU_125_FILE_NAME)
                debugPrint('Played 125 audio')

            debugPrint('\r\nLoop count max reached ('+str(loop_count)+' at '+str(datetime.now()))
            loop_count = 0      # reset the counter
            debugPrint('\r\nClosing log file at '+str(datetime.now()))
            time.sleep(1)     # it appears that you need to give the system a little time to write everything out
            logfile.close       # for forensic analysis
            time.sleep(1)     # it appears that you need to give the system a little time
            logfile = open(LOGFILE_NAME, 'wb')
            time.sleep(1)     # it appears that you need to give the system a little time
            debugPrint('\r\nLog file re-opened at '+str(datetime.now()))
            debugPrint(logfile_open_string)
            debugPrint(logfile_args_string)
            debugPrint(logfile_temp_string)

# start roaming again            
            no_person_count = 0
            p_detect_count  = 0

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
 
            debugPrint('Previous temperature measurement')
            print_temps(temperature_previous)
            debugPrint('')

# Format: (bytes_read, temperature_array, room_temp) = omron_read(sensor_handle, C/F, length of temperature array, pigpio socket handle)
# returns bytes_read - if not equal to length of temperature array, then sensor error
 
            (bytes_read, temperature_array, room_temp) = omron_read(omron1_handle, DEGREE_UNIT, OMRON_BUFFER_LENGTH, pi)
            omron_read_count += 1
         
# Display each element's temperature in F
            debugPrint('New temperature measurement')
            print_temps(temperature_array)

            if bytes_read != OMRON_BUFFER_LENGTH: # sensor problem
                omron_error_count += 1
                debugPrint('ERROR: Omron thermal sensor failure! Bytes read: '+str(bytes_read))
                fatal_error = 1
                break

            for i in range(0,OMRON_DATA_LIST):
                temp_list = [temperature_array[i], temperature_previous[i], temperature_moving_ave[i]]
                temperature_moving_ave[i] = avg(temp_list)

# Display each element's temperature in F
            debugPrint('Temperature moving average')
            print_temps(temperature_moving_ave)

# Display the Omron internal temperature (room temp - something to compare signals with)
            debugPrint('Omron D6T internal temp = '+"%.1f"%room_temp+' F')

            if MONITOR:
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
# Analyze sensor data
###########################

            if ((p_detect_count >= DETECT_COUNT_THRESH and p_detect_count%DETECT_COUNT_THRESH == 0) or (no_person_count >= DETECT_COUNT_THRESH and no_person_count%DETECT_COUNT_THRESH == 0)):    # this is used to lessen the repeate hello-goodbye issue
                                                                      # anytime a person is there or not there, measure once in five counts
                p_detect, p_pos = person_position_1_hit(room_temp, temperature_array, servo_position)
                debugPrint('Person detect (p_detect): '+str(p_detect)+' Person position (p_pos): '+str(p_pos))

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
                    text = font.render("WARNING! Burn danger!", 1, name_to_rgb('yellow'))
                    textpos = text.get_rect()
                    textpos.center = message_area_xy
                    screen.blit(text, textpos)
# update the screen
                    pygame.display.update()

                play_sound(MAX_VOLUME, BURN_FILE_NAME)
                debugPrint('Played Burn warning audio')
                if CONNECTED:
                    try:
                        speakSpeechFromText("The temperature is "+"%.1f"%max(temperature_array)+" degrees fahrenheit", "mtemp.mp3")
                        play_sound(MAX_VOLUME, "mtemp.mp3")
                    except:
                        continue
                debugPrint('\r\n'+"Burn hazard temperature is "+"%.1f"%max(temperature_array)+" degrees")
                
                break

###########################
# Person Detected !
###########################
            elif p_detect :    # Here is where a person is detected
                roam_count = 0
                no_person_count = 0
                p_detect_count += 1
                LED_state = True
                GPIO.output(LED_GPIO_PIN, LED_state)
                CPUtemp = getCPUtemperature()
                debugPrint('\r\nPerson count: '+str(p_detect_count)+' Max: '+"%.1f"%max(temperature_array)+' Servo: '+str(servo_position)+' CPU: '+str(CPUtemp))

                if MONITOR:
                    screen.fill(name_to_rgb('white'), message_area)
                    text = font.render("Hello!", 1, name_to_rgb('red'))
                    textpos = text.get_rect()
                    textpos.center = message_area_xy
                    screen.blit(text, textpos)
# update the screen
                    pygame.display.update()

                if SERVO:
# face the servo twoards the heat
                    pid_controller.setPoint(p_pos)                      # setpoint is the desired position
                    pid_error = pid_controller.update(servo_position)         # process variable is current position
                    debugPrint('Des Pos: '+str(p_pos)+' Cur Pos: '+str(servo_position)+' PID Error: '+str(pid_error))

# make the robot turn its head to the person
# if previous error is the same absolute value as the current error, then we are oscillating - stop it
                    if abs(pid_error) > MINIMUM_ERROR_GRANULARITY:
                        previous_pid_error = pid_error
                        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                            servo_position += pid_error
                        else:
                            servo_position -= pid_error
                           
                        servo_position = set_servo_to_position(servo_position)
##                        play_sound(MAX_VOLUME, MOVE_FILE_NAME)
##                        if DEBUG:
##                            print 'Played move audio'
##                        if (pid_error < -100):
##                            play_sound(MAX_VOLUME, BADGE_FILE_NAME)
##                            if DEBUG:
##                                print 'Played badge audio'

                time.sleep(MEASUREMENT_WAIT_PERIOD*SETTLE_TIME)                 #let the temp's settle

                person = 1
                burn_hazard = 0
                break

###########################
# Nobody Detected !
###########################
            else:
                no_person_count += 1
                
                CPUtemp = getCPUtemperature()
                debugPrint('\r\nNo person count: '+str(no_person_count)+' Max: '+"%.1f"%max(temperature_array)+' Servo: '+str(servo_position)+' CPU: '+str(CPUtemp))

                if ((p_detect_count >= DETECT_COUNT_THRESH and p_detect_count%DETECT_COUNT_THRESH == 0) or (no_person_count >= DETECT_COUNT_THRESH and no_person_count%DETECT_COUNT_THRESH == 0)) :    # this is used to lessen the repeate hello-goodbye issue
                                                                      # anytime a person is there or not there, measure once in five counts
                    p_detect_count = 0
                    GPIO.output(LED_GPIO_PIN, False)
                    
                    if MONITOR:
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

                debugPrint('Servo Type: '+str(SERVO_TYPE)+' Servo position: '+str(servo_position)+' Servo direction: '+str(servo_direction)+' Roam count = '+str(roam_count))

                if SERVO and ROAM and roam_count <= ROAM_MAX:

                    roam_count += 1

                    if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                        if (servo_position <= SERVO_LIMIT_CCW and servo_direction == SERVO_CUR_DIR_CCW):
                            debugPrint('CCW limit hit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                    else:
                        if (servo_position >= SERVO_LIMIT_CCW and servo_direction == SERVO_CUR_DIR_CCW):
                            debugPrint('CCW limit hit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                        
                    if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                        if (servo_position >= SERVO_LIMIT_CW and servo_direction == SERVO_CUR_DIR_CW):
                            debugPrint('CW limit hit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CCW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                    else:
                        if (servo_position <= SERVO_LIMIT_CW and servo_direction == SERVO_CUR_DIR_CW):
                            debugPrint('CW limit hit, changing direction')
                            servo_direction = SERVO_CUR_DIR_CCW
                            #play_sound(MAX_VOLUME, BORED_FILE_NAME)
                        
                    debugPrint('Servo: Roaming. Position: '+str(servo_position))
                    if servo_direction == SERVO_CUR_DIR_CCW:
                        debugPrint(' Direction: CCW')
                        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                            servo_position -= ROAMING_GRANULARTY
                        else:
                            servo_position += ROAMING_GRANULARTY
                    if servo_direction == SERVO_CUR_DIR_CW:
                        debugPrint(' Direction: CW')
                        if SERVO_TYPE == LOW_TO_HIGH_IS_CLOCKWISE:
                            servo_position += ROAMING_GRANULARTY
                        else:
                            servo_position -= ROAMING_GRANULARTY
                      
                    if RAND:
                        servo_position = random.randint(MIN_SERVO_POSITION, MAX_SERVO_POSITION)
                        debugPrint('Random servo_position: '+str(servo_position))

                    debugPrint('Servo: Setting position to: '+str(servo_position))
                    servo_position = set_servo_to_position(servo_position)
                else:
#                    debugPrint('Roam count maximum reached; roaming stopped until person detected. Roam count = '+str(roam_count))
#                    debugPrint('LED_state = '+str(LED_state))
                    if (LED_state == False):
                        LED_state = True
#                        debugPrint('Turning LED on')
                        GPIO.output(LED_GPIO_PIN, LED_state)
                    else:
                        LED_state = False
#                        debugPrint('Turning LED off')
                        GPIO.output(LED_GPIO_PIN, LED_state)
                    time.sleep(0.5)
                    
                    
# End of inner While loop
            break

#############################
# End main while loop
#############################

        if fatal_error:
            logfile.write('\r\nFatal error at '+str(datetime.now()))
            break

        if person == 1:
            if person_existed_last_time == 0:           # person detected for the first time

                if MONITOR:
                    screen.fill(name_to_rgb('white'), message_area)
                    text = font.render("Hello!", 1, name_to_rgb('red'))
                    textpos = text.get_rect()
                    textpos.center = message_area_xy
                    screen.blit(text, textpos)
# update the screen
                    pygame.display.update()

                debugPrint('************************** Hello Person! **************************')

# Move head
#            if SERVO:
#               if DEBUG:
#                  print 'Servo: Facing Person'
#               servo.set_servo(SERVO_GPIO_PIN, CCW_HALF)
#               time.sleep(0.5)         # Wait for the temps to normalize

# Play "hello" sound effect
                play_sound(MAX_VOLUME, HELLO_FILE_NAME)
                debugPrint('Played hello audio')
                time.sleep(3)
                play_sound(MAX_VOLUME, AFTER_HELLO_FILE_NAME)
                debugPrint('Played after hello audio')

                person_existed_last_time = 1
                played_hello =1

                if CONNECTED:
                    try:
                        speakSpeechFromText("The room temperature is "+"%.1f"%room_temp+" degrees fahrenheit", "rtemp.mp3")
                        play_sound(MAX_VOLUME, "rtemp.mp3")

                        #speakSpeechFromText("and your temperature is "+"%.1f"%max(temperature_array)+" degrees fahrenheit", "mtemp.mp3")
                        #play_sound(MAX_VOLUME, "mtemp.mp3")
                    except:
                        continue

        else:
            if person_existed_last_time == 1:           # person moved away from the device
                if MONITOR:
                    screen.fill(name_to_rgb('white'), message_area)
                    text = font.render("Bye bye!", 1, name_to_rgb('red'))
                    textpos = text.get_rect()
                    textpos.center = message_area_xy
                    screen.blit(text, textpos)
# update the screen
                    pygame.display.update()

                debugPrint('************************** Bye Bye Person! **************************')

# Move head
#            if SERVO:
#               if DEBUG:
#                  print 'Servo: Facing AWAY'
#               servo.set_servo(SERVO_GPIO_PIN, CW_HALF)
#               time.sleep(0.5)         # Wait for the temps to normalize

# Play "bye bye" sound effect
                #byebye_message = random.choice(BYEBYE_FILE_NAME)
                play_sound(MAX_VOLUME, BADGE_FILE_NAME)
                debugPrint('Played badge audio')

                play_sound(MAX_VOLUME, GOODBYE_FILE_NAME)
                debugPrint('Played bye audio')

                played_byebye =1
                person_existed_last_time = 0

   # end if

# end of main loop

except KeyboardInterrupt:
    crash_msg = '\r\nKeyboard interrupt; quitting'
    crash_and_burn(crash_msg, pygame, servo, logfile)

except IOError:
    # do not close the logfile here - that allows the previous logfile to stay intact for a forensic analysis
    crash_msg = '\r\nI/O Error; quitting'
    debugPrint(crash_msg)
    if SERVO:
        servo.stop_servo(SERVO_GPIO_PIN)
    pygame.quit()
    sys.exit()


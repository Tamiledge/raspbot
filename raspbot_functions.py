
from datetime import datetime

# function for celcius to farenheiht conversion
def c2f (centigrade):
    """
    Convert Centigrade to Fahrenheit
    """
    return 9.0*centigrade/5.0 + 32

# function to get the average value of a list
def avg(incoming_list):
    """
    Calculates the average value of a list
    """
    return sum(incoming_list, 0.0) / len(incoming_list)

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

def downloadFile(url, fileName):
    fp = open(fileName, "wb")
    curl = pycurl.Curl()
    curl.setopt(pycurl.URL, url)
    curl.setopt(pycurl.WRITEDATA, fp)
    curl.perform()
    curl.close()
    fp.close()

def getGoogleSpeechURL(phrase):
    googleTranslateURL = \
        "http://translate.google.com/translate_tts?tl=en&"
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
    temp_str = fo.read()
    fo.close()
    temp_degC = float(temp_str)/1000
    temp_degF = c2f(temp_degC)
#    temp1 = temp_str[5:9]
#    temp2 = eval(temp1)
    return temp_degF

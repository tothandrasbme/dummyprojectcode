###
### export DISPLAY=:0 python pygame_example.py
###
import serial
import time
# Main thread
from threading import Thread
import inputs
import pygame       # Import for active input from keyboard
import os           # Import for
import signal       # Import for
import struct       # Import for
import socket       # Import for Socket server functionality
import select
import logging      # Logging for tests
import sys
import string
import pynmea2
import math
from state import sysState
from state import States
from state import ERRORS
from commonvariables import systemStateMachine
from jsonparser import JsonParserClass
import json
from jsonsocketservice import JSONSocketServer

global uartArduinoOK
global ser
global arDuinoSerialPort
global currentMotorRPM1
global currentMotorRPM2
global currentMotorRPM3
global targetMotorRPM1
global targetMotorRPM2
global targetMotorRPM3
global softFollower         # Flag for using softFoloower function with soft steps to reach the target rpm
global softStep             # steps for reaching the target rpm
global lowLimitValue        # minimum value in positive and negativ direction
global softTiming           # secs waiting between the soft steps

global leftValue
global rightValue
global rotate_value
global center_value
global GPPThread
global KBPThread
global MVSThread
global haltThreadFlag
global newSettingsForESC
global newStopESC
global newTestForwardESC
global newTestBackESC
global newTestByteESC
global motor1ByteToESC
global motor2ByteToESC
global motor3ByteToESC
global stepTestValue
global gamePadTestingCount
global gamePadTestingCountLimit
global startposition
global basestation
global steplist
global routeLoaded

#balint
global x                        # current x pos
global y                        # current y pos
global phi
global vx
global vy
global omega_z
global regulatorMode
global regFinished
global posPrecision
global orientPrecision
global coord_list
global finish_pos
global start_pos
global pos_state
global ori_state
global rotation_time
global deltaTime
global offsetTime



vx = 0
vy = 0
omega_z = 0
regFinished = False             # vegigert a pontsoron
posPrecision = 0.2              # 0.2 m
orientPrecision = math.pi/18    # 10°
pos_state = 0
ori_state = 0
phi = 0                         # nincs orientáció jel, kezdetben ebből indul a robot
deltaTime = 0.5                 # 1rad=180° megtételéhez szükséges idő
offsetTime = 0.1                # platform megmozdulásához szükséges idő


rotate_left = False
rotate_right = False
rotate_value = 1500
center_value = 5000
pads = inputs.devices.gamepads
currentCent = 0
currentSection = 0
leftValue = 1300
rightValue = 1300
gamePadTestingCount = 0
gamePadTestingCountLimit = 3

currentMotorRPM1 = center_value
currentMotorRPM2 = center_value
currentMotorRPM3 = center_value

targetMotorRPM1 = center_value
targetMotorRPM2 = center_value
targetMotorRPM3 = center_value
softFollower = True
softStep = 500
lowLimitValue = 650
softTiming = 0.5
newSettingsForESC = False
newTestForwardESC = False
newTestBackESC = False
newStopESC = False
newTestByteESC = False
stepTestValue = center_value
motor1ByteToESC = False
motor2ByteToESC = False
motor3ByteToESC = False


haltThreadFlag = False
uartArduinoOK = False


# GPS variables
global serGPS
global uartGPSOK

uartGPSOK = False

# Socket server variables

global sock
global haveSocketOpened
global client_sock

haveSocketOpened = False

# Log File variables

global logFileName
global isLogging
global dateTimeYear
global dateTimeMonth
global dateTimeDay
global dateTimeHour
global dateTimeMin
global dateTimeSec
global lat
global lon
global latTarget
global lonTarget
global altitude
global speed
global messageNumber
global fixedGPS
global currentGPSlat
global currentGPSlon

logFileName = "dummyPLogFile_0"
isLogging = False
messageNumber = 0
dateTimeYear = "0"
dateTimeMonth = "0"
dateTimeDay = "0"
dateTimeHour = "0"
dateTimeMin = "0"
dateTimeSec = "0"
lat = "0.0"
lon = "0.0"
latTarget = "47.68472"
lonTarget = "16.58306"
altitude = "0.0"
speed = "0.0"
fixedGPS = False
currentGPSlat = 46.711627
currentGPSlon = 17.479634

routeLoaded = False
steplist = list()
coord_list = list()

#### System state machine
## using it from common variables systemStateMachine = sysState(States.IDLE)  ## Set it to IDLE and inside the Class we set the default NOERROR Code

#### System JSON Parser
json_parser_tool = JsonParserClass()


class InitDelayClass:
    def run(self):
        global MVSThread
        global GPPThread
        global KBPThread
        global newStopESC

        print('Wait 1 secs for Keyboard...')

        time.sleep(1)

        KBPThread.start()

        print('Keyboard started! Wait 1 secs for Game Controller and Motor value setter...')

        time.sleep(1)

        #MVSThread.start()
        GPPThread.start()

        print('Game Controller and Motor value setter starting')

        #####STOP for testing #### newStopESC = True

class KeyboardPoller:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False


    def run(self):
        global arDuinoSerialPort
        global pads
        global rotate_value
        global targetMotorRPM1
        global targetMotorRPM2
        global targetMotorRPM3
        global currentMotorRPM1
        global currentMotorRPM2
        global currentMotorRPM3
        global haltThreadFlag
        global center_value
        global newSettingsForESC
        global newTestForwardESC
        global newTestBackESC
        global newStopESC
        global newTestByteESC
        global stepTestValue
        global center_value
        global motor1ByteToESC
        global motor2ByteToESC
        global motor3ByteToESC

        print('Keyboard poller has been started!!')

        while self._running and not haltThreadFlag:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.KEYDOWN:
                    #print("EventType:" + str(event.type) + " - Event key:" + str(event.key))
                    if event.key == pygame.K_LEFT:
                        print("Event: Go Left!")
                        targetMotorRPM1 = (-1 * rotate_value) + center_value
                        targetMotorRPM2 = (-1 * rotate_value) + center_value
                    if event.key == pygame.K_RIGHT:
                        print("Event: Go Right!")
                        targetMotorRPM1 = (rotate_value) + center_value
                        targetMotorRPM2 = (rotate_value) + center_value
                    if event.key == pygame.K_w:
                        print("Event: Go forward!")
                        targetMotorRPM1 = center_value + 1000
                        targetMotorRPM2 = center_value - 1000
                    if event.key == pygame.K_s:
                        print("Event: Go backward!")
                        targetMotorRPM1 = center_value - 1000
                        targetMotorRPM2 = center_value + 1000
                    if event.key == pygame.K_e:
                        print("Event: Go slow forward!")
                        targetMotorRPM1 = center_value + 1000
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = center_value - 1000
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = center_value
                        currentMotorRPM3 = targetMotorRPM3
                        newSettingsForESC = True
                    if event.key == pygame.K_d:
                        print("Event: Go slow backward!")
                        targetMotorRPM1 = center_value - 1000
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = center_value + 1000
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = center_value
                        currentMotorRPM3 = targetMotorRPM3
                        newSettingsForESC = True

                    if event.key == pygame.K_f:
                        print("Event: Rotate!")
                        targetMotorRPM1 = center_value + 750
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = center_value + 750
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = center_value + 750
                        currentMotorRPM3 = targetMotorRPM3
                        newSettingsForESC = True

                    if event.key == pygame.K_g:
                        print("Event: Rotate!")
                        targetMotorRPM1 = center_value - 750
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = center_value - 750
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = center_value - 750
                        currentMotorRPM3 = targetMotorRPM3
                        newSettingsForESC = True

                    if event.key == pygame.K_x:
                        print("Event: Stop it!")
                        targetMotorRPM1 = center_value
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = center_value
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = center_value
                        currentMotorRPM3 = targetMotorRPM3
                        newSettingsForESC = True

                    if event.key == pygame.K_r:
                        print("Event: Test Forward!")
                        newTestForwardESC = True
                    if event.key == pygame.K_f:
                        print("Event: Test Back!")
                        newTestBackESC = True
                    if event.key == pygame.K_v:
                        print("Event: Stop it at once!")
                        stepTestValue = center_value
                        newStopESC = True
                    if event.key == pygame.K_z:
                        print("Event: Check Bytes")
                        targetMotorRPM1 = center_value + 1000
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = center_value - 3000
                        currentMotorRPM2 = targetMotorRPM2
                        newTestByteESC = True
                    if event.key == pygame.K_u:
                        print("Event: Check Bytes for steps" + str(stepTestValue))
                        stepTestValue = stepTestValue + softStep
                        targetMotorRPM1 = stepTestValue
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = stepTestValue
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = stepTestValue
                        currentMotorRPM3 = targetMotorRPM3
                        newTestByteESC = True
                    if event.key == pygame.K_j:
                        print("Event: Check Bytes for steps" + str(stepTestValue))
                        stepTestValue = stepTestValue - softStep
                        targetMotorRPM1 = stepTestValue
                        currentMotorRPM1 = targetMotorRPM1
                        targetMotorRPM2 = stepTestValue
                        currentMotorRPM2 = targetMotorRPM2
                        targetMotorRPM3 = stepTestValue
                        currentMotorRPM3 = targetMotorRPM3
                        newTestByteESC = True
                    if event.key == pygame.K_k:
                        print("Event: Motor 1 speed up")
                        stepTestValue = stepTestValue + softStep
                        targetMotorRPM1 = stepTestValue
                        currentMotorRPM1 = targetMotorRPM1
                        motor1ByteToESC = True
                    if event.key == pygame.K_n:
                        print("Event: Motor 2 speed up")
                        stepTestValue = stepTestValue + softStep
                        targetMotorRPM2 = stepTestValue
                        currentMotorRPM2 = targetMotorRPM2
                        motor2ByteToESC = True
                    if event.key == pygame.K_o:
                        print("Event: Motor 3 speed up")
                        stepTestValue = stepTestValue + softStep
                        targetMotorRPM3 = stepTestValue
                        currentMotorRPM3 = targetMotorRPM3
                        motor3ByteToESC = True

                if event.type == pygame.KEYUP:
                    #print("EventType:" + str(event.type) + " - Event key:" + str(event.key))
                    if event.key == pygame.K_LEFT:
                        print("Event: Stop!")
                        targetMotorRPM1 = 0 + center_value
                        targetMotorRPM2 = 0 + center_value
                    if event.key == pygame.K_RIGHT:
                        print("Event: Stop!")
                        targetMotorRPM1 = 0 + center_value
                        targetMotorRPM2 = 0 + center_value
                    if event.key == pygame.K_w:
                        print("Event: Stop!")
                        targetMotorRPM1 = 0 + center_value
                        targetMotorRPM2 = 0 + center_value
                    if event.key == pygame.K_s:
                        print("Event: Stop!")
                        targetMotorRPM1 = 0 + center_value
                        targetMotorRPM2 = 0 + center_value


class GPSUARTConnection:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global serGPS
        global uartGPSOK
        global haltThreadFlag
        global currentGPSlat
        global currentGPSlon

        if uartGPSOK == True:
            while not haltThreadFlag:
                data = serGPS.readline()
                #print(data)
                message = data[0:6]
                if (message == "$GNRMC"):
                    # GNRMC = Recommended minimum specific GPS/Transit data
                    # Reading the GPS fix data is an alternative approach that also works
                    parts = data.split(",")
                    msg = pynmea2.parse(data)
                    print("Your position: lon = " + formatDegreesMinutes(str(msg.lon),3) + ", lat = " + formatDegreesMinutes(str(msg.lat),2))

                    currentGPSlat = formatDegreesMinutes(str(msg.lat),2)    # Save out lattitude
                    currentGPSlon = formatDegreesMinutes(str(msg.lon),3)    # Save out longitude
                else:
                    # Handle other NMEA messages and unsupported strings
                    pass
        else:
            time.sleep(5)
            self.initGPS()
            self.run()

    def initGPS(self):
        global uartGPSOK
        global serGPS
        # SIMEnFlag.on()
        time.sleep(10)
        print('OPENING serial port for GPS GSN')
        try:
            serGPS = serial.Serial(
                port='/dev/ttyS0',
                baudrate=38400,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            uartGPSOK = True
        except:
            # SIMEnFlag.off()
            writeLogFileError("GPS - Could not open Serial port for GPS")
            print("Error opening serial port 0")
        print('GPS GSN connected succesfully')

class GamePadPoller:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global arDuinoSerialPort
        global pads
        global rotate_value
        global targetMotorRPM1
        global targetMotorRPM2
        global targetMotorRPM3
        global currentMotorRPM1
        global currentMotorRPM2
        global currentMotorRPM3
        global haltThreadFlag
        global center_value
        global newSettingsForESC
        global newTestForwardESC
        global newTestBackESC
        global newStopESC
        global newTestByteESC
        global stepTestValue
        global center_value
        global motor1ByteToESC
        global motor2ByteToESC
        global motor3ByteToESC
        global gamePadTestingCount
        global gamePadTestingCountLimit


        #balint
        global vx
        global vy
        global omega_z
        global regulatorMode
        global regFinished

        print('GamePad controller has been started!!')

        while self._running and not haltThreadFlag:
            try:
                events = inputs.get_gamepad()
                for event in events:
#                 #print(event.ev_type, event.code, event.state)
#                 if event.code == 'BTN_TL' and event.state == 1:
#                     print('Rotate Left! ')
#                     targetMotorRPM1 = (-1 * rotate_value) + 5000
#                     targetMotorRPM2 = (-1 * rotate_value) + 5000
#                 if event.code == 'BTN_TL' and event.state == 0:
#                     print('Stop Rotate Left! (Now stop the motor)')
#                     targetMotorRPM1 = 0 + 5000
#                     targetMotorRPM2 = 0 + 5000
#                 if event.code == 'BTN_TR' and event.state == 1:
#                     print('Rotate Right! ')
#                     targetMotorRPM1 = rotate_value + 5000
#                     targetMotorRPM2 = rotate_value + 5000
#                 if event.code == 'BTN_TR' and event.state == 0:
#                     print('Stop Rotate Right! (Now stop the motor)')
#                     targetMotorRPM1 = 0 + 5000
#                     targetMotorRPM2 = 0 + 5000
#                 if event.code == 'ABS_Y':
#                     targetMotorRPM1 = ((event.state - 127) * 5) + 5000

                    if event.code == 'BTN_SELECT':
                        # print("Event: Stop it!")
                        vx = 0
                        vy = 0
                        omega_z = 0
                        # targetMotorRPM1 = center_value
                        # currentMotorRPM1 = targetMotorRPM1
                        # targetMotorRPM2 = center_value
                        # currentMotorRPM2 = targetMotorRPM2
                        # targetMotorRPM3 = center_value
                        # currentMotorRPM3 = targetMotorRPM3
                        newStopESC = True

                    else:
                        if event.code == 'ABS_RZ':
                            vx = 0.0
                            vy = 0.0
                            omega_z = 1.0
                            #print(event.ev_type, event.code, event.state)
                            #targetMotorRPM1 = 5500#(event.state * 3) + center_value
                            #targetMotorRPM2 = 5500#(event.state * 3) + center_value
                            #targetMotorRPM3 = 5500#(event.state * 3) + center_value

                            #currentMotorRPM1 = targetMotorRPM1
                            #currentMotorRPM2 = targetMotorRPM2
                            #currentMotorRPM3 = targetMotorRPM3
                            #print(str(currentMotorRPM1))
                            newSettingsForESC = True

                        if event.code == 'ABS_Z':
                            vx = 0.0
                            vy = 0.0
                            omega_z = -1.0
                            #print(event.ev_type, event.code, event.state)

                            newSettingsForESC = True
                        if event.code == 'ABS_RX':
                            vx = event.state/32768+0.0
                            omega_z=0.0
                            newSettingsForESC = True

                        if event.code == 'ABS_RY':
                            vy = event.state/32768+0.0
                            omega_z=0.0
                            newSettingsForESC = True

                        if event.code == 'ABS_HAT0X':
                            vx = event.state
                            vy= 0.0
                            omega_z=0.0
                            newSettingsForESC = True

                        if event.code == 'ABS_HAT0Y':
                            vx = 0.0
                            vy = event.state
                            omega_z=0.0
                            newSettingsForESC = True

            except:
                print('GamePad: Exception, there is no gamepad, but everything is going on without gamepad')
                gamePadTestingCount = gamePadTestingCount + 1
                if(gamePadTestingCount > gamePadTestingCountLimit):
                    self._running = False
                    systemStateMachine.setState(States.ERROR)
                    systemStateMachine.setErrorCode(ERRORS.MISSINGGAMEPAD)
                    print('GamePad: Stop checking')
                time.sleep(10)

class MotorValuesSetter:
    _running = True
    def __init__(self):
        global arDuinoSerialPort

    def run(self):
        global arDuinoSerialPort
        global softFollower  # Flag for using softFolower function with soft steps to reach the target rpm
        global softStep  # steps for reaching the target rpm
        global lowLimitValue  # minimum value in positive and negativ direction
        global softTiming  # secs waiting between the soft steps

        global currentMotorRPM1
        global currentMotorRPM2
        global currentMotorRPM3
        global targetMotorRPM1
        global targetMotorRPM2
        global targetMotorRPM3

        global haltThreadFlag
        global newSettingsForESC

        print('MotorValuesSetter has been started!!')

        while self._running and not haltThreadFlag:
            time.sleep(softTiming)
            #print("---> c:" + str(currentMotorRPM1) + " t:" + str(targetMotorRPM1))
            if softFollower == True:
                # Motor 1
                if currentMotorRPM1 > targetMotorRPM1 and currentMotorRPM1 - targetMotorRPM1 >= softStep:
                    currentMotorRPM1 = currentMotorRPM1 - softStep
                    newSettingsForESC = True
                    #print('Set Motor1 value : ' + str(currentMotorRPM1) + ' target - ' + str(targetMotorRPM1))
                else:
                    if currentMotorRPM1 < targetMotorRPM1 and targetMotorRPM1 - currentMotorRPM1 >= softStep:
                        currentMotorRPM1 = currentMotorRPM1 + softStep
                        newSettingsForESC = True
                        #print('Set Motor1 value : ' + str(currentMotorRPM1) + ' target - ' + str(targetMotorRPM1))

                # Motor 2
                if currentMotorRPM2 > targetMotorRPM2 and currentMotorRPM2 - targetMotorRPM2 >= softStep:
                    currentMotorRPM2 = currentMotorRPM2 - softStep
                    newSettingsForESC = True
                    #print('Set Motor2 value : ' + str(currentMotorRPM2) + ' target - ' + str(targetMotorRPM2))
                else:
                    if currentMotorRPM2 < targetMotorRPM2 and targetMotorRPM2 - currentMotorRPM2 >= softStep:
                        currentMotorRPM2 = currentMotorRPM2 + softStep
                        newSettingsForESC = True
                        #print('Set Motor2 value : ' + str(currentMotorRPM2) + ' target - ' + str(targetMotorRPM2))

                # Motor 3
                if currentMotorRPM3 > targetMotorRPM3 and currentMotorRPM3 - targetMotorRPM3 >= softStep:
                    currentMotorRPM3 = currentMotorRPM3 - softStep
                    newSettingsForESC = True
                    #print('Set Motor3 value : ' + str(currentMotorRPM3) + ' target - ' + str(targetMotorRPM3))
                else:
                    if currentMotorRPM3 < targetMotorRPM3 and targetMotorRPM3 - currentMotorRPM3 >= softStep:
                        currentMotorRPM3 = currentMotorRPM3 + softStep
                        newSettingsForESC = True
                        #print('Set Motor3 value : ' + str(currentMotorRPM3) + ' target - ' + str(targetMotorRPM3))

            # save the values and send the value to arduino in the main loop

    def stop(self):
        self._running = False

    #def sendValues(self):
        #   global leftValue
        #   global rightValue
        #   global arDuinoSerialPort
        #   arDuinoSerialPort.send("8 " + str(1300) + " " + str(rightValue) + " " + str(leftValue))

    def setMRPMs(self,m1rpm, m2rpm, m3rpm):
        global currentMotorRPM1
        global currentMotorRPM2
        global currentMotorRPM3
        global arDuinoSerialPort

        currentMotorRPM1 = m1rpm
        currentMotorRPM2 = m2rpm
        currentMotorRPM3 = m3rpm

        arDuinoSerialPort.send("M " + str(currentMotorRPM1) + " " + str(currentMotorRPM2) + " " + str(currentMotorRPM3))

    def setMRPM_With_Timing(self,m1rpm, m2rpm, m3rpm, step, timing):
        global currentMotorRPM1
        global currentMotorRPM2
        global currentMotorRPM3
        global targetMotorRPM1
        global targetMotorRPM2
        global targetMotorRPM3

        global softFollower  # Flag for using softFoloower function with soft steps to reach the target rpm
        global softStep  # steps for reaching the target rpm
        global lowLimitValue  # minimum value in positive and negativ direction
        global softTiming  # secs waiting between the soft steps


        global arDuinoSerialPort

        targetMotorRPM1 = m1rpm
        targetMotorRPM2 = m2rpm
        targetMotorRPM3 = m3rpm

        softStep = step
        softTiming = timing
        softFollower = True

        ### Remove the motor setter will do this ### arDuinoSerialPort.send("M " + str(currentMotorRPM1) + " " + str(currentMotorRPM2) + " " + str(currentMotorRPM3))


    def getMotorPARAMs(self,motorNum):
        global arDuinoSerialPort
        arDuinoSerialPort.send("R" + str(motorNum))
        # Todo: Get the params through serial port



class SerialPort:
    def __init__(self):
        global ser
        global uartArduinoOK
        self.initUART()

    def initUART(self):
        global uartArduinoOK
        global ser
        # print('OPENING serial port 0')
        try:
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=230400,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            uartArduinoOK = True
            print('Arduino connected succesfully')
        except:
            print("Error opening serial port for Arduino")


    def close(self):
        global ser
        global uartArduinoOK
        ''' Close the serial port.'''
        ser.close()
        uartArduinoOK = False

    def send(self, msg):
        global ser
        global uartArduinoOK
        if(uartArduinoOK):
            ser.write(msg)
            ser.flush
        #print("Send message to arDuino:" + msg)

    def recv(self):
        global ser
        global uartArduinoOK
        if(uartArduinoOK == True):
            return ser.readline()
        else:
            return ""

###
### Log file section
###
def initNewLogFile():
    global logFileName
    # init logging
    LOG_LEVEL = logging.INFO
    # LOG_LEVEL = logging.DEBUG
    setNextLogFileNumber()
    # LOG_FILE = "/dev/stdout"
    LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
    logging.basicConfig(filename=logFileName, format=LOG_FORMAT, level=LOG_LEVEL)

def closeLogFile():
    logging.shutdown()

def setNextLogFileNumber():
    global logFileName
    onlyfiles = [f for f in os.listdir("/home/pi") if os.path.isfile("/home/pi" + f)]

    #print("Find files in the directiory:")
    #print("Original file list: " + str(onlyfiles))
    if len(onlyfiles) != 0:
        numbersList = [0]
        for fileItem in onlyfiles:
            if fileItem.find("dummyPLogFile") >= 0:
                numbersList.append(fileItem[11:len(fileItem)])
                #print(fileItem[11:len(fileItem)])
        #print("Maximum: " + str(max(numbersList)))
        stringNumber = str(int(max(numbersList))+1)
        logFileName = "%s%s" % ("dummyPLogFile_", stringNumber)
    else:
        logFileName = "dummyPLogFile_0"

def writeLogFileITEM(msg):
    global isLogging
    if isLogging:
        logging.info(msg)


def writeINFOLogFile(msg):
    global isLogging
    if isLogging:
        writeLogFileITEM("INFO;" + str(int(time.time())) + ";" + str(messageNumber) + ";" + msg)

def writeGPSLogInfos():
    global dateTimeYear
    global dateTimeMonth
    global dateTimeDay
    global dateTimeHour
    global dateTimeMin
    global dateTimeSec
    global lat
    global lon
    global latTarget
    global lonTarget
    global altitude
    global speed
    global isLogging

    global messageNumber

    if isLogging:
        writeLogFileITEM("GPS;"+ str(int(time.time())) + ";" + str(messageNumber) + ";" + dateTimeYear + "." + dateTimeMonth + "." + dateTimeDay + ";" + dateTimeHour + ":" + dateTimeMin + ":" + dateTimeSec )
        messageNumber += 1

def getCoord(expression):
    global dateTimeYear
    global dateTimeMonth
    global dateTimeDay
    global dateTimeHour
    global dateTimeMin
    global dateTimeSec
    global lat
    global lon
    global altitude
    global speed

    global messageNumber
    # Start the serial connection
    if "+CGNSINF: 1," in expression:
        # Split the reading by commas and return the parts referencing lat and long
        array = expression.split(",")
        dateTimeYear = array[2][0:4]
        dateTimeMonth = array[2][4:6]
        dateTimeDay = array[2][6:8]
        dateTimeHour = array[2][8:10]
        dateTimeMin = array[2][10:12]
        dateTimeSec = array[2][12:14]

        lat = array[3]
        print(lat)
        lon = array[4]
        print(lon)

        altitude = array[5]
        speed = array[6]

        writeGPSLogInfos()

        print("Date " + dateTimeYear + "." + dateTimeMonth + "." + dateTimeDay + " --- " + dateTimeHour + ":" + dateTimeMin + ":" + dateTimeSec + " Alt : " + altitude + " Speed : " + speed)


def writeDATELogInfos():

    global messageNumber
    global recSettings
    global nodeDataBase
    global isLogging

    if isLogging == True:
        nodenum = 0
        resultstring = "DATE;"+ str(int(time.time())) + ";" + str(messageNumber)
        resultstring += ";" + str(recSettings.radioMode)
        resultstring += ";" + str(recSettings.deviceReset)
        resultstring += ";" + str(recSettings.trControl)
        resultstring += ";" + str(recSettings.numOfDevices)
        while nodenum < 3:
            #print(str(nodenum))
            resultstring += ";" + str(nodeDataBase[nodenum].txAddress)
            resultstring += ";" + str(nodeDataBase[nodenum].vBattery)
            resultstring += ";" + str(nodeDataBase[nodenum].mDecLDValue)
            resultstring += ";" + str(nodeDataBase[nodenum].tempDevice)
            # pprint(vars(nodeDataBase[nodeNum]))
            nodenum += 1
        while nodenum < 6:
            #print(str(nodenum))
            resultstring += ";" + str(nodeDataBase[nodenum].txAddress)
            resultstring += ";" + str(nodeDataBase[nodenum].vBattery)
            resultstring += ";" + str(nodeDataBase[nodenum].mDecWeigthMeasure)
            resultstring += ";" + str(nodeDataBase[nodenum].tempDevice)
            # pprint(vars(nodeDataBase[nodeNum]))
            nodenum += 1
        while nodenum < 9:
            #print(str(nodenum))
            resultstring += ";" + str(nodeDataBase[nodenum].txAddress)
            resultstring += ";" + str(nodeDataBase[nodenum].vBattery)
            resultstring += ";" + str(nodeDataBase[nodenum].mAccelX)
            resultstring += ";" + str(nodeDataBase[nodenum].mAccelY)
            resultstring += ";" + str(nodeDataBase[nodenum].mAccelZ)
            resultstring += ";" + str(nodeDataBase[nodenum].tempDevice)
            # pprint(vars(nodeDataBase[nodeNum]))
            nodenum += 1
        writeLogFileITEM(resultstring)
        messageNumber += 1

def writeLogFileError(msg):
    global isLogging
    if isLogging:
        writeLogFileITEM("ERROR;"+ str(int(time.time())) + ";" + str(messageNumber) + ";" + msg)


# In the NMEA message, the position gets transmitted as:
# DDMM.MMMMM, where DD denotes the degrees and MM.MMMMM denotes
# the minutes. However, I want to convert this format to the following:
# DD.MMMM. This method converts a transmitted string to the desired format
def formatDegreesMinutes(coordinates,longness):
    partsDD = coordinates[0:longness]
    partsMM = coordinates[longness:]

    print('DD: ' + partsDD + 'MM: ' + partsMM)

    floatPartsDD = float(partsDD)
    floatPartsMM = float(partsMM)

    return str(floatPartsDD + (floatPartsMM / 60))

## Pygame for controlling
#os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init()
pygame.display.init()

width, height = 64*10, 64*8
screen=pygame.display.set_mode((width, height))

# Create Class for connection to ArDuino
arDuinoSerialPort = SerialPort()

# Create Class for polling Game Pad controller
GPP = GamePadPoller()
# Create Thread
GPPThread = Thread(target=GPP.run)

# Create Class for polling Keyboard
KBP = KeyboardPoller()
# Create Thread
KBPThread = Thread(target=KBP.run)

MVS = MotorValuesSetter()
MVSThread = Thread(target=MVS.run)

# Create SocketServer Class for conctrol from GUI
SocketServerClass = JSONSocketServer()
# Create Thread for Socket server class
SocketServerThread = Thread(target=SocketServerClass.run)
# Start the created Thread for listening for the messages
SocketServerThread.start()

# Create GPS Connection Class for reading location
GPSConnection = GPSUARTConnection()
# Create Thread for reading
GPSConnectionThread = Thread(target=GPSConnection.run)
# Start Thread
GPSConnectionThread.start()

# We have to delay a bit at the beginning for the game controller
InitDC = InitDelayClass()
InitDCThread = Thread(target=InitDC.run())
InitDCThread.start()
print(pads)
if len(pads) == 0:
    #raise Exception("Couldn't find any Gamepads!")
    print ("Couldn't find any Gamepads!")

Exit = False  # Exit flag
def signal_handler(sig, frame):
    global haltThreadFlag
    print('You pressed Ctrl+C! Wait 3 sec for exiting')
    haltThreadFlag = True
    SocketServerClass.terminate();
    time.sleep(3)
    print('Exiting...')
    # sys.exit(0)
    os._exit(1)
signal.signal(signal.SIGINT, signal_handler)

#Init log file for testing purposes
initNewLogFile()

# Commented for better performance # print('Waiting for data')
while Exit==False:
    state = systemStateMachine.getState()
    #print(arDuinoSerialPort.recv())  # Commented for better performance


    if systemStateMachine.isIDLE():

        vx = 0
        vy = 0
        omega_z = 0
        targetMotorRPM1 = center_value
        targetMotorRPM2 = center_value
        targetMotorRPM3 = center_value

        currentMotorRPM1 = int(targetMotorRPM1)
        currentMotorRPM2 = int(targetMotorRPM2)
        currentMotorRPM3 = int(targetMotorRPM3)
        arDuinoSerialPort.send("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))

    if systemStateMachine.isUnderCONTROL():
        while(systemStateMachine.isUnderCONTROL()):
            #arDuinoSerialPort.send("M 05000 05000 05000")
            if newSettingsForESC:
                print('Main M')
                arDuinoSerialPort.send("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))
                newSettingsForESC = False

            if newTestForwardESC:
                arDuinoSerialPort.send("F")
                print("Send Forward")
                newTestForwardESC = False

            if newTestBackESC:
                arDuinoSerialPort.send("B")
                print("Send Backward")
                newTestBackESC = False

            if newStopESC:
                arDuinoSerialPort.send("S")
                print("Send Stop")
                newStopESC = False

            if newTestByteESC:
                print('Main Z')
                arDuinoSerialPort.send(['Z', currentMotorRPM1 / 256, currentMotorRPM1 - ((currentMotorRPM1 / 256) * 256), currentMotorRPM2 / 256, currentMotorRPM2 - ((currentMotorRPM2 / 256) * 256), currentMotorRPM3 / 256, currentMotorRPM3 - ((currentMotorRPM3 / 256) * 256)])
                newTestByteESC = False

            if motor1ByteToESC:
                #print('Main K')
                arDuinoSerialPort.send(['K', currentMotorRPM1 / 256, currentMotorRPM1 - ((currentMotorRPM1 / 256) * 256)])
                motor1ByteToESC = False

            if motor2ByteToESC:
                #print('Main N')
                arDuinoSerialPort.send(['N', currentMotorRPM2 / 256, currentMotorRPM2 - ((currentMotorRPM2 / 256) * 256)])
                print("h1 : " + hex(currentMotorRPM2 / 256) + " h2: " + hex(currentMotorRPM2 - ((currentMotorRPM2 / 256) * 256)))
                #arDuinoSerialPort.send(['N', 0x17,0x70])
                motor2ByteToESC = False

            if motor3ByteToESC:
                #print('Main O')
                #arDuinoSerialPort.send(['N', 0x1B, 0x58])
                arDuinoSerialPort.send(['O', currentMotorRPM3/256,currentMotorRPM3-((currentMotorRPM3/256)*256)])
                motor3ByteToESC = False

    if systemStateMachine.isDRIVING():
        while(systemStateMachine.isDRIVING()):

            #temporary solution:
            #need to get real relative position coordinates
            x = 0
            y = 0

            if (start_pos[0] - pos_precision < x < start_pos[0] + pos_precision) and (start_pos[1] - pos_precision < y < start_pos[1] + pos_precision):
                print('Current position is the start position, starting...')
            else:
                print('Current position is NOT the start position')
                systemStateMachine.setState(States.ERROR)
                systemStateMachine.setErrorCode(ERRORS.NOTINSTARTPOS)
            if (finish_pos[0] - pos_precision < x < finish_pos[0] + pos_precision) and (finish_pos[1] - pos_precision < y < finish_pos[1] + pos_precision):
                print('Driving finished - platform reached the desired location')
                print('IDLE state')
                systemStateMachine.setState(States.IDLE)

            if pos_state < len(coord_list):

                # target coordinates
                xf = coord_list[pos_state][0]
                yf = coord_list[pos_state][1]
                phif = coord_list[ori_state][2]

                if ori_state > pos_state:
                    routeMode = coord_list[pos_state][5]
                elif ori_state <= pos_state:
                    routeMode = coord_list[ori_state][5]


            elif pos_state == len(coord_list):
                pos_state = int(pos_state)
                ori_state = int(ori_state)
                xf = finish_pos[0]
                yf = finish_pos[1]
                phif = coord_list[ori_state][2]
                routeMode = coord_list[ori_state][5]


            dx = xf - x
            dy = yf - y
            dphi = phif - phi
            s = (dx ** 2 + dy ** 2) ** (1 / 2)

            rotation_time = offsetTime + dphi * deltaTime

            if routeMode == 0:

                if pos_state == ori_state:

                    if s > precision:
                        v = coord_list[pos_state][3]
                        omega_z = 0.0

                    else:
                        v = 0
                        omega_z = 0.0
                        pos_state = pos_state + 1


                elif pos_state > ori_state:
                    v = 0

                    if dphi > 0.08 or dphi < - 0.08:
                        omega_z = coord_list[ori_state][4]
                    else:
                        omega_z = 0.0
                        ori_state += 1
                        phi = phif


            elif routeMode == 1:

                if s > 0.02:
                    v = coord_list[pos_state][3]

                else:
                    v = 0

                if dphi > 0.08 or dphi < - 0.08:
                    omega_z = coord_list[ori_state][4]
                else:
                    omega_z = 0.0

                if s < 0.02 and - 0.08 < dphi < 0.08:
                    ori_state += 1
                    pos_state += 1
                    phi = phif

            if dy > 0:
                phi_v_x = math.acos(dx / s)
            elif dy < 0:
                phi_v_x = -math.acos(dx / s)
            elif dy == 0:
                phi_v_x = math.acos(dx / s)

            vx = v * math.cos(phi_v_x);
            vy = v * math.sin(phi_v_x);

            targetMotorRPM1 = center_value + 1 / 0.1 * (-math.sin(phi) * vx + math.cos(phi) * vy + 0.3 * 3 * omega_z) * 100
            targetMotorRPM2 = center_value + 1 / 0.1 * (-math.sin(phi + math.pi * 2 / 3) * vx + math.cos(phi + math.pi * 2 / 3) * vy + 0.3 * 3 * omega_z) * 100
            targetMotorRPM3 = center_value + 1 / 0.1 * (-math.sin(phi + math.pi * 4 / 3) * vx + math.cos(phi + math.pi * 4 / 3) * vy + 0.3 * 3 * omega_z) * 100

            currentMotorRPM1 = int(targetMotorRPM1)
            currentMotorRPM2 = int(targetMotorRPM2)
            currentMotorRPM3 = int(targetMotorRPM3)

            if (omega_z == 0):
                arDuinoSerialPort.send("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))
            elif (omega_z != 0):
                arDuinoSerialPort.send("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))

                time.sleep(rotation_time)

                targetMotorRPM1 = center_value
                targetMotorRPM2 = center_value
                targetMotorRPM3 = center_value

                currentMotorRPM1 = int(targetMotorRPM1)
                currentMotorRPM2 = int(targetMotorRPM2)
                currentMotorRPM3 = int(targetMotorRPM3)
                arDuinoSerialPort.send("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))

    if systemStateMachine.isInERROR():
        while (systemStateMachine.isInERROR()):
            targetMotorRPM1 = center_value
            targetMotorRPM2 = center_value
            targetMotorRPM3 = center_value

            currentMotorRPM1 = int(targetMotorRPM1)
            currentMotorRPM2 = int(targetMotorRPM2)
            currentMotorRPM3 = int(targetMotorRPM3)

            arDuinoSerialPort.send("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))

    # Todo: Check in every cycle for feedback from ESC
print('End of Motor Controller application')


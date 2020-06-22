###
### export DISPLAY=:0 python pygame_example.py
###
import serial
import time
# Main thread
from threading import Thread
import inputs
import pygame
import os
import signal
import struct

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


rotate_left = False
rotate_right = False
rotate_value = 1500
center_value = 5000
pads = inputs.devices.gamepads
currentCent = 0
currentSection = 0
leftValue = 1300
rightValue = 1300

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

        newStopESC = True

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

class SocketServer:
    global sock
    """ Simple socket server that listens to one single client. """

    def __init__(self, host='0.0.0.0', port=2010):
        global sock
        """ Initialize the server with a host and port to listen to. """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = host
        self.port = port
        sock.bind((host, port))
        sock.listen(1)
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global haveSocketOpened
        global haltThreadFlag
        while self._running and not haltThreadFlag:
            self.run_server()
            haveSocketOpened = False
            time.sleep(5)
            self.close()
            time.sleep(5)
            self.__init__()
        print('Exiting Socket Server')

    def close(self):
        global sock
        """ Close the server socket. """
        print('Closing server socket (host {}, port {})'.format(self.host, self.port))
        if sock:
            sock.close()
            sock = None

    def sendSocketMessage(self, message):
        global sock
        global haveSocketOpened
        global client_sock
        # print('Send Socket message : ')
        try:
            if haveSocketOpened:
                client_sock.send(message)
        except:
            writeLogFileError("Socket server - error while sending messages")
            print("Error sending Socket message")

    def run_server(self):
        global sock
        global client_sock
        global haveSocketOpened
        global haltThreadFlag
        """ Accept and handle an incoming connection. """
        print('Starting socket server (host {}, port {})'.format(self.host, self.port))
        client_sock, client_addr = sock.accept()
        print('Client {} connected'.format(client_addr))
        haveSocketOpened = True
        stop = False
        while not stop and not haltThreadFlag:
            if client_sock:
                # Check if the client is still connected and if data is available:
                try:
                    rdy_read, rdy_write, sock_err = select.select([client_sock, ], [], [])
                except select.error:
                    print('Select() failed on socket with {}'.format(client_addr))
                    writeLogFileError("Socket server - Select() failed on socket with {}")
                    return 1
                if len(rdy_read) > 0:
                    try:
                        read_data = client_sock.recv(255)
                        # Check if socket has been closed
                        if len(read_data) == 0:
                            print('{} closed the socket.'.format(client_addr))
                            stop = True
                            haveSocketOpened = False
                        else:
                            print('>>> Received: {}'.format(read_data.rstrip()))
                            if read_data.rstrip() == 'quit':
                                stop = True
                                haveSocketOpened = False
                            else:
                                client_sock.send(read_data)

                    except ConnectionResetError:
                        print('Client has disconnected while reading from the socket, go back to listening')
                        stop = True
                        haveSocketOpened = False
            else:
                print("No client is connected, SocketServer can't receive data")
                stop = True
                haveSocketOpened = False
        # Close socket
        print('Closing connection with {}'.format(client_addr))
        client_sock.close()
        haveSocketOpened = True
        return 0


class GPSUARTConnection:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        global serGPS
        global uartGPSOK
        global haltThreadFlag

        if uartGPSOK == True:
            try:
                time.sleep(1)
                serialmessage = str.encode('AT\r\n')
                serGPS.write(serialmessage)
                time.sleep(2)
                #print("Read from GPS:" + serialmessage)
                x = serGPS.readline()
                #print("Feedback from GPS : " + x)
                writeINFOLogFile("GPS - AT:" + x)
                SocketServerClass.sendSocketMessage(x)
                time.sleep(2)
                x = serGPS.readline()
                #print(x)
                writeINFOLogFile("GPS - AT2:" + x)
                SocketServerClass.sendSocketMessage(x)

                time.sleep(1)
                serialmessage = str.encode('AT+CGNSPWR=1\r\n')
                serGPS.write(serialmessage)
                time.sleep(2)
                x = serGPS.readline()
                #print(x)
                writeINFOLogFile("GPS - AT+CGNSPWR=1:" + x)
                SocketServerClass.sendSocketMessage(x)
                time.sleep(2)
                x = serGPS.readline()
                #print(x)
                writeINFOLogFile("GPS - AT+CGNSPWR2:" + x)
                SocketServerClass.sendSocketMessage(x)



                while not haltThreadFlag:
                    time.sleep(1)
                    serialmessage = str.encode('AT+CGNSINF\r\n')
                    serGPS.write(serialmessage)
                    time.sleep(2)
                    x = serGPS.readline()
                    #print(x)
                    #writeINFOLogFile("GPS - AT+CGNSINF1:" + x)
                    SocketServerClass.sendSocketMessage(x)
                    time.sleep(2)
                    x = serGPS.readline()
                    #print(x)
                    #writeINFOLogFile("GPS - AT+CGNSINF2:" + x)
                    getCoord(x)
                    SocketServerClass.sendSocketMessage(x)
            except Exception as e:
                print("Error using GPS serial port - " + str(e))
                writeLogFileError("ERROR;GPS error while using serial port for GPS:" + str(e))
                serGPS.close()
                uartGPSOK = False
        else:
            time.sleep(5)
            self.initGPS()
            self.run()

    def initGPS(self):
        global uartGPSOK
        global serGPS
        # SIMEnFlag.on()
        time.sleep(1)
        print('OPENING serial port for GPS GSN')
        try:
            serGPS = serial.Serial(
                port='/dev/ttyS0',
                baudrate=115200,
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

    def sendControlSignal(self,rpm,direction,motor): # 0 - forward, 1 - backward
        global currentCent
        global arDuinoSerialPort
        centWhich = rpm - (rpm % 100)
        if centWhich != currentCent:
            currentCent = centWhich
            if motor == "L":  # this for the motor Left
                controlType = "9 "
            else:
                if motor == "R":
                    controlType = "A "
            if direction == 0:
                arDuinoSerialPort.send(controlType + str(currentCent + 450))
                print("Rise speed to rpm (" + motor + "): " + str(currentCent + 450))
            else:
                arDuinoSerialPort.send(controlType + str((-1) * (currentCent + 450)))
                print("Rise speed to rpm (" + motor + "): " + str((-1) * (currentCent + 450)))

    def sendPPMControlSignal(self,measurement,motor):
        global currentSection
        global arDuinoSerialPort
        #print("Controller value: " + str(measurement))
        sectionWhich = measurement - (measurement % 200)
        if sectionWhich != currentSection:
            currentSection = sectionWhich
            if motor == "L":  # this for the motor Left
                controlType = "7 "
            else:
                if motor == "R":
                    controlType = "6 "
            arDuinoSerialPort.send(controlType + str(currentSection))
            print("Rise speed to ppm (" + motor + "): " + str(currentSection))



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

        print('GamePad controller has been started!!')

        while self._running and not haltThreadFlag:
            print('Running')
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
                    
                if event.code == 'ABS_RZ':
                    print(event.ev_type, event.code, event.state)
                    targetMotorRPM1 = (event.state * 3) + center_value
                    targetMotorRPM2 = (event.state * 3) + center_value
                    targetMotorRPM3 = (event.state * 3) + center_value
                    
                    currentMotorRPM1 = targetMotorRPM1
                    currentMotorRPM2 = targetMotorRPM2
                    currentMotorRPM3 = targetMotorRPM3
                    print(str(currentMotorRPM1))
                    newSettingsForESC = True
                    
                if event.code == 'ABS_Z':
                    print(event.ev_type, event.code, event.state)
                    targetMotorRPM1 = -(event.state * 3) + center_value
                    targetMotorRPM2 = -(event.state * 3) + center_value
                    targetMotorRPM3 = -(event.state * 3) + center_value
                    
                    currentMotorRPM1 = targetMotorRPM1
                    currentMotorRPM2 = targetMotorRPM2
                    currentMotorRPM3 = targetMotorRPM3
                    newSettingsForESC = True
                    
                if event.code == 'BTN_SELECT': 
                    print("Event: Stop it!")
                    targetMotorRPM1 = center_value
                    currentMotorRPM1 = targetMotorRPM1
                    targetMotorRPM2 = center_value
                    currentMotorRPM2 = targetMotorRPM2
                    targetMotorRPM3 = center_value
                    currentMotorRPM3 = targetMotorRPM3
                    newSettingsForESC = True

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
            print('Arduino connected succesfully')
        except:
            print("Error opening serial port for Arduino")


    def close(self):
        global ser
        ''' Close the serial port.'''
        ser.close()

    def send(self, msg):
        global ser
        ser.write(msg)
        ser.flush
        #print("Send message to arDuino:" + msg)

    def recv(self):
        global ser
        return ser.readline()

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
SocketServerClass = SocketServer()
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
    time.sleep(3)
    print('Exiting...')
    # sys.exit(0)
    os._exit(1)
signal.signal(signal.SIGINT, signal_handler)
# Commented for better performance # print('Waiting for data')
while Exit==False:

    #print(arDuinoSerialPort.recv())  # Commented for better performance

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



    #print("M 0" + str(currentMotorRPM1) + " 0" + str(currentMotorRPM2) + " 0" + str(currentMotorRPM3))

    # Todo: Check in every cycle for feedback from ESC
print('End of Motor Controller application')


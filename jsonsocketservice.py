import socket
import select
import json
from jsonparser import JsonParserClass
import time

class JSONSocketServer:
    global sock
    """ Simple socket server that listens to one single client. """

    def __init__(self,host='0.0.0.0', port=2010):
        global sock
        """ Initialize the server with a host and port to listen to. """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = host
        self.port = port
        sock.bind((host, port))
        sock.listen(1)
        self._running = True
        self.json_parser_tool = JsonParserClass()

    def terminate(self):
        self._running = False

    def run(self):
        global haveSocketOpened
        while self._running:
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
                client_sock.send(message.encode())
        except:
            writeLogFileError("Socket server - error while sending messages")
            print("Error sending Socket message")

    def run_server(self):
        global sock
        global client_sock
        global haveSocketOpened
        """ Accept and handle an incoming connection. """
        print('Starting socket server (host {}, port {})'.format(self.host, self.port))
        client_sock, client_addr = sock.accept()
        print('Client {} connected'.format(client_addr))
        haveSocketOpened = True
        stop = False
        while not stop and self._running:
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
                        #finalMessageContent = ""
                        #read_data = client_sock.recv(255)
                        #while read_data[len(read_data)-1] != '_':
                        #for x in range(0, 10):
                        #    finalMessageContent += str(read_data)
                        #    read_data = client_sock.recv(255)
                        #    print("Get one package of message:" + str(finalMessageContent))

                        read_data = client_sock.recv(4000)
                        finalMessageContent = read_data

                        # Check if socket has been closed
                        if len(finalMessageContent) == 0:
                            print('{} closed the socket.'.format(client_addr))
                            stop = True
                            haveSocketOpened = False
                        else:
                            #print('>>> Received: {}'.format(finalMessageContent.rstrip()))
                            if finalMessageContent.rstrip() == 'quit':
                                stop = True
                                haveSocketOpened = False
                            else:
                                ### client_sock.send(read_data) ### Test with loop back
                                #print("Parse this : " + str(finalMessageContent[2:].decode("utf-8")))
                                try:
                                    json_content = json.loads(str(finalMessageContent[2:].decode("utf-8")))
                                    print("Json - message type: " + str(json_content["type"]))
                                    if(str(json_content["type"]) == "MOVEMENTCONTROL"):
                                        self.json_parser_tool.parse_movement_control_message(json_content)
                                        print("Send back OK message to the socket server")
                                        #self.sendSocketMessage("Moveing control message - OK\r\n")
                                        self.sendSocketMessage("{\"optional\": \"NaN\", \"state\": \"OK\", \"type\": \"MOVEMENTCONTROL\"}\r\n")
                                    else:
                                        if(str(json_content["type"]) == "STATECONTROL"):
                                           self.json_parser_tool.parse_state_control_message(json_content)
                                           #self.sendSocketMessage("State control message - OK\r\n")
                                           self.sendSocketMessage("{\"optional\": \"NaN\", \"state\": \"OK\", \"type\": \"STATECONTROL\"}\r\n")
                                        else:
                                            if (str(json_content["type"]) == "STEPLIST"):
                                                self.json_parser_tool.parse_steps_message(json_content)
                                                # self.sendSocketMessage("State control message - OK\r\n")
                                                self.sendSocketMessage(
                                                    "{\"optional\": \"NaN\", \"state\": \"OK\", \"type\": \"STEPLIST\"}\r\n")
                                            print("MOve on")
                                except Exception as f:
                                    print(
                                        'Issue during json parsing (' + str(
                                            f) + ") !")

                    except Exception as e:
                        print('Client has disconnected while reading from the socket, go back to listening (' + str(e)+ ") !")
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
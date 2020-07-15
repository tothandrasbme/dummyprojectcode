from jsonsocketservice import JSONSocketServer

class JsonParserClass:
    def __init__(self):
        print("JSONparser initialized")
        this.socketServerInstance = socketServerInstance

    def parse_movement_control_message(self,json_content):
        print("Parse movement control message"+str(json_content))
        ## Check action
        ### move,stop,pause,goback
        ###
        if(str(json_content["action"]) == "move"):
            print("Action:move")
        else:
            if(str(json_content["action"]) == "stop"):
               print("Action:stop")
            else:
                if(str(json_content["action"]) == "pause"):
                    print("Action:pause")
                else:
                    if (str(json_content["action"]) == "goback"):
                        print("Action:goback")
                    else:
                        print("Action:unknown")

    def parse_state_control_message(self,json_content):
        print("Parse state control message"+str(json_content))
        ## Check state - ERROR - IDLE - CONTROLLED - DRIVING
        if (str(json_content["state"]) == "IDLE"):
            print("State: IDLE")
        else:
            if (str(json_content["state"]) == "CONTROLLED"):
                print("State: CONTROLLED")
            else:
                if (str(json_content["state"]) == "DRIVING"):
                    print("State: DRIVING")
                else:
                    if (str(json_content["state"]) == "ERROR"):
                        print("State: ERROR")
                        print("Error code: " + str(json_content["optional"]))
                    else:
                        print("State: UNKNOWN")

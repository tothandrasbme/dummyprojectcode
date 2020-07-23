from routestep import routeStepClass
from state import sysState
from state import States
from state import ERRORS
from commonvariables import systemStateMachine

class JsonParserClass:
    coord_list_in_parser = []
    def __init__(self):
        print("JSONparser initialized")
        self.coord_list_in_parser = []

    def parse_movement_control_message(self,json_content):
        print("Parse movement control message"+str(json_content))
        ## Check action
        ### move,stop,pause,goback
        ###
        if(str(json_content["action"]) == "move"):
            print("Action:move")
            systemStateMachine.setState(States.DRIVING)
        else:
            if(str(json_content["action"]) == "stop"):
               print("Action:stop")
               systemStateMachine.setState(States.IDLE)
            else:
                if(str(json_content["action"]) == "pause"):
                    print("Action:pause")
                    systemStateMachine.setState(States.IDLE)
                else:
                    if (str(json_content["action"]) == "goback"):
                        print("Action:goback")
                        systemStateMachine.setState(States.DRIVING)
                    else:
                        print("Action:unknown")
                        systemStateMachine.setState(States.ERROR)
                        systemStateMachine.setErrorCode(ERRORS.UNKNOWNCOMMAND)

    def parse_state_control_message(self,json_content):

        print("Parse state control message"+str(json_content))
        ## Check state - ERROR - IDLE - CONTROLLED - DRIVING
        if (str(json_content["state"]) == "IDLE"):
            print("State: IDLE")
            systemStateMachine.setState(States.IDLE)
        else:
            if (str(json_content["state"]) == "CONTROLLED"):
                print("State: CONTROLLED")
                systemStateMachine.setState(States.CONTROLLED)
            else:
                if (str(json_content["state"]) == "DRIVING"):
                    print("State: DRIVING")
                    systemStateMachine.setState(States.DRIVING)
                else:
                    if (str(json_content["state"]) == "ERROR"):
                        print("State: ERROR")
                        print("Error code: " + str(json_content["optional"]))
                        systemStateMachine.setState(States.ERROR)
                        systemStateMachine.setErrorCode(ERRORS.UNKNOWNCOMMAND) ## TODO: create parsing step for string version of the error code.
                    else:
                        print("State: UNKNOWN")
                        systemStateMachine.setState(States.ERROR)
                        systemStateMachine.setErrorCode(ERRORS.UNKNOWNCOMMAND)

    def parse_steps_message(self,json_content):
        global startposition
        global basestation
        global steplist
        global routeLoaded
        global coord_list

        coord_list = list()

        # Set start position parameters
        startposition = routeStepClass(json_content["optional"]["startposition"])
        coord_list.append(startposition.getStepParametersList())
        # Set basestation parameters
        basestation = routeStepClass(json_content["optional"]["basestation"])
        coord_list.append(basestation.getStepParametersList())
        # Set steps parameters
        steplist = []
        for i in range(0, len(json_content["optional"]["mainsteps"])):
            newStep = routeStepClass(json_content["optional"]["mainsteps"][i])
            steplist.append(newStep)
            coord_list.append(newStep.getStepParametersList())
        routeLoaded = True


        print("Parse steps message  -" + str((steplist[0]).getPosX()))
        print("Parse steps message  --" + str((steplist[0]).getTransitMode()))
        print("Parse steps message  ---" + str(coord_list))
        ## Check action
        ### move,stop,pause,goback
        ### {
        #       'action': 'LOAD',
        #       'optional':
        #           {
        #           'startposition':
        #               [
        #                   {'px': '625.0'},
        #                   {'py': '147.0'},
        #                   {'tphi': '181.56222491684238'},
        #                   {'msp': '0.0'},
        #                   {'mo': '0.0'},
        #                   {'trm': 'softStartStraightAndRotate'},
        #                   {'stc': '0'},
        #                   {'tpx': '645'},
        #                   {'tpy': '167'},
        #                   {'cpx': '610'},
        #                   {'cpy': '132'},
        #                   {'t': 'startState'}
        #               ],
        #           'basestation':
        #               [
        #                   {'px': '695.0'},
        #                   {'py': '136.0'},
        #                   {'tphi': '0.0'},
        #                   {'msp': '0.0'},
        #                   {'mo': '0.0'},
        #                   {'trm': 'softStartStraightAndRotate'},
        #                   {'stc': '0'},
        #                   {'tpx': '715'},
        #                   {'tpy': '156'},
        #                   {'cpx': '680'},
        #                   {'cpy': '121'},
        #                   {'t': 'baseStation'}],
        #           'type': 'STEPS',
        #           'mainsteps':
        #               [
        #                   [
        #                       {'px': '451.0'},
        #                       {'py': '148.0'},
        #                       {'tphi': '180.59680945122915'},
        #                       {'msp': '2.0'},
        #                       {'mo': '2.0'},
        #                       {'trm': 'straightAndRotate'},
        #                       {'stc': '0'},
        #                       {'tpx': '471'},
        #                       {'tpy': '168'},
        #                       {'cpx': '436'},
        #                       {'cpy': '133'},
        #                       {'t': 'movementStep'}
        #                    ],
        #                    [
        #                       {'px': '324.0'},
        #                       {'py': '161.0'},
        #                       {'tphi': '278.13010235415595'},
        #                       {'msp': '2.0'},
        #                       {'mo': '2.0'},
        #                       {'trm': 'straightAndRotate'},
        #                       {'stc': '1'},
        #                       {'tpx': '344'},
        #                       {'tpy': '181'},
        #                       {'cpx': '309'},
        #                       {'cpy': '146'},
        #                       {'t': 'movementStep'}
        #                    ],
        #                    [
        #                       {'px': '327.0'},
        #                       {'py': '296.0'},
        #                       {'tphi': '274.86451443776053'},
        #                       {'msp': '2.0'},
        #                       {'mo': '2.0'},
        #                       {'trm': 'straightAndRotate'},
        #                       {'stc': '2'},
        #                       {'tpx': '347'},
        #                       {'tpy': '316'},
        #                       {'cpx': '312'},
        #                       {'cpy': '281'},
        #                       {'t': 'movementStep'}
        #                     ],
        #                     [
        #                       {'px': '333.0'},
        #                       {'py': '441.0'},
        #                       {'tphi': '264.02567671351943'},
        #                       {'msp': '2.0'},
        #                       {'mo': '2.0'},
        #                       {'trm': 'straightAndRotate'},
        #                       {'stc': '3'},
        #                       {'tpx': '353'},
        #                       {'tpy': '461'},
        #                       {'cpx': '318'},
        #                       {'cpy': '426'},
        #                       {'t': 'movementStep'}
        #                     ]
        #                ]
        #           },
        #      'type': 'STEPLIST'}




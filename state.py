import enum

    #####
    ## IDLE
    ## ERROR
    ## CONTROLLED (robot mozgasban van es SOCKETEN-n vagy gamepaddel stb vez.)
    ## DRIVING (szab modban van, az adott pontsoron halad vegig)
    #####

class States(enum.Enum):
   IDLE = 1
   ERROR = 2
   CONTROLLED = 3
   DRIVING = 4

class ERRORS(enum.Enum):
    NOERROR = 0
    MISSINGNETWORK = 1
    MISSINGGPS = 2
    NONEXTSTEP = 3
    STOPPEDBYUSER = 4
    NOTINSTARTPOS = 5
    MISSINGGAMEPAD = 6
    UNKNOWNCOMMAND = 7

class sysState:
    def __init__(self,state):
        self.currentState = state
        self.errorCode = ERRORS.NOERROR

    def getState(self):
        return self.currentState

    def getErrorCode(self):
        return self.errorCode

    def setErrorCode(self, errorCode):
        self.errorCode = errorCode

    def setState(self,state):
        self.currentState = state

    def isIDLE(self):
        if self.currentState == States.IDLE :
            return True
        else:
            return False

    def isInERROR(self):
        if self.currentState == States.ERROR:
            return True
        else:
            return False

    def isInERROR(self):
        if self.currentState == States.ERROR:
            return True
        else:
            return False

    def isUnderCONTROL(self):
        if self.currentState == States.CONTROLLED:
            return True
        else:
            return False

    def isDRIVING(self):
        if self.currentState == States.DRIVING:
            return True
        else:
            return False
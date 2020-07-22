class routeStepClass:

    stepposx = 0
    stepposy = 0
    steptargetphi = 0
    stepmaxspeed = 0
    stepmaxomega = 0
    steptransitmode = ""
    stepcounters = 0
    steptextposx = 0
    steptextposy = 0
    circleposx = 0
    circleposy = 0
    steptype = ""

    def __init__(self,messagecontent):
        self.originalMessageContent = messagecontent
        self.parse()

    def parse(self):
        self.stepposx =  float(self.originalMessageContent[0]["px"])
        self.stepposy = float(self.originalMessageContent[1]["py"])
        self.steptargetphi = float(self.originalMessageContent[2]["tphi"])
        self.stepmaxspeed = float(self.originalMessageContent[3]["msp"])
        self.stepmaxomega = float(self.originalMessageContent[4]["mo"])
        self.steptransitmode = self.originalMessageContent[5]["trm"]
        self.stepcounters = int(self.originalMessageContent[6]["stc"])
        self.steptextposx = int(self.originalMessageContent[7]["tpx"])
        self.steptextposy = int(self.originalMessageContent[8]["tpy"])
        self.circleposx = int(self.originalMessageContent[9]["cpx"])
        self.circleposy = int(self.originalMessageContent[10]["cpy"])
        self.steptype = self.originalMessageContent[11]["t"]
        print(self.steptransitmode)

    def getPosX(self):
        return self.stepposx

    def getPosY(self):
        return self.stepposy

    def targetPhi(self):
        return self.stepposx

    def maxSpeed(self):
        return self.stepmaxspeed

    def maxOmega(self):
        return self.stepmaxomega

    def getTransitMode(self):
        return self.steptransitmode

    def getCounter(self):
        return self.stepcounters

    def getTextPosX(self):
        return self.steptextposx

    def getTextPosY(self):
        return self.steptextposy

    def getCirclePosX(self):
        return self.circleposx

    def getCirclePosY(self):
        return self.circleposy

    def getType(self):
        return self.steptype

    ### [
    #       {'px': '324.0'},
    #       {'py': '161.0'},
    #       {'tphi': '278.13010235415595'},
    #       {'msp': '2.0'},
    #       {'mo': '2.0'},
    #       {'trm': 'straightAndRotate'},
    #       {'stc': '1'},
    #       {'tpx': '344'},
    #       {'tpy': '181'},
    #       {'cpx': '309'},
    #       {'cpy': '146'},
    #       {'t': 'movementStep'}

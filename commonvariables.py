from state import sysState
from state import States
from state import ERRORS

systemStateMachine = sysState(States.IDLE)
steplist = list()
routeLoaded = False
coord_list = list()
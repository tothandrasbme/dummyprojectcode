from state import sysState
from state import States
from state import ERRORS

systemStateMachine = sysState(States.IDLE)
steplist = list()
routeLoaded = False
coord_list = list()
deltaTime = 0.5                 # 1rad=180° megtételéhez szükséges idő
offsetTime = 0.1                # platform megmozdulásához szükséges idő
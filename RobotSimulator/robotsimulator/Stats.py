LOCALISATION_FAULT_FILE = 'localisationFault.txt'
BOX_POSITIONS_FILE = 'boxPositions.txt'
ROBOT_POSITIONS_FILE = 'robotPositions.txt'

f1 = None
f2 = None
f3 = None

def init():
    global f1, f2, f3
    if (not f1):
        f1 = open(LOCALISATION_FAULT_FILE, 'w')
        f2 = open(BOX_POSITIONS_FILE, 'w')
        f3 = open(ROBOT_POSITIONS_FILE, 'w')

def localisationFault(*args, **keywargs):
    init()
    print (*args, file=f1)
def boxPositions(*args, **keywargs):
    init()
    print (*args, file=f2)
def robotPositions(*args, **keywargs):
    init()
    print (*args, file=f3)
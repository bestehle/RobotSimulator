import matplotlib.pyplot as plt

from robotsimulator import Stats

f1 = open(Stats.LOCALISATION_FAULT_FILE)
f2 = open(Stats.BOX_POSITIONS_FILE)
f3 = open(Stats.ROBOT_POSITIONS_FILE)

def localisationFault():
    # read file
    faults = []
    for line in f1:
        line = line.split('\t')
        lst = [float(x) for x in line]
        faults.append(lst)
    
    # extract
    xFault = list(map(lambda l : l[0], faults))
    yFault = list(map(lambda l : l[1], faults))
    thetaFault = list(map(lambda l : l[2], faults))
    
    # number of items
    num = len(xFault) + 1
    time = list(range(1, num))
    
    # plot
    plt.plot(time, xFault, 'r')
    plt.plot(time, yFault, 'b')
    plt.plot(time, thetaFault, 'y')
    # xMin - xMax and yMin - yMax
    plt.axis([0, num, 0, 0.6])
    plt.show()
    
def robotPosition():
    positons = []
    for line in f3:
        line = line.split('\t')
        lst = [float(x) for x in line]
        positons.append(lst)
        
    # extract
    approX = list(map(lambda l : l[0], positons))
    approY = list(map(lambda l : l[1], positons))
    approTheta = list(map(lambda l : l[2], positons))
    trueX = list(map(lambda l : l[3], positons))
    trueY = list(map(lambda l : l[4], positons))
    trueTheta = list(map(lambda l : l[5], positons))
        
    # number of items
    num = len(approX) + 1
    time = list(range(1, num))
        
    plt.suptitle('Robot position', fontsize=20)
    # plot
    plt.subplot(311)
    plt.ylabel('x', fontsize=18)
    plt.plot(time, approX, 'r', label='Line 1')
    plt.plot(time, trueX, 'g')
    
    plt.subplot(312)
    plt.ylabel('y', fontsize=18)
    plt.plot(time, approY, 'r')
    plt.plot(time, trueY, 'g')
    
    plt.subplot(313)
    plt.ylabel('theta', fontsize=18)
    plt.plot(time, approTheta, 'r')
    plt.plot(time, trueTheta, 'g')


localisationFault()
robotPosition()
plt.show()





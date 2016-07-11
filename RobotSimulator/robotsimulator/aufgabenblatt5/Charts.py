import matplotlib.pyplot as plt

from robotsimulator import Stats, GeometryHelper

f1 = open(Stats.LOCALISATION_FAULT_FILE)
f2 = open(Stats.BOX_POSITIONS_FILE)
f3 = open(Stats.ROBOT_POSITIONS_FILE)
f4 = open(Stats.GLOBAL_LOCALISATION_WEIGHT_SUM)

def globalLocalisationWeightSum():
    weights = []
    varX = []
    varY = []
    weightsLimit = []
    varXLimit = []
    varYLimit = []
    for line in f4:
        line = line.split('\t')
        weights.append(int(line[0]))
        varX.append(float(line[1]))
        varY.append(float(line[2]))
        weightsLimit.append(int(line[3]))
        varXLimit.append(float(line[4]))
        varYLimit.append(float(line[5]))
    print (varX)
    # number of items
    num = len(weights)
    time = list(range(0, num))
    
    #weightsLimit = [10000000000] * num
    #varXLimit = [0.03] * num
    #varYLimit = [0.03] * num
    
    plt.suptitle('Robot position', fontsize=20)
    # plot
    plt.subplot(311)
    plt.ylabel('weight sum', fontsize=15)
    plt.plot(time, weights, 'r')
    plt.plot(time, weightsLimit, 'g')
    plt.axis([0, num, 0, weightsLimit[0] * 10])

    plt.subplot(312)
    plt.ylabel('Varianz x', fontsize=15)
    plt.plot(time, varX, 'r')
    plt.plot(time, varXLimit, 'g')
    plt.axis([0, num, 0, varXLimit[0] * 10])
    
    plt.subplot(313)
    plt.ylabel('Varianz y', fontsize=15)

    plt.plot(time, varY, 'r')
    plt.plot(time, varYLimit, 'g')
    plt.axis([0, num, 0, varYLimit[0] * 10])
    
    plt.show()

def boxPositions():
    positons = []
    f2 = open(Stats.BOX_POSITIONS_FILE)
    for line in f2:
        line = line.split('\t')
        lst = [float(x) for x in line]
        positons.append(lst)
        
    approX = list(map(lambda l : l[0], positons))
    approY = list(map(lambda l : l[1], positons))
    trueX = list(map(lambda l : l[2], positons))
    trueY = list(map(lambda l : l[3], positons))
    
    # number of items
    num = len(approX) + 1
    time = list(range(1, num))
    
    plt.suptitle('Box positions', fontsize=20)
    # plot
    plt.subplot(211)
    plt.ylabel('x', fontsize=18)
    plt.plot(time, approX, 'r', label='Approximate Position')
    plt.plot(time, trueX, 'g', label='True Position')
    plt.legend(numpoints=1)
    plt.subplot(212)
    plt.ylabel('y', fontsize=18)
    plt.plot(time, approY, 'r')
    plt.plot(time, trueY, 'g')

    plt.show()
    

def boxPositionsInGrid():
    f2 = open(Stats.BOX_POSITIONS_FILE)
    positons = []
    for line in f2:
        line = line.split('\t')
        lst = [float(x) for x in line]
        positons.append(lst)
        
    approX = list(map(lambda l : l[0], positons))
    approY = list(map(lambda l : l[1], positons))
    trueX = list(map(lambda l : l[2], positons))
    trueY = list(map(lambda l : l[3], positons))
    
    plt.suptitle('Box positions', fontsize=20)
    # plot
    plt.ylabel('y', fontsize=18)
    plt.xlabel('x', fontsize=18)
    plt.plot(approX, approY, 'g+', label='Approximate Position')
    plt.plot(trueX, trueY, 'ro', label='True Position')
    plt.axis([0, 19, 0, 14])
    plt.legend(numpoints=1)
    plt.show()

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
    
    plt.suptitle('Robot localisation fault', fontsize=20)
    # plot
    plt.plot(time, xFault, 'r', label='x fault')
    plt.plot(time, yFault, 'b', label='y fault')
    plt.plot(time, thetaFault, 'y', label='theta fault')
    plt.legend(numpoints=3)
    # xMin - xMax and yMin - yMax
    plt.axis([0, num, 0, 1.2])
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
    approTheta = list(map(lambda l : GeometryHelper.diffDegree(l[2], 0), positons))
    trueX = list(map(lambda l : l[3], positons))
    trueY = list(map(lambda l : l[4], positons))
    trueTheta = list(map(lambda l : GeometryHelper.diffDegree(l[5], 0), positons))
        
    # number of items
    num = len(approX) + 1
    time = list(range(1, num))
        
    plt.suptitle('Robot position', fontsize=20)
    # plot
    plt.subplot(311)
    plt.ylabel('x', fontsize=18)
    plt.plot(time, approX, 'r', label='Approximate Position')
    plt.plot(time, trueX, 'g', label='True Position')
    plt.legend(numpoints=3)
    plt.subplot(312)
    plt.ylabel('y', fontsize=18)
    plt.plot(time, approY, 'r', label='Approximate Position')
    plt.plot(time, trueY, 'g', label='True Position')
    
    plt.subplot(313)
    plt.ylabel('theta', fontsize=18)
    plt.plot(time, approTheta, 'r', label='Approximate Position')
    plt.plot(time, trueTheta, 'g', label='True Position')
    

    plt.show()

boxPositionsInGrid()
boxPositions()
#globalLocalisationWeightSum()
localisationFault()
robotPosition()





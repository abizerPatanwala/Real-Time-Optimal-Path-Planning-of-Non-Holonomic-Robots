import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Circle
import time
import copy
from scipy.optimize import minimize

class diffDrive:
    def __init__(self, radius, L,  omegaRMax, omegaLMax, currState,stepTime):
       self.radius = radius #radius of the wheels
       self.L = L #distance between two wheels along the wheel axle
       self.omegaRMax = omegaRMax
       self.omegaLMax = omegaLMax
       self.velMax = (self.radius*(omegaRMax + omegaLMax)) / 2.0
       self.omegaMax = (self.radius*(omegaRMax)/self.L)
       self.currState = currState #State of the robot, expressed as (x,y,theta) vector
       self.simStepTime = stepTime
       self.Qe = np.array([[100, 0, 0],[0, 100, 0],[0, 0, 1]])
       self.Qi = np.array([[1, 0],[0, 1]])
       self.Qu = np.array([[0.01, 0],[0, 0.01]])
       #self.C = 1
       #self.pho = 1
       #self.epsilon = 0.0000001
    
    def forwardKin(self, omega_l, omega_r):
     v = (self.radius*(omega_r + omega_l)) / 2.0
     omega = (self.radius*(omega_r - omega_l)) / self.L
     self.currState[0] = self.currState[0] + v*math.cos(self.currState[2])*self.simStepTime
     self.currState[1] = self.currState[1] + v*math.sin(self.currState[2])*self.simStepTime
     self.currState[2] = self.currState[2] + omega*self.simStepTime
     if self.currState[2] > 2*math.pi:
         self.currState[2] = self.currState[2] - 2*math.pi
     if self.currState[2] < 0:
         self.currState[2] = 2*math.pi + self.currState[2] 


def obsAvoid(u,pathDes, robot, obstList, sensorRad, obsAvoidThresh, controlHorizon, prevU) :
    controls = np.reshape(u, (int(len(u) / 2), 2))
    if len(pathDes) > controlHorizon + 1:
        temp = np.full((len(pathDes) - 1 - controlHorizon ,2), [controls[-1][0], controls[-1][1]])
        controls = np.vstack((controls,temp))
    temprobot = copy.deepcopy(robot)
    conList = []
    
    '''
    conList.append(2 - abs(prevU[0] - controls[0][0]))
    conList.append(2 - abs(prevU[1] - controls[0][1]))
    '''
    '''
    if prevU[0] - prevU[1] >= 1:
        conList.append(controls[0][0] - controls[0][1])
    if prevU[1] - prevU[0] >= 1:
        conList.append(controls[0][1] - controls[0][0])
    '''
    
    for i in range(len(pathDes)):
        corners = [[0,0],[0,0]]
        currState = copy.deepcopy(temprobot.currState)
        sensorCenter = [currState[0] + (robot.L/2 + 2*sensorRad)*math.cos(currState[2]), currState[1] + (robot.L/2 + 2*sensorRad)*math.sin(currState[2])] 
        '''
        back = [2*currState[0] - sensorCenter[0],2*currState[1] - sensorCenter[1]]  
        corners[0][0] = sensorCenter[0] - (robot.L/2)*math.sin(currState[2])
        corners[0][1] = sensorCenter[1] + (robot.L/2)*math.cos(currState[2])
        corners[1][0] = sensorCenter[0] + (robot.L/2)*math.sin(currState[2])
        corners[1][1] = sensorCenter[1] - (robot.L/2)*math.cos(currState[2])
        corners[2][0] = back[0] - (robot.L/2)*math.sin(currState[2])
        corners[2][1] = back[1] + (robot.L/2)*math.cos(currState[2])
        corners[3][0] = back[0] + (robot.L/2)*math.sin(currState[2])
        corners[3][1] = back[1] - (robot.L/2)*math.cos(currState[2])
        minDist = math.inf
        minObst1 = []
        for k in range(len(obstList)):
            dist = math.sqrt((corners[0][0] - obstList[k][0])**2 + (corners[0][1] - obstList[k][1])**2) 
            if  dist < minDist:
                minObst1 = [obstList[k][0], obstList[k][1]]
                minDist = dist 
        minDist = math.inf
        minObst2 = []
        for k in range(len(obstList)):
            dist = math.sqrt((corners[0][0] - obstList[k][0])**2 + (corners[0][1] - obstList[k][1])**2) 
            if dist < minDist:
                minObst2 = [obstList[k][0], obstList[k][1]]   
                minDist = dist       
        if len(obstList) > 0:
            conList.append(math.sqrt((corners[0][0] - minObst1[0])**2 + (corners[0][1] - minObst1[1])**2) - 0.1)
            conList.append(math.sqrt((corners[1][0] - minObst2[0])**2 + (corners[1][1] - minObst2[1])**2) - 0.1)
        
        for j in range(len(obstList)):
            dist = math.sqrt((sensorCenter[0] - obstList[j][0])**2 + (sensorCenter[1] - obstList[j][1])**2)
            cost = cost + math.pow(robot.C / (dist + robot.epsilon), robot.pho)
            
            for k in range(len(corners)):
                dist = math.sqrt((corners[k][0] - obstList[j][0])**2 + (corners[k][1] - obstList[j][1])**2)
                cost = cost + math.pow(robot.C / (dist + robot.epsilon), robot.pho)
            
        '''
        for j in range(0, len(obstList), 2):
            conList.append(math.sqrt((sensorCenter[0] - obstList[j][0])**2 + (sensorCenter[1] - obstList[j][1])**2) - obsAvoidThresh)
        if i != (len(pathDes) - 1):
            temprobot.forwardKin(controls[i][0], controls[i][1])    
    return conList

def cost(u, pathDes, robot, controlHorizon, prevU):
    controls = np.reshape(u, (int(len(u) / 2), 2))
    if len(pathDes) > controlHorizon + 1:
        temp = np.full((len(pathDes) - 1 - controlHorizon ,2), [controls[-1][0], controls[-1][1]])
        controls = np.vstack((controls,temp))
    cost = 0
    temprobot = copy.deepcopy(robot)
    for i in range(len(pathDes)):
        corners = [[0,0],[0,0]]
        currState = copy.deepcopy(temprobot.currState)
        desState =  np.array([[pathDes[i][0], pathDes[i][1], pathDes[i][2]]])
        errorTraj = np.reshape(np.array(currState - desState), (1,3))
        cost = cost + np.matmul(np.matmul(errorTraj, robot.Qe), np.transpose(errorTraj))
        #endState =  np.array([[pathDes[-1][0], pathDes[-1][1], pathDes[-1][2]]])
        #endState =  np.array(endState)
        #errorLast = np.reshape(np.array(currState - endState), (1,3))
        #cost = cost + np.matmul(np.matmul(errorLast, robot.Qt), np.transpose(errorLast))
        if i != (len(pathDes) - 1):
            temprobot.forwardKin(controls[i][0], controls[i][1])
            if i == 0:
                errorControl = np.reshape(np.array(controls[i] - prevU), (1,2))
            else:
                errorControl = np.reshape(np.array(controls[i] - controls[i-1]), (1,2))
            cost = cost +  np.matmul(np.matmul(errorControl, robot.Qu), np.transpose(errorControl))
    return cost

def plot_grid(ax, obs_boundary, obs_rectangle, obs_circle):
        for (ox, oy, w, h) in obs_boundary:
            ax.add_patch(
                Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in obs_rectangle:
            ax.add_patch(
                Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in obs_circle:
            ax.add_patch(
                Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )
def main():
    obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
    obs_rectangle = [
            [2, 27, 8, 3],
            [12, 27, 8, 3],
            [22, 27, 8, 3],
            [32, 27, 8, 3],
            [42, 27, 6, 3],
            [40, 1, 6, 8],
            [33, 1, 6, 8],
            [26, 1, 6, 8],
            [42, 12, 6, 8],
            [35, 16, 6, 4],
            [4, 12, 6, 10],
            [12, 15, 2, 2],
            [12, 18, 2, 2],
            [15, 15, 2, 2],
            [15, 18, 2, 2],
            [2, 1, 20, 8],
            [14, 23, 10, 2]
        ]
    obs_circle = [
            [46, 23, 2],
            [38, 13, 2]
        ]
    ##Initialize parameters and robot           
    L = 0.16   # axle length of the  car
    radius = 0.033       # radius of the wheel 
    omegaRMax = 6.67
    omegaLMax = 6.67
    obsAvoidThresh = 0.16
    rayLength = 1.5
    #L = 10  
    #radius = 10/4
    #omegaRMax = 3
    #omegaLMax = 3
    Horizon = 20  # in (no of pointts)
    controlHorizon = max(int(math.ceil((10/100)*Horizon)) , 5)
    stepTime = 0.225   # in sec, used in optimization, used for descretizing the path, and used in robot simulation

    
    step = (rayLength / Horizon)
    #print(step)

    path_x = []
    path_y = []

    path_x_rev = [11, 11.128495942087765, 12.719477090044013, 13.033606514801276, 15.455575196741822, 17.838618283542385, 19.62132693870136, 21.390418224855743, 22.370586576560186, 23.488744766410782, 24]
    for i in range(len(path_x_rev)):
        x = path_x_rev[len(path_x_rev)-1-i]
        path_x.append(x)
    #print(path_x)

    path_y_rev = [28, 26.29397106351433, 23.616827283982122, 22.463548210728447, 21.596758256390768, 20.33002091685223, 17.678797069365306, 14.146242815220994, 10.42891238014871, 7.738448831887791, 6]
    for i in range(len(path_y_rev)):
        y = path_y_rev[len(path_y_rev)-1-i]
        path_y.append(y)
    #print(path_y)

    len_path = len(path_x_rev)
    pathDes = []
    # add for loop here for len_path
    for i in range(len_path-1):
        start = [path_x[i], path_y[i]]
        end = [path_x[i+1], path_y[i+1]]
        theta_curr = math.atan2(end[1] - start[1], end[0] - start[0] )
        dist_curr = math.sqrt((start[1] - end[1])**2 + (start[0] - end[0])**2)
        numPoints = int(dist_curr / (0.22* stepTime) + 1)
        #print(numPoints)
        pathxDes = np.reshape(np.linspace(start[0], end[0], numPoints), (numPoints,1))
        pathyDes = np.reshape(np.linspace(start[1], end[1], numPoints), (numPoints,1))
        theta = np.reshape(np.array([theta_curr] * numPoints), (numPoints,1))  
        path = np.concatenate((pathxDes,pathyDes, theta), axis = 1)

        for j in range(len(path)):
            pathDes.append([path[j][0], path[j][1], path[j][2]])
            
    #pathDes.append([path_x[len(path_x)-1], path_y[len(path_y)-1],0])
    pathDes = np.array(pathDes)
    print("pathDes: ", pathDes)
    initState = [pathDes[0][0], pathDes[0][1] , pathDes[0][2]]
    robot = diffDrive(radius, L, omegaRMax, omegaLMax, initState, stepTime)

    '''
    start = [0.00755, 0.7]
    end = [8,5, 0.7]
    
    #numPoints = int(((finalTime - initialTime) / stepTime) + 1)
    numPoints = int((abs(end[1] - start[1]) / (0.22* stepTime)) + 1)
    print(numPoints)
    pathxDes = np.reshape(np.linspace(start[0], end[0], numPoints), (numPoints,1))
    pathyDes = np.reshape(np.array([start[1]] * numPoints), (numPoints,1))
    theta = np.reshape(np.array([0] * numPoints), (numPoints,1))  
    pathDes = np.concatenate((pathxDes,pathyDes, theta), axis = 1)
    '''

    # Run MPC
    count = 0
    pathAct = []
    pathAct.append([robot.currState[0], robot.currState[1], robot.currState[2]])
    uCurr = [0, 0]
    numRays = 50
    sensorRad = 0.02
    
    angleDiff = math.pi / (numRays - 1)
    obsx = [20.5, 21]
    obsy = [12, 12.5]
    RectW = obsx[1] - obsx[0]
    RectH = obsy[1] - obsy[0]
    obsx2 = [13.033606514801276, 13.033606514801276 + 0.5]
    obsy2 = [22.463548210728447, 22.463548210728447 + 0.5]
    RectW2 = obsx2[1] - obsx2[0]
    RectH2 = obsy2[1] - obsy2[0]
    iter = 1
    iter_list = []
    opt_time = []
    path_deviation = []
    dynm_thresh = 2.85
    velObs = 0.07
    while(True):
        startTime = time.time()
        #diff_array = np.absolute(timeStamp-(currTime + timeHorizon))
        #index = diff_array.argmin()
        obstList = []
        #print("path no", iter)
        iter = iter + 1
        path_deviation.append(math.sqrt((robot.currState[0] - pathDes[count][0])**2 + (robot.currState[1] - pathDes[count][1])**2))
        
        
        if math.sqrt((robot.currState[0] - obsx[0])**2 + (robot.currState[1] - obsy[0])**2) <= dynm_thresh:
            obsx[0] = obsx[0] + velObs*stepTime
            obsy[0] = obsy[0] + velObs*stepTime
            obsx[1] = obsx[1] + velObs*stepTime
            obsy[1] = obsy[1] + velObs*stepTime
        
        sensorCenter = [robot.currState[0] + (L/2 + 2*sensorRad)*math.cos(robot.currState[2]), robot.currState[1] + (L/2 + 2*sensorRad)*math.sin(robot.currState[2])]
        for j in range(numRays):
            currTheta  = robot.currState[2] - math.pi/2 + j*angleDiff
            startx = sensorCenter[0]
            endx = sensorCenter[0] + rayLength*math.cos(currTheta)
            starty = sensorCenter[1]
            endy = sensorCenter[1] + rayLength*math.sin(currTheta)
            rayPointx = np.linspace(startx,endx,50)
            rayPointy = np.linspace(starty,endy,50)
            for k in range(len(rayPointx)):
                if rayPointx[k] > obsx[0] and rayPointx[k] < obsx[1]  and rayPointy[k] > obsy[0] and rayPointy[k] < obsy[1]:
                    obstList.append([rayPointx[k], rayPointy[k]])
                    break
                elif rayPointx[k] > obsx2[0] and rayPointx[k] < obsx2[1]  and rayPointy[k] > obsy2[0] and rayPointy[k] < obsy2[1]:
                    obstList.append([rayPointx[k], rayPointy[k]])
                    break

        

        
        
        distToCurr = (pathDes[count][0] - robot.currState[0])**2 + (pathDes[count][1] - robot.currState[1])**2 + (pathDes[count][2] - robot.currState[2])**2
        distToNext = (pathDes[count + 1][0] - robot.currState[0])**2 + (pathDes[count + 1][1] - robot.currState[1])**2 + (pathDes[count + 1][2] - robot.currState[2])**2
        if distToCurr > distToNext:
           count = count + 1
        if count == len(pathDes) - 1:
           break
        if count <= len(pathDes) - 1 - Horizon:
            pathHorizon = pathDes[count: count + Horizon,:]
        else:
           pathHorizon = pathDes[count:,:]

        if len(pathHorizon) >= controlHorizon + 1:
            u0 = np.array([0]* (2*controlHorizon - 2))
            u0 = np.concatenate((uCurr, u0))
            bndsize = controlHorizon
        else:
            usize = int((len(pathHorizon - 1) * 2)) 
            u0 = np.array([0]* (usize - 2))
            u0 = np.concatenate((uCurr, u0))
            bndsize = int(usize / 2)
        
        
        
        bnds = [(0,robot.omegaLMax), (0,robot.omegaRMax)] * bndsize

        cons = ({'type': 'ineq', 'fun': obsAvoid, 'args': (pathHorizon, robot, obstList, sensorRad, obsAvoidThresh, controlHorizon,uCurr)})
       
        #result= minimize(cost, u0, args=(pathHorizon, robot, obstList, sensorRad, endState), method='SLSQP', bounds=bnds, options = {'maxiter' : 20}, constraints= cons)
        result= minimize(cost, u0, args=(pathHorizon, robot, controlHorizon, uCurr), method='SLSQP', bounds=bnds, constraints = cons)

        #result= minimize(cost, u0, args=(pathHorizon, robot, controlHorizon, uCurr), method='SLSQP', bounds=bnds)
        #print("cost is: ", result.fun)
        uCurr = [result.x[0], result.x[1]]
        robot.forwardKin(uCurr[0], uCurr[1])
        pathAct.append([robot.currState[0], robot.currState[1], robot.currState[2]])
        endTime = time.time()
        opt_time.append(endTime - startTime)
        iter_list.append(iter)
        #print("control inputs are: ", uCurr[0], uCurr[1])
        print("current position of the robot ", robot.currState )
        #print("time taken", endTime - startTime)
            
    #Initialize figure
    plt.plot(iter_list, opt_time)
    plt.title("optimization time vs No of control steps ")
    plt.xlabel("No of Control steps")
    plt.ylabel("Optimization time")
    plt.show()
    
    plt.plot(iter_list, path_deviation[0:len(path_deviation) - 1])
    plt.title("Pah deviation vs No of control steps")
    plt.xlabel("No of Control steps")
    plt.ylabel("Path deviation")
    plt.show()
    plt.show()
    plt.ion()
    fig ,ax = plt.subplots(figsize = [70,70])
    ax.set_xlim(0,50)
    ax.set_ylim(0,35)
    
    #Initialize Robot
    robotFig = Rectangle((start[0], start[1]), L, L, rotation_point='center')
    ax.add_patch(robotFig)
    
    #Initialize sensors
    sensor = Circle((start[0], start[1]), radius = sensorRad)
    ax.add_patch(sensor)

    #Initialize sensor rays
    #sensorLines  = []
    #for i in range(numRays):
    #   temp, = ax.plot(1, 1, color = (0.6350, 0.0780, 0.1840), linestyle = '--')
    #   sensorLines.append(temp)
   
    #Initialize desired path and actual path
    lineDes, = ax.plot(1, 1, color = (0.4660, 0.6740, 0.1880))
    lineAct, = ax.plot(1, 1, color = 'b')
    
    #Initialize obstacles
    obsx = [20.5, 21]
    obsy = [12, 12.5]
    RectW = obsx[1] - obsx[0]
    RectH = obsy[1] - obsy[0]
    obsx2 = [13.033606514801276, 13.033606514801276 + 0.5]
    obsy2 = [22.463548210728447, 22.463548210728447 + 0.5]
    RectW2 = obsx2[1] - obsx2[0]
    RectH2 = obsy2[1] - obsy2[0]
    obs = Rectangle((obsx[0], obsy[0]), RectW, RectH)
    obs2 = Rectangle((obsx2[0], obsy2[0]), RectW2, RectH2)
    ax.add_patch(obs)
    ax.add_patch(obs2)
    plot_grid(ax, obs_boundary, obs_rectangle, obs_circle)
    
    pathAct = np.array(pathAct)
    for i in range(len(pathAct)):
        #start = time.time()
        #obsx[0] = obsx[0] + velObs*stepTime
        #obsy[0] = obsy[0] + velObs*stepTime
        #obs = Rectangle((obsx[0], obsy[0]), RectW, RectH)
        #ax.add_patch(obs)
        #ax.add_patch(obs2)
        if math.sqrt((pathAct[i][0] - obsx[0])**2 + (pathAct[i][1] - obsy[0])**2) <= dynm_thresh:
            obsx[0] = obsx[0] + velObs*stepTime
            obsy[0] = obsy[0] + velObs*stepTime
        obs.set_xy((obsx[0], obsy[0]))
        lineDes.set_xdata(pathDes[:, 0])
        lineDes.set_ydata(pathDes[:, 1])
        lineAct.set_xdata(pathAct[0: i+1, 0])
        lineAct.set_ydata(pathAct[0: i+1, 1])
        robotFig.set_xy((pathAct[i][0] - L/2, pathAct[i][1] - L/2))
        robotFig.set_angle((pathAct[i][2] / math.pi) * 180)
        sensorCenter = [pathAct[i][0] + (L/2 + sensorRad)*math.cos(pathAct[i][2]), pathAct[i][1] + (L/2 + sensorRad)*math.sin(pathAct[i][2])]
        sensor.set(center = (sensorCenter[0], sensorCenter[1]))
        #for j in range(numRays):
        #    currTheta  = pathAct[i][2] - math.pi/2 + j*angleDiff
        #    sensorLines[j].set_xdata([sensorCenter[0], sensorCenter[0] + rayLength*math.cos(currTheta)])
        #    sensorLines[j].set_ydata([sensorCenter[1], sensorCenter[1] + rayLength*math.sin(currTheta)])
        fig.canvas.draw()
        fig.canvas.flush_events()
        #end = time.time()
        #print("time taken : ", end - start)
        #time.sleep()
       
    
if __name__ == '__main__':
    main()

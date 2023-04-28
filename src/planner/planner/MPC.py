import rclpy
from rclpy.node import Node
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
import numpy as np
import copy
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('MPC')
        self.subscription = self.create_subscription(LaserScan,'scan',self.listener_callback1, 1)
        self.subscription = self.create_subscription(Odometry,'odom',self.listener_callback, 1)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription  # prevent unused variable warning
        self.t0 = 0
        self.init = True
        self.L = 0.16   # axle length of the  car
        self.sensDist = 0.07
        self.radius = 0.033       # radius of the wheel 
        self.omegaRMax = 6.67
        self.omegaLMax = 6.67
        self.velMax = (self.radius*(self.omegaRMax + self.omegaLMax)) / 2.0
        self.obsAvoidThresh = 0.18
        self.rayLength = 1.5
        self.numRays = 180
        self.angRes = 0.0174
        self.Horizon = 20 # in (no of pointts)
        self.controlHorizon = max(int(math.ceil((10/100)*self.Horizon)) , 5)
        self.simStepTime = 0.225  # in sec, used in optimization, used for descretizing the path, and used in robot simulation
        self.Qe = np.array([[100, 0, 0],[0, 100, 0],[0, 0, 1]])
        self.Qu = np.array([[3, 0],[0, 3]])
        self.Qs = 10
        self.goalThresh = 0.05
        self.count = 0
        self.uCurr = [0, 0]
        self.data = []
        self.obstList = []
        self.start = [-0.7, 0.00755]
        self.end = [-0.7, -8.5]
        self.currState =[0, 0, 0]  #State of the robot, expressed as (x,y,theta) vector
        self.epsilon = 0.0001
        self.C = 1
        self.pho = 1.0
        path_x_rev = [3.89, 2.85, 1.97, 1.18, 0.47, -0.27, -0.82, -2.08, -2.9, -4.14]
        path_x = []
        path_y = []
        for i in range(len(path_x_rev)):
            x = path_x_rev[len(path_x_rev)-1-i]
            path_x.append(x)
    
        path_y_rev = [1.77, 1.82, 1.72, 1.48, 0.6, -0.35, -1.39, -1.53, -2.42, -3.15]
        for i in range(len(path_y_rev)):
            y = path_y_rev[len(path_y_rev)-1-i]
            path_y.append(y)
    
        len_path = len(path_x_rev)
        self.pathDes = []
        for i in range(len_path-1):
            start = [path_x[i], path_y[i]]
            end = [path_x[i+1], path_y[i+1]]
            theta_curr = math.atan2(end[1] - start[1], end[0] - start[0] )
            dist_curr = math.sqrt((start[1] - end[1])**2 + (start[0] - end[0])**2)
            numPoints = int(dist_curr / (0.22* self.simStepTime) + 1)
            pathxDes = np.reshape(np.linspace(start[0], end[0], numPoints), (numPoints,1))
            pathyDes = np.reshape(np.linspace(start[1], end[1], numPoints), (numPoints,1))
            theta = np.reshape(np.array([theta_curr] * numPoints), (numPoints,1))  
            path = np.concatenate((pathxDes,pathyDes, theta), axis = 1)
            for j in range(len(path)):
                self.pathDes.append([path[j][0], path[j][1], path[j][2]])
            
        self.pathDes = np.array(self.pathDes)
    
        #numPoints = int((abs(self.end[1] - self.start[1]) / (self.velMax* self.simStepTime)) + 1)
        #pathxDes = np.reshape(np.array([self.start[0]] * numPoints), (numPoints,1))
        #pathyDes = np.reshape(np.linspace(self.start[1], self.end[1], numPoints), (numPoints,1))
        #theta = np.reshape(np.array([-math.pi/2] * numPoints), (numPoints,1))  
        #elf.pathDes = np.concatenate((pathxDes,pathyDes, theta), axis = 1)
        
    def listener_callback1(self, msg):
        self.data = msg.ranges
        self.init = False
    
    def forwardKin(self, omega_l, omega_r, currPos):
        v = (self.radius*(omega_r + omega_l)) / 2.0
        omega = (self.radius*(omega_r - omega_l)) / self.L
        currPos[0] = currPos[0] + v*math.cos(currPos[2])*self.simStepTime
        currPos[1] = currPos[1] + v*math.sin(currPos[2])*self.simStepTime
        currPos[2] = currPos[2] + omega*self.simStepTime        
        return currPos
    
    
    def create_obstacle(self):
        sensorCenter = [self.currState[0] + (self.sensDist)*math.cos(self.currState[2]),  self.currState[1] + (self.sensDist)*math.sin(self.currState[2])]
        min = math.inf
        flag = False
        index = []
        
        '''
        for j in range(0, self.numRays, 35):
            currTheta  = self.currState[2] - math.pi/2 + j*self.angRes
            currDist = self.data[j]
            if self.data[j] <= self.rayLength:
                x = sensorCenter[0] + currDist*math.cos(currTheta)
                y = sensorCenter[1] + currDist*math.sin(currTheta)
                self.obstList.append([x, y])
                flag = True
        '''
        count = 0
        for j in range(self.numRays):
            if self.data[j] <= self.rayLength:
                count = count + 1
                index.append(j)
                #if count == 3:
                break
        count = 0
        for j in range(self.numRays):
            if self.data[self.numRays - j - 1] <= self.rayLength:
                count = count + 1
                index.append(self.numRays - j - 1)
                #if count == 3:
                break
        
        if len(index) == 2:
            mid = int((index[0] + index[1]) / 2)
            if self.data[mid] <= self.rayLength:
                index.append(mid)
    
        for j in range(len(index)):
            currTheta  = self.currState[2] - math.pi/2 + index[j]*self.angRes
            currDist = self.data[index[j]]
            x = sensorCenter[0] + currDist*math.cos(currTheta)
            y = sensorCenter[1] + currDist*math.sin(currTheta)
            if (x - sensorCenter[0]) >= 0:
                self.obstList.append([x, y])
                flag = True
        
        #if flag == True:
            #print("obstacles are: ", self.obstList)
    
    def obsAvoid(self, u, pathHorizon, prevU) :
        controls = np.reshape(u, (int(len(u) / 2), 2))
        if len(pathHorizon) > self.controlHorizon + 1:
            temp = np.full((len(pathHorizon) - 1 - self.controlHorizon ,2), [controls[-1][0], controls[-1][1]])
            controls = np.vstack((controls,temp))
        conList = []
        #if prevU[0] - prevU[1] >= 1:
        #    conList.append(controls[0][0] - controls[0][1])
        #if prevU[1] - prevU[0] >= 1:
        #    conList.append(controls[0][1] - controls[0][0])
        #conList.append(1 - abs(controls[0][0] -  prevU[0]))
        #conList.append(1 - abs(controls[0][1] -  prevU[1]))
       
        state = copy.deepcopy(self.currState)
        for i in range(len(pathHorizon)):
            sensorCenter = [state[0] + (self.sensDist)*math.cos(state[2]), state[1] + (self.sensDist)*math.sin(state[2])] 
            for j in range(len(self.obstList)):
                conList.append(math.sqrt((sensorCenter[0] - self.obstList[j][0])**2 + (sensorCenter[1] - self.obstList[j][1])**2) - self.obsAvoidThresh)
            if i != (len(pathHorizon) - 1):
                state = self.forwardKin(controls[i][0], controls[i][1], state)    
        return conList
    
    
    def cost(self, u, pathHorizon, prevU):
        controls = np.reshape(u, (int(len(u) / 2), 2))
        if len(pathHorizon) > self.controlHorizon + 1:
            temp = np.full((len(pathHorizon) - 1 - self.controlHorizon ,2), [controls[-1][0], controls[-1][1]])
            controls = np.vstack((controls,temp))
        cost = 0
        state = copy.deepcopy(self.currState)
        for i in range(len(pathHorizon)):
            sensorCenter = [state[0] + (self.sensDist)*math.cos(state[2]), state[1] + (self.sensDist)*math.sin(state[2])] 
            desState =  np.array([[pathHorizon[i][0], pathHorizon[i][1], pathHorizon[i][2]]])
            errorTraj = np.reshape(np.array(state - desState), (1,3))
            cost = cost + np.matmul(np.matmul(errorTraj, self.Qe), np.transpose(errorTraj))
            '''
            max = 0
            for j in range(len(self.obstList)):
                dist = math.sqrt((sensorCenter[0] - self.obstList[j][0])**2 + (sensorCenter[1] - self.obstList[j][1])**2)
                if dist > max:
                    max = dist
            
            min = math.inf
            for j in range(len(self.obstList)):
                dist = math.sqrt((sensorCenter[0] - self.obstList[j][0])**2 + (sensorCenter[1] - self.obstList[j][1])**2)
                if dist < min:
                    min = dist
            '''
            for j in range(len(self.obstList)):
                dist = math.sqrt((sensorCenter[0] - self.obstList[j][0])**2 + (sensorCenter[1] - self.obstList[j][1])**2)
                #C_curr = (self.C / (min- max))*dist - (self.c*max / (min - max))
                cost = cost + math.pow(self.C/ (dist + self.epsilon), self.pho)
            
            if i != (len(pathHorizon) - 1):
                state = self.forwardKin(controls[i][0], controls[i][1], state)
                if i == 0:
                    errorControl = np.reshape(np.array(controls[i] - prevU), (1,2))
                else:
                    errorControl = np.reshape(np.array(controls[i] - controls[i-1]), (1,2))
                cost = cost +  np.matmul(np.matmul(errorControl, self.Qu), np.transpose(errorControl))
        return cost

    def listener_callback(self, msg):
        #if self.init == True:
        #    self.t0 = msg.header.stamp._sec + msg.header.stamp.nanosec*(10**-9)
        #    self.init = False

        #t = msg.header.stamp._sec + msg.header.stamp.nanosec*(10**-9)
        #if t - self.t0 >=self.simStepTime:
        if self.init == False:
        
            #print("callback")
            startTime = time.time()
            xpos = msg.pose.pose.position.x
            ypos = msg.pose.pose.position.y
            quatx = msg.pose.pose.orientation.x
            quaty = msg.pose.pose.orientation.y
            quatz = msg.pose.pose.orientation.z
            quatw = msg.pose.pose.orientation.w
            r = R.from_quat([quatx, quaty, quatz, quatw])
            theta = r.as_rotvec()[2]
            self.currState = [xpos, ypos, theta]
            print(self.currState)
            
            distToCurr = (self.pathDes[self.count][0] - self.currState[0])**2 + (self.pathDes[self.count][1] - self.currState[1])**2 + (self.pathDes[self.count][2] - self.currState[2])**2
            distToNext = (self.pathDes[self.count + 1][0] - self.currState[0])**2 + (self.pathDes[self.count + 1][1] - self.currState[1])**2 + (self.pathDes[self.count + 1][2] - self.currState[2])**2
            
            if distToCurr > distToNext:
                self.count = self.count + 1
            if self.count <= len(self.pathDes) - 1 - self.Horizon:
                pathHorizon = self.pathDes[self.count: self.count + self.Horizon,:]
            else:
                pathHorizon = self.pathDes[self.count:,:]
            
            if len(pathHorizon) >= self.controlHorizon + 1:
                u0 = np.array([0]* (2*self.controlHorizon - 2))
                u0 = np.concatenate((self.uCurr, u0))
                bndsize = self.controlHorizon
            else:
                usize = int((len(pathHorizon - 1) * 2)) 
                u0 = np.array([0]* (usize - 2))
                u0 = np.concatenate((self.uCurr, u0))
                bndsize = int(usize / 2)
            
            self.create_obstacle()
            bnds = [(0,self.omegaLMax), (0,self.omegaRMax)] * bndsize
            #if len(self.obstList) > 0:
            #    bnds = [(0,self.omegaLMax), (0,self.omegaRMax)] * bndsize

            cons = ({'type': 'ineq', 'fun': self.obsAvoid, 'args': (pathHorizon, self.uCurr)})
            
            result = minimize(self.cost, u0, args=(pathHorizon, self.uCurr), method='SLSQP', bounds=bnds, constraints = cons) 
            self.uCurr = [result.x[0], result.x[1]]
           #print("angular velocities: ", result.x[0], result.x[1])
            #print("current state: ", self.currState)
            vb = (self.radius*(self.uCurr[1] + self.uCurr[0])) / 2.0
            wb = (self.radius*(self.uCurr[1] - self.uCurr[0])) / self.L
            vel = Twist()
            vel.linear.x = float(vb)
            vel.angular.z = float(wb)
            self.obstList = []
            if math.sqrt((self.currState[0] - self.end[0])**2 + (self.currState[1] - self.end[1])**2) <= self.goalThresh or self.count == len(self.pathDes) - 1 :
                vb = 0
                wb = 0
                vel.linear.x = float(vb)
                vel.angular.z = float(wb)
                self.publisher_.publish(vel)
                raise SystemExit
            endTime = time.time()
            timeTaken = endTime - startTime
            #print("time taken for optimization: ", timeTaken)
            waitTime = self.simStepTime - timeTaken
            if waitTime > 0:
                time.sleep(waitTime)
            self.publisher_.publish(vel)

       
        
    
def main(args=None):
    rclpy.init(args=args)

    MPC = MinimalSubscriber()

    
    try:
        rclpy.spin(MPC)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Goal Reached')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    MPC.destroy_node()
    rclpy.shutdown()

     

if __name__ == '__main__':
    main()

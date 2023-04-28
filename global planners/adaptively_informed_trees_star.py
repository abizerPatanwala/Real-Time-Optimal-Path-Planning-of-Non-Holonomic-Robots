"""
Adaptively Informed Trees (BIT*)
@author: Swapneel Wagholikar, Keshubh Sharma
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot

#sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                "/../../Sampling_based_Planning/")

import env, plotting, utils

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None

class Tree:
    def __init__(self,x_start,x_goal):
        self.x_start = x_start
        self.goal = x_goal
        self.r = 4.0
        self.Vf = set()
        self.Ef = set()
        self.Vr = set()
        self.Er = set()
        self.Qf = set()
        self.Qr = set()
        self.Einvalid = set()

class AITStar:
    def __init__(self,x_start, x_goal, eta, iter_max):
        self.x_start = Node(x_start[0], x_start[1])
        self.x_goal = Node(x_goal[0], x_goal[1])
        self.eta = eta
        self.iter_max = iter_max

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        self.fig, self.ax = plt.subplots()

        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range

        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        print(self.obs_rectangle)
        self.obs_boundary =  self.env.obs_boundary

        self.Tree = Tree(self.x_start, self.x_goal)
        self.X_sample = set()
        self.Ccurrent = 0.0
        self.hcon = dict()
        self.hexp = dict()

    def init(self):
        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        self.cMin = cMin
        self.theta = theta
        self.C = C
        self.m = 350
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],[(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])
        self.xCenter = xCenter
        self.Ccurrent = np.inf
        self.Tree.Vf.add(self.x_start)
        self.Tree.Vr.add(self.x_goal)
        self.X_sample.add(self.x_goal)
        self.X_sample.add(self.x_start)
        #self.Tree.Qf.add(self.x_start)
        self.Tree.Qr.add(self.x_goal)
        neighbors = self.expand(self.x_start,350,self.Ccurrent,cMin,xCenter,C)
        if(neighbors != None):
            for start,neighbor in neighbors:
                self.Tree.Qf.add((start,neighbor))
                #self.Tree.Ef.add((start,neighbor))
                self.hcon[neighbor] = 0
                self.hexp[neighbor] = self.h_estimated(neighbor)
        self.hcon[self.x_start] = 0.0
        self.hexp[self.x_start] = 0.0

        self.UpdateHeuristicNone()

        return theta, cMin, xCenter, C
        
    def planning(self):
        theta, cMin, xCenter, C = self.init()
        for k in range(1000):
            print("k = ",k)
            #print("EF length:-",len(self.Tree.Ef))
            print("QF length:-",len(self.Tree.Qf))
            #print("Ccurrent =",self.Ccurrent)
            #for xp,xc in self.Tree.Ef:
            #    print("parent: ",xp.x,",",xp.y)
            #    print("child: ",xc.x,",",xc.y)
            if k==0:
                m = 350
            else:
                m = 500
                self.m = m
            #print("checking for goal")
            if self.x_goal.parent is not None:
                path_x, path_y = self.ExtractPath()
                print("path_x",path_x)
                print("path_y",path_y)
                plt.plot(path_x, path_y, linewidth=2, color='r')
                plt.pause(0.5)

            #print("checking neighbors for qf")
            bestQf = self.GetBestFromForwardQueue()
            if(bestQf == None):
                #print("no node pair in qf")
                samples = self.Sample(m,self.Ccurrent,cMin,xCenter,C)
                for s in samples:
                    self.X_sample.add(s)
                    self.hexp[s] = self.h_estimated(s)
                    self.hcon[s] = 0
                expanded = self.expand(self.x_start,m,self.Ccurrent,cMin,xCenter,C)
                #self.Tree.Qf.clear()
                for p,xi in expanded:
                    self.Tree.Qf.add((self.x_start,xi))
                    self.hcon[xi] = 0
                    self.hexp[xi] = self.h_estimated(xi)
                #print("updating heuristic none")
                self.UpdateHeuristicNone()
                continue
            #print("Got the best pair in qf")
            self.Tree.Qf.remove(bestQf)
            xp,xc = bestQf
            #print("xp: ",xp.x,",",xp.y)
            #print("xc: ",xc.x,",",xc.y)
            a = self.g_forward_estimated(xp)
            b = self.cost(xp,xc)
            c = self.h_estimated(xc)
            d = self.g_forward_estimated(xp)
            #if xp != self.x_start:
                #print("149 a:",a," b:",b," c:",c," d:",d)
            if a + b + c < self.Ccurrent:
                #print("updating current tree")
                if bestQf in self.Tree.Ef:
                    #print("getting new node for children")
                    expanded = self.expand(xc,m,self.Ccurrent,cMin,xCenter,C)
                    if expanded != None:
                        for p in expanded:
                            t,xi = p
                            self.Tree.Qf.add((t,xi))
                    #print("expanding xc and adding to qf")
                elif self.g_forward_estimated(xp) + self.cost(xp,xc) <= self.g_forward_estimated(xc):
                    #print("elif 1")
                    if bestQf not in self.Tree.Einvalid and not self.utils.is_collision(xp,xc):
                        #print("if 2")
                        if self.g_forward_estimated(xp) + self.cost(xp,xc) + self.h_estimated(xc) <= self.Ccurrent:
                            #print("if 3")
                            if self.g_forward_estimated(xp) + self.cost(xp,xc) <= self.g_forward_estimated(xc):
                                #print("if 4")
                                if xc not in self.Tree.Vf:
                                    #print("adding xc to Vf")
                                    self.Tree.Vf.add(xc)
                                else:
                                    parent = self.getParentForward(xc)
                                    #print("removing xc parent",parent.x,",",parent.y,"and xc",xc.x,",",xc.y," from Ef")
                                    if parent != xc:
                                        self.Tree.Ef.remove((parent,xc))
                                #print("Adding to Ef : (",xp.x,",",xp.y,"), (",xc.x,",",xc.y,")")
                                self.Tree.Ef.add((xp,xc))
                                self.hcon[xp] = self.h_estimated(xp)
                                self.hcon[xc] = self.h_estimated(xc)
                                expanded = self.expand(xc,m,self.Ccurrent,cMin,xCenter,C)
                                if expanded is not None:
                                    #print("expanding xc ",xc.x,",",xc.y)
                                    for p,xi in expanded:
                                        self.Tree.Qf.add((xc,xi))
                                        self.hcon[xi] = 0
                                        self.hexp[xi] = self.h_estimated(xi)
                                if self.searchForwardTreeForChild(self.x_goal):
                                    self.Ccurrent = self.g_forward_estimated(self.x_goal)
                else:
                    #print("else 1")
                    self.UpdateHeuristic(xp,xc)
            else:
                #print("SAmpling new points")
                samples = self.Sample(m,self.Ccurrent,cMin,xCenter,C)
                for s in samples:
                    self.X_sample.add(s)
                expanded = self.expand(self.x_start,m,self.Ccurrent,cMin,xCenter,C)
                self.Tree.Qf.clear()
                for p,xi in expanded:
                    self.Tree.Qf.add((self.x_start,xi))
                self.UpdateHeuristicNone()
            
            if k%5 == 0:
                self.animation(xCenter,self.Ccurrent,cMin,theta)

        path_x, path_y = self.ExtractPath()
        plt.plot(path_x,path_y, linewidth=2,color='r')
        plt.pause(0.01)
        plt.show()
    
    def searchForwardTreeForChild(self,node):
        for xp,xc in self.Tree.Ef:
            if xc == node:
                return True
        return False

    def UpdateHeuristicNone(self):
        for x in self.Tree.Vr:
            self.hcon[x] = np.inf
            self.hexp[x] = np.inf
        self.hcon[self.x_goal] = np.inf
        self.hexp[self.x_goal] = self.h_estimated(self.x_goal)
        self.Tree.Qr.clear() 
        self.Tree.Qr.add(self.x_goal)
        x = self.GetBestFromReverseQueue()
        if x == None or x == self.x_goal:
            return
        x_key = self.getQrLexKey(x)
        self.Tree.Qr.remove(x)
        #print("216 x_key:",x_key)
        x_init_key = self.getQrLexKey(self.x_start)
        #print("221 x_init_key:",x_init_key)
        count = 0
        while((x_key[0]<x_init_key[0]) or (x_key[0]==x_init_key[0] and x_key[1]<x_init_key[1])) or (self.hexp[self.x_start] < self.hcon[self.x_start]):
            count += 1
            #print("221 count:",count)
            #print("226 x:",x.x,",",x.y)
            if(self.hcon[x] < self.hexp[x]):
                self.hexp[x] = self.hcon[x]
            else:
                self.hexp[x] = np.inf
                self.UpdateState(x)
            neighbors = self.get_neighbors(x,self.m,self.Ccurrent,self.cMin,self.xCenter,self.C)
            if(neighbors == None):
                best = self.GetBestFromReverseQueue()
                if best == None:
                    break
                continue
            for xi in neighbors:
                self.UpdateState(xi)
            best = self.GetBestFromReverseQueue()
            if best == None:
                break
            x_key = self.getQrLexKey(best)
            self.Tree.Qr.remove(best)
    

    def UpdateHeuristic(self,xp,xc):
        self.Tree.Einvalid.add((xp,xc))
        if (xp,xc) in self.Tree.Ef:
            self.Tree.Ef.remove((xp,xc))
        self.UpdateState(xp)
        x = self.GetBestFromReverseQueue()
        if x == None or x == self.x_goal:
            return
        x_key = self.getQrLexKey(x)
        self.Tree.Qr.remove(x)
        x_init_key = self.getQrLexKey(self.x_start)
        while((x_key[0]<x_init_key[0]) or (x_key[0]==x_init_key[0] and x_key[1]<x_init_key[1])) or (self.hexp[self.x_start] < self.hcon[self.x_start]):
            if(self.hcon[x] < self.hexp[x]):
                self.hexp[x] = self.hcon[x]
            else:
                self.hexp[x] = np.inf
                self.UpdateState(x)
            neighbors = self.get_neighbors(x,self.m,self.Ccurrent,self.cMin,self.xCenter,self.C)
            if(neighbors == None):
                continue
            for xi in neighbors:
                self.UpdateState(xi)
            best = self.GetBestFromReverseQueue()
            if best == None:
                break
            x_key = self.getQrLexKey(best)
            self.Tree.Qr.remove(best)
            

    def UpdateState(self,node):
        if node == self.x_start:
            return
        neighbors = self.get_neighbors(node,self.m,self.Ccurrent,self.cMin,self.xCenter,self.C)
        if(neighbors != None):
            minVal = np.inf
            xp = node
            for xi in neighbors:
                if(self.hexp[xi]+self.cost(xi,node) < minVal):
                    xp = xi
                    minVal = self.hexp[xi]+self.cost(xi,node)
            self.hcon[node] = self.hexp[xp] + self.cost(xp,node)
        if self.hcon[node] != self.hexp[node]:
            if node not in self.Tree.Qr:
                self.Tree.Qr.add(node)
        elif node in self.Tree.Qr:
            self.Tree.Qr.remove(node)

    def GetBestFromForwardQueue(self):
        minCost = [np.inf, np.inf, np.inf]
        minNode = None
        #if(len(self.Tree.Qf) == 1 and list(self.Tree.Qf)[0] == self.x_start):
        #    return None
        for r in self.Tree.Qf:
            cost = self.getQfLexKey(r)
            #print("297 cost:",cost)
            if(cost[0] < minCost[0]):
                minCost = cost
                minNode = r
            elif(cost[0] == minCost[0] and cost[1] < minCost[1]):
                minCost = cost
                minNode = r
            elif(cost[0] == minCost[0] and cost[1] == minCost[1] and cost[2] < minCost[2]):
                minCost = cost
                minNode = r
        #print("306 minNode:",minNode[0].x,",",minNode[0].y," ; ",minNode[1].x,",",minNode[1].y)
        return minNode

    def GetBestFromReverseQueue(self):
        minCost = [np.inf,np.inf]
        minNode = None
        for r in self.Tree.Qr:
            cost = self.getQrLexKey(r)
            if(cost[0] < minCost[0]):
                minCost = cost
                minNode = r
            elif(cost[0] == minCost[0] and cost[1] < minCost[1]):
                minCost = cost
                minNode = r
        return minNode
    
    def getQfLexKey(self,nodes):
        xp,xc = nodes
        key = [0,0,0]
        key[0] = self.g_forward_estimated(xp) + self.cost(xp,xc) + self.h_estimated(xc)
        key[1] = self.g_forward_estimated(xp) + self.cost(xp,xc)
        key[2] = self.g_forward_estimated(xp)
        return key

    def getQrLexKey(self,node):
        lexKey = list([0,0])
        lexKey[0] = min(self.hcon[node],self.hexp[node])
        lexKey[0] += self.g_estimated(node)
        lexKey[1] = min(self.hcon[node],self.hexp[node])
        return lexKey
    
    def cost(self, start, end):
        if self.utils.is_collision(start, end):
            return np.inf

        return self.calc_dist(start, end)

    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal)

    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node)
    
    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)
    
    def g_forward_estimated(self,node):
        found = False
        xp_1 = None
        xc_1 = None
        for xp,xc in self.Tree.Ef:
            xp_1 = xp
            xc_1 = xc
            if xc == node:
                found = True
                break
        if found:
            #print("g_forward_estimated xp:",xp_1.x,",",xp_1.y," xc:",xc_1.x,",",xc_1.y)
            return self.cost(xp_1,xc_1) + self.g_forward_estimated(xp_1)
        elif self.getParentForward(node) != None:
            return self.cost(self.getParentForward(node),node) + self.g_estimated(self.getParentForward(node))
        else: 
            return 1.5*self.g_estimated(node)

    def ExtractPath(self):
        node = self.x_goal
        path_x, path_y = [node.x], [node.y]
        while node.parent:
            node = node.parent
            path_x.append(node.x)
            path_y.append(node.y)
        return path_x, path_y

    def expand(self,x,m,cMax,cMin,xCenter,C):
        neighbors = self.get_neighbors(x,m,cMax,cMin,xCenter,C)
        if neighbors == None:
            return None
        e_out = set()
        for n in neighbors:
            e_out.add((x,n))
        return e_out
    
    def get_neighbors(self, x,m,cMax,cMin,xCenter,C):
        neighbors = {n for n in self.X_sample if self.calc_dist(x, n) <= self.Tree.r}
        parent_F = self.getParentForward(x)
        parent_R = self.getParentReverse(x)
        children_F = self.getChildrenForward(x)
        children_R = self.getChildrenReverse(x)
        
        if parent_F != None:
            neighbors.add(parent_F)
        if parent_R !=  None:
            neighbors.add(parent_R)
        if len(children_F) > 0:
            for c in children_F:
                neighbors.add(c)
        if len(children_R) > 0:
            for c in children_R:
                neighbors.add(c)
        to_remove = set()
        for n in neighbors:
            self.hexp[n] = self.h_estimated(n)
            self.hcon[n] = self.h_estimated(n)
            if self.utils.is_collision(x, n) or (x,n) in self.Tree.Einvalid:
                to_remove.add(n)
            if n == x:
                to_remove.add(n)
        for n in to_remove:
            neighbors.remove(n)
        if(len(neighbors) > 0):
            return neighbors
        else:
            return None

    def getParentForward(self,x):
        parent_F = None
        for (u,v) in self.Tree.Ef:
            if(v == x):
                parent_F = v
        return parent_F
    
    def getParentReverse(self,x):
        parent_R = None
        for (u,v) in self.Tree.Er:
            if(v == x):
                parent_R = v
        return parent_R
    
    def getChildrenForward(self,x):
        children_F = set()
        for (u,v) in self.Tree.Ef:
            if(u == x):
                children_F.add(v)
        return children_F
    
    def getChildrenReverse(self,x):
        children_R = set()
        for (u,v) in self.Tree.Er:
            if(u == x):
                children_R.add(v)
        return children_R

    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.x - end.x, start.y - end.y)
    
    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    
    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],[(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T
        return C

    def Sample(self, m, cMax, cMin, xCenter, C):
        if cMax < np.inf:
            return self.SampleEllipsoid(m, cMax, cMin, xCenter, C)
        else:
            return self.SampleFreeSpace(m)

    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
        L = np.diag(r)

        ind = 0
        delta = self.delta
        Sample = set()

        while ind < m:
            xBall = self.SampleUnitNBall()
            x_rand = np.dot(np.dot(C, L), xBall) + xCenter
            node = Node(x_rand[(0, 0)], x_rand[(1, 0)])
            in_obs = self.utils.is_inside_obs(node)
            in_x_range = self.x_range[0] + delta <= node.x <= self.x_range[1] - delta
            in_y_range = self.y_range[0] + delta <= node.y <= self.y_range[1] - delta

            if not in_obs and in_x_range and in_y_range:
                Sample.add(node)
                ind += 1

        return Sample

    def SampleFreeSpace(self, m):
        delta = self.utils.delta
        Sample = set()

        ind = 0
        while ind < m:
            node = Node(random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                        random.uniform(self.y_range[0] + delta, self.y_range[1] - delta))
            if self.utils.is_inside_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1
        return Sample
    
    def animation(self, xCenter, cMax, cMin, theta):
        plt.cla()
        self.plot_grid("Batch Informed Trees (BIT*)")

        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        for v in self.X_sample:
            plt.plot(v.x, v.y, marker='.', color='lightgrey', markersize='2')

        if cMax < np.inf:
            self.draw_ellipse(xCenter, cMax, cMin, theta)

        for v, w in self.Tree.Ef:
            plt.plot([v.x, w.x], [v.y, w.y], '-g')

        plt.pause(2)

    def plot_grid(self, name):
        for (ox, oy, w, h) in self.obs_boundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.x_start.x, self.x_start.y, "bs", linewidth=3)
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def SampleUnitNBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    @staticmethod
    def draw_ellipse(x_center, c_best, dist, theta):
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.2)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, marker='.', color='darkorange')
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)

def main():
    #x_start = (18, 8)  # Starting node
    #x_goal = (37, 18)  # Goal node
    x_start = (24, 6)  # Starting node
    x_goal = (11, 28)  # Goal node
    eta = 2
    iter_max = 200
    print("start!!!")
    ait = AITStar(x_start, x_goal, eta, iter_max)
    # bit.animation("Batch Informed Trees (BIT*)")
    ait.planning()


if __name__ == '__main__':
    main()
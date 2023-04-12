import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from shapely.geometry import Point,Polygon,LineString,MultiPolygon,box
import matplotlib.pyplot as plt
import random

fig=plt.figure("RRT Algorithm")
def plot_coords(coords):
    pts = list(coords)
    x,y = zip(*pts)
    plt.plot(x,y)
    
def plot_polys(polys):
    for poly in polys:
        if (not getattr(poly, "exterior", None)):
            print("got line?")

        plot_coords(poly.exterior.coords)

        for hole in poly.interiors:
            plot_coords(hole.coords)
            


coords = [(200, 600.169158), (350, 601.69158), (300.53510, 400.70104), (300.950958, 400.169990)]
obst1= Polygon(coords)

coords2 = [(400, 300), (500, 500), (550, 0),(600,200)]
obst2 = Polygon(coords2)

plot_polys([obst1,obst2])
gridh=800
gridw=1000
plt.xlim([0, gridw])
plt.ylim([0, gridh])

##print(Point((300,529)).within(Polygon(coords)))


flag=False
start=np.array([200,300])
goal=np.array([700,250])
numIter=200
stepSize=80
goalReg=plt.Circle((goal[0],goal[1]),stepSize-50,color='b',fill=False)


##np.set_printoptions(precision=3,suppress=True)

class treeNode():
    def __init__(self,locx,locy):
        self.locx=locx
        self.locy=locy
        self.children=[]
        self.parent=None

class RRT():
    def __init__(self,start,goal,numIter,stepSize):
        self.randomTree=treeNode(start[0],start[1])
        self.goal=treeNode(goal[0],goal[1])
        self.nearestNode=None
        self.iter=min(numIter,200)
        self.rho=stepSize
        self.path_distance=0
        self.nearestDist=10000
        self.numWaypoints=0
        self.Waypoints=[]
        
    def sampleAPoint(self):
        x=random.randint(1,gridw)
        y=random.randint(1,gridh)
        point=np.array([x,y])
        return point
    
    def findNearest(self,root,point):
        if not root:
            return
        dist=self.distance(root,point)
        ##                
        if dist<= self.nearestDist:
            self.nearestNode =root
            self.nearestDist=dist
        for child in root.children:
            self.findNearest(child,point)
##        self.nearestDist=self.distance(self.nearestNode,point)
##        for child in root.children:
##            if (self.distance(child,point)<min):
##                min=self.distance(child,point)
##                self.nearestNode =child
##                self.nearestDist=self.distance(child,point)

    def steerToPoint(self,locStart,locEnd):
        offset=self.rho*self.unitVector(locStart,locEnd)
        point=np.array([locStart.locx +offset[0],locStart.locy +offset[1]])
        if (point[0]>=gridw):
            point[0]=gridw
        if (point[1] >= gridh):
            point[1]=gridh

        return point


    def isInObstacle(self,locStart,locEnd):
        u_hat=self.unitVector(locStart,locEnd)
        testPoint1=np.array([0,0])
        
        for i in range(self.rho):
            testPoint1[0]=locStart.locx + i*u_hat[0]
            testPoint1[1]=locStart.locy + i*u_hat[1]
            if (testPoint1[0]>=gridw):
                testPoint1[0]=gridw-1
            if (testPoint1[1] >= gridh):
                testPoint1[1]=gridh-1
            testPoint=Point([testPoint1[0],testPoint1[1]])
           
            
            if (testPoint.within(obst1) or testPoint.within(obst2)):
                return True
        return False
        
    def addChild(self,locx,locy):
        if(locx ==self.goal.locx):
            self.nearestNode.children.append(self.goal)
            self.goal.parent=self.nearestNode
        else:
            tempNode=treeNode(locx,locy)
            self.nearestNode.children.append(tempNode)
            tempNode.parent=self.nearestNode
        

    def unitVector(self,locStart,locEnd):
        v=np.array([locEnd[0] -locStart.locx, locEnd[1] - locStart.locy])
        u=v/np.linalg.norm(v)
        return u




    def distance(self,node1,point):
        dist=np.sqrt(((node1.locx -point[0])**2) + ((node1.locy-point[1])**2))
        return dist

    def goalFound(self,point):
        if self.distance(self.goal,point)<=self.rho:
            return True
        pass

    def resetNearestValues(self):
        self.nearestNode=None
        self.nearestDist=10000

    def retraceRRT(self,goal):
        if goal.locx==self.randomTree.locx:
            return
        self.numWaypoints+=1

        currentPoint=np.array([goal.locx,goal.locy])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance+=self.rho
        self.retraceRRT(goal.parent)

##
##grid=np.load('cspace.npy')
##print(len(grid))
##print(grid.shape)



plt.plot(start[0],start[1],'rx')
plt.plot(goal[0],goal[1],'bx')
ax=fig.gca()
ax.add_patch(goalReg)

rrt=RRT(start,goal,numIter,stepSize)

for i in range(rrt.iter):
    rrt.resetNearestValues()
    print("interation:",i)
    point=rrt.sampleAPoint()
    print(point)
    rrt.findNearest(rrt.randomTree,point)
    print(rrt.randomTree.children)
    print("nearestNode")
    print(rrt.nearestNode.children)
    print(rrt.nearestNode.locy)
    new=rrt.steerToPoint(rrt.nearestNode,point)
    print(new[0])
    print(new[1])
    bool=rrt.isInObstacle(rrt.nearestNode,new)
    if(bool==False):
        rrt.addChild(new[0],new[1])
        plt.pause(0.1)
        plt.plot([rrt.nearestNode.locx,new[0]],[rrt.nearestNode.locy,new[1]],'gx',linestyle='--')

        if (rrt.goalFound(new)):
            rrt.addChild(goal[0],goal[1])
            print('goal found!')
            flag=True
            break
if(flag):
    rrt.retraceRRT(rrt.goal)
    rrt.Waypoints.insert(0,start)
    print("num of way points",rrt.numWaypoints)
    print("path distance:",rrt.path_distance)
        

    for i in range(len(rrt.Waypoints)-1):
        plt.plot([rrt.Waypoints[i][0],rrt.Waypoints[i+1][0]],[rrt.Waypoints[i][1],rrt.Waypoints[i+1][1]],'rx')

else:
    print('try more iterations')

   

plt.show()

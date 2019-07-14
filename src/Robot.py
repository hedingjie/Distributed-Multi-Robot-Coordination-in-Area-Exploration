import numpy as np

import AStar
import utils
import Config

# 参数
ALPHA=0.5
W1=1
W2=1
W3=1.5

# 感知范围
RS=1
# 通信范围
RC=15


class Robot(object):

    def __init__(self,world,id,x,y,rs=RS,rc=RC):
        self._id=id
        self._x=x
        self._y=y
        self._rs=rs
        self._rc=rc
        self._world = world
        self.height,self.width=world.map.shape
        self.localMap=np.full((self.height,self.width),-1)
        # 目前子网下的智能体
        self._knownRobots=[]
        self._subnet=[]
        self._world.addRobot(self)
        self.lambda_i=0
        self.cmds=[]

    def getId(self):
        return self._id

    def xy(self):
        return (self._x,self._y)

    def getRS(self):
        return self._rs

    def getRC(self):
        return self._rc

    def getLocalMap(self):
        return self.localMap

    def getKnownRobots(self):
        return self._knownRobots

    def setId(self,id):
        return self._id

    def setLocalMap(self,map):
        self.localMap = map

    def setKnownRobots(self,robotList):
        self._knownRobots=robotList

    def sense(self):
        """get area in sense"""
        wMap=self._world.map
        rowNum,colNum=wMap.shape
        xMin=(self._x-self._rs) if (self._x-self._rs >= 0) else 0
        xMax=(self._x+self._rs) if (self._x+self._rs < colNum-self._rs) else colNum-self._rs

        yMin = (self._y - self._rs) if (self._y - self._rs >= 0) else 0
        yMax = (self._y + self._rs) if (self._y + self._rs < rowNum - self._rs) else rowNum-self._rs


        # -1表示未探索，0表示以探索为free,1表示已探索被占用
        x,y = self.xy()
        try:
            for i in range(xMin,xMax+1,1):
                for j in range(yMin,yMax+1,1):
                    self.localMap[j][i]=wMap[j][i]
                    self._world.mask[j][i]=1
        except Exception:
            pass
        if Config.DRAW:
            utils.drawSense(self.getLocalMap(),self)
            utils.drawRobot(self.getLocalMap(),self)

    def move(self,cmd):
        """change its position according cmd [R,L,U,D]"""
        wMap=self._world.map
        height,width=wMap.shape
        print(self,'direct:',cmd)
        x,y = self.xy()
        # right
        if cmd == 'R':
            if(x < width-1 and wMap[y][x+1] != 1):
                self._x=self._x+1
                # self._y=self._y+1
        # left
        elif cmd == 'L':
            if (x>0 and wMap[y][x-1]!=1):
                self._x=self._x-1
                # self._y=self._y-1
        # up
        elif cmd == 'D':
            if (y<height-1 and wMap[y+1][x]!=1):
                # self._x=self._x-1
                self._y=self._y+1
        # down
        else:
            if (y > 0 and wMap[y-1][x]!=1):
                # self._x=self._x+1
                self._y=self._y-1

    def updateKnownRobots(self):
        '''update robots in the communicate limits after changing its position'''
        knownRobots=[]
        robotList=self._world.getRobotList()
        for robot in robotList:
            x,y = robot.xy()
            if self.getId()!=robot.getId() and utils.length(self._x,self._y,x,y) <= self._rc:
                knownRobots.append(robot)
        self._knownRobots=knownRobots


    def chooseDest(self):
        '''choose a destination for the next step'''
        x,y=self.xy()
        x_i,y_i=x,y
        gi=-9999999999999999
        fronterList=utils.getFronter(self.localMap)
        # print('__________________________________')
        for fronter in fronterList:
            value=utils.getLambda(self._world,self,fronter[0],fronter[1])
            lmda_i=value if value!=None else 0
            d_i=abs(fronter[0]-x)+abs(fronter[1]-y)
            g=W1*fronter[2]-W2*d_i+W3*lmda_i
            # print('g:%f',g)
            if g>gi:
                gi=g
                x_i=fronter[0]
                y_i=fronter[1]

        print('Robot%d at (%d,%d) Choose Dest:(%d,%d,%d)'%(self._id,x,y,x_i,y_i,gi))
        # print('__________________________________')
        return (x_i,y_i,gi)

    def explorationByOneStep(self):
        """exploration map by one step"""
        start_x,start_y=self.xy()
        end=self.chooseDest()
        if len(self.cmds)==0:
            # if the cmd is empty, navigate again
            self.cmds=AStar.navigate(self._world.map,start_x,start_y,end[0],end[1])
        cmd = self.cmds.pop(0)
        self.move(cmd)
        self.sense()
        self.updateKnownRobots()

    def __str__(self):
        return "Robot"+str(self._id)+" at ("+str(self._x)+','+str(self._y)+")"












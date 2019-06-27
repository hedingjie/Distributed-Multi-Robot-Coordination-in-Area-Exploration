import math
import random
import operator
import numpy as np
import Navigation

import utils

WORLD_WIDTH=50
WORLD_HEIGHT=50

ALPHA=0.5
W1=1
W2=1
W3=1.5

RS=1
RC=3

class Robot(object):

    def __init__(self,world,id,x,y,rs=RS,rc=RC):
        self._id=id
        self._x=x
        self._y=y
        self._rs=rs
        self._rc=rc
        self.localMap=np.full((WORLD_HEIGHT,WORLD_WIDTH),-1)
        # 目前子网下的智能体
        self._knownRobots=[]
        self._subnet=[]
        self._world=world
        self._world.addRobot(self)
        self.lambda_i=0

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
        # 计算大致的感知区域
        wMap=self._world.map
        rowNum,colNum=wMap.shape
        xMin=(self._x-self._rs) if (self._x-self._rs >= 0) else 0
        xMax=(self._x+self._rs) if (self._x+self._rs < colNum-self._rs) else colNum-self._rs

        yMin = (self._y - self._rs) if (self._y - self._rs >= 0) else 0
        yMax = (self._y + self._rs) if (self._y + self._rs < rowNum - self._rs) else rowNum-self._rs


        # -1表示未探索，0表示以探索为free,1表示已探索被占用
        try:
            for i in range(xMin,xMax+1,1):
                for j in range(yMin,yMax+1,1):
                    self.localMap[j][i]=wMap[j][i]
                    # if utils.length(self._x,self._y,i,j) <= self._rs :
                    #     self.localMap[j][i]=world[j][i]
                    # else:
                    #     pass
        except Exception:
            pass

    def move(self,cmd):
        # 测试移动功能，此处为随机游走
        wMap=self._world.map
        height,width=wMap.shape
        print(self,'direct:',cmd)
        x,y = self.xy()
        # 右
        if cmd == 'R':
            if(x < width-1 and wMap[y][x+1] != 1):
                self._x=self._x+1
                # self._y=self._y+1
        # 左
        elif cmd == 'L':
            if (x>0 and wMap[y][x-1]!=1):
                self._x=self._x-1
                # self._y=self._y-1
        # 上
        elif cmd == 'D':
            if (y<height-1 and wMap[y+1][x]!=1):
                # self._x=self._x-1
                self._y=self._y+1
        # 下
        else:
            if (y > 0 and wMap[y-1][x]!=1):
                # self._x=self._x+1
                self._y=self._y-1

    # 更新子网的情况，必须在所有的智能体都更新了位置之后才能更新子网
    def updateKnownRobots(self):
        knownRobots=[]
        robotList=self._world.getRobotList()
        for robot in robotList:
            x,y = robot.xy()
            if self.getId()!=robot.getId() and utils.length(self._x,self._y,x,y) <= self._rc:
                knownRobots.append(robot)
        self._knownRobots=knownRobots

    def communicate(self):
        robotList=utils.getSubnet(self._world.get)
        for robot in robotList:
            map = utils.mergeMap(self.localMap,robot.getLocalMap())
            self.localMap=map
            robot.setLocalMap(map)

    def broadcast(self):
        for robot in self._knownRobots:
            if operator.eq(self.localMap,robot.getKnownRobots):
                # 如果二者的地图信息相同，则不用更新
                return
            else:
                # 二者地图信息不同，融合二者的地图信息并进行更细
                map = utils.mergeMap(self.localMap,robot.getLocalMap)
                self.localMap=map
                robot.setLocalMap(map)

    def chooseDest(self):
        # 选择下一个目标点
        # robotList=self._world.getRobotList()
        # subnets=utils.getSubnet(robotList)
        # for subnet in subnets:
        #     if self._id in subnet:
        #         self._subnet=subnet
        # print(self._subnet)
        #
        # # 计算lambda
        # lambda_i=0
        # weight=1
        # x1,y1=self.xy()
        # for r_id in subnet:
        #     if self._id != r_id:
        #         robot=robotList[r_id]
        #         x2,y2=robot.xy()
        #         d=utils.length(x1,y1,x2,y2)
        #         lambda_i=lambda_i+weight*math.pow(math.e,int(-d/self.getRC()))
        #         weight=weight*ALPHA
        # self.lambda_i=lambda_i
        # print(self.lambda_i)
        x,y=self.xy()
        x_i,y_i=x,y
        gi=-9999999999999999999
        fronterList=utils.getFronter(self.localMap)
        print('__________________________________')
        for fronter in fronterList:
            value=utils.getLambda(self._world,self,fronter[0],fronter[1])
            lmda_i=value if value!=None else 0
            d_i=abs(fronter[0]-x)+abs(fronter[1]-y)
            g=W1*fronter[2]-W2*d_i+W3*lmda_i
            print('g:%f',g)
            if g>gi:
                gi=g
                x_i=fronter[0]
                y_i=fronter[1]

        print('__________________________________')
        if len(fronterList)==0:
            print('Hello')

        print('Robot at (%d,%d) Choose Dest:(%d,%d,%d)'%(x,y,x_i,y_i,gi))
        return (x_i,y_i,gi)

    def exploration(self):
        pass

    def __str__(self):
        return "Robot"+str(self._id)+" at ("+str(self._x)+','+str(self._y)+")"












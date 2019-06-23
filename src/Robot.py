import math
import random
import operator
import numpy as np

import utils

WORLD_WIDTH=50
WORLD_HEIGHT=50

RS=1
RC=2

class Robot(object):
    def __init__(self,x,y,rs=RS,rc=RC):
        self._x=x
        self._y=y
        self._rs=rs
        self._rc=rc
        # self.localMap=[
        #     [-1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1],
        #     [-1, -1, -1, -1, -1, -1]
        # ]

        self.localMap=np.full((WORLD_HEIGHT,WORLD_WIDTH),-1)
        # 目前子网下的智能体
        self._knownRobots=[]

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

    def setLocalMap(self,map):
        self.localMap = map

    def setKnownRobots(self,robotList):
        self._knownRobots=robotList

    def sense(self,world):
        # 计算大致的感知区域
        rowNum,colNum=world.shape
        xMin=(self._x-self._rs) if (self._x-self._rs >= 0) else (0)
        xMax=(self._x+self._rs) if (self._x+self._rs < colNum) else colNum

        yMin = (self._y - self._rs) if (self._y - self._rs >= 0) else (0)
        yMax = (self._y + self._rs) if (self._y + self._rs < rowNum) else rowNum


        # -1表示未探索，0表示以探索为free,1表示已探索被占用
        for i in range(xMin,xMax,1):
            for j in range(yMin,yMax,1):
                self.localMap[j][i]=world[j][i]
                # if utils.length(self._x,self._y,i,j) <= self._rs :
                #     self.localMap[j][i]=world[j][i]
                # else:
                #     pass

    def move(self,world):
        # 测试移动功能，此处为随机游走
        height,width=world.shape
        while True:
            num=int(random.random()*4)
            print(self,'num:',num)
            x,y = self.xy()
            if num == 0:
                if(x < width-1 and world[y][x+1]!=1 ):
                    self._x=self._x+1
                    # self._y=self._y+1
                    break
                else:
                    continue
            elif num==1:
                if (x>0 and world[y][x-1]!=1):
                    self._x=self._x-1
                    # self._y=self._y-1
                    break
                else:
                    continue
            elif num==2:
                if (y<height-1 and world[y+1][x]!=1):
                    # self._x=self._x-1
                    self._y=self._y+1
                    break
                else:
                    continue
            else:
                if (y > 0 and world[y-1][x]!=1):
                    # self._x=self._x+1
                    self._y=self._y-1
                    break
                else:
                    continue

    # 更新子网的情况，必须在所有的智能体都更新了位置之后才能更新子网
    def updateKnownRobots(self,robotList):
        knownRobots=[]
        for robot in robotList:
            x,y = robot.xy()
            if utils.length(self._x,self._y,x,y) <= self._rc:
                knownRobots.append(robot)
        self._subnet=knownRobots

    def communicate(self):
        robotList=self._knownRobots
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



    def bid(self):
        pass

    def __str__(self):
        return "Robot at ("+str(self._x)+','+str(self._y)+")"











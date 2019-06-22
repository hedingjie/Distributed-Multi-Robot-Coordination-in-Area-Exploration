import math
import random

import utils

class Robot(object):
    def __init__(self,x,y,rs,rc):
        self._x,self._y=utils.add_some_noise(x,y)
        self._rs=rs
        self._rc=rc
        self.localMap=[
            [-1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1],
            [-1, -1, -1, -1, -1, -1]
        ]
        # 目前子网下的智能体
        self._subnet=[]

    @property
    def xy(self):
        return self._x,self._y

    @property
    def getRS(self):
        return self._rs

    @property
    def getRC(self):
        return self._rc

    @property
    def getLocalMap(self):
        return self.localMap

    @property
    def getSubset(self):
        return self.setSubnet

    @property
    def setLocalMap(self,map):
        self.localMap = map

    @property
    def setSubnet(self,robotList):
        self._subnet=robotList

    @property
    def sense(self,world):
        # 计算大致的感知区域
        xMin=(self._x-self._rs) if (self._x-self._rs >= 0) else (0)
        xMax=(self._x+self._rs) if (self._x+self._rs < len(world)) else len(world)

        yMin = (self._y - self._rs) if (self._y - self._rs >= 0) else (0)
        yMax = (self._y + self._rs) if (self._y + self._rs < len(world)) else len(world)


        # -1表示未探索，0表示以探索为free,1表示已探索被占用
        for i in range(xMin,xMax,1):
            for j in range(yMin,yMax,1):
                if utils.length(self._x,self._y,i,j) <= self._rs :
                    self.localMap[j][i]=world[j][i]
                else:
                    pass

    @property
    def move(self,world):
        # 测试移动功能，此处为随机游走
        while True:
            num=int(random.random()*4)
            x,y = self.xy()
            if num == 0:
                if(world[y][x] == 1 or y >= len(world) or y<=0 or x >= len(world[0]) or x <= 0 ):
                    self._x=self._x+1
                    self._y=self._y+1
                    break
                else:
                    continue
            elif num==1:
                if (world[y][x] == 1 or y >= len(world) or y <= 0 or x >= len(world[0]) or x <= 0):
                    self._x=self._x-1
                    self._y=self._y-1
                    break
                else:
                    continue
            elif num==2:
                if (world[y][x] == 1 or y >= len(world) or y <= 0 or x >= len(world[0]) or x <= 0):
                    self._x=self._x-1
                    self._y=self._y+1
                    break
                else:
                    continue
            else:
                if (world[y][x] == 1 or y >= len(world) or y <= 0 or x >= len(world[0]) or x <= 0):
                    self._x=self._x+1
                    self._y=self._y-1
                    break
                else:
                    continue

    # 更新子网的情况，必须在所有的智能体都更新了位置之后才能更新子网
    @property
    def updateSubnet(self,robotList):
        subset=[]
        for robot in robotList:
            x,y = robot.xy()
            if utils.length(self._x,self._y,x,y) <= self._rc:
                subset.append(robot)
        self._subnet=subset

    @property
    def communicate(self):
        robotList=self.setSubnet
        for robot in robotList:
            map = utils.mergeMap(self.localMap,robot.getLocalMap())
            self.localMap=map
            robot.setLocalMap(map)

    @property
    def bid(self):
        pass














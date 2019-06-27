import random
import math
import networkx as nx
import matplotlib.pyplot as plt
import turtle
import numpy as np

import Robot

ALPHA=0.5

def add_noise(level, *coords):
    return [x+random.uniform(-level,level) for x in coords]

def add_some_noise(*coords):
    return add_noise(0.1,coords)

def length(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def mergeMap(robotList):
    height,width=robotList[0].getLocalMap().shape
    mergedMap=np.full((height,width),-1)
    for robot in robotList:
        map=robot.getLocalMap()
        for j,_ in enumerate(map):
            for i,_ in enumerate(map[0]):
                if map[j][i] != -1:
                    mergedMap[j][i]=map[j][i]
                else:
                    pass
    return mergedMap

def shareMsg(r1, r2):
    x1,y1 = r1.xy()
    x2,y2 = r2.xy()
    if length(x1,y1,x2,y2) <= r1.getRS():
        map=mergeMap(r1.getLocalMap(),r2.getLocalMap())
        r1.setLocalMap(map)
        r2.setLocalMap(map)

def getSubnet(world):
    robotList=world.getRobotList()
    G = nx.Graph()
    for node in robotList:
        G.add_node(node.getId())
        for node_ in node.getKnownRobots():
            G.add_edge(node.getId(),node_.getId())

    return (list(nx.connected_components(G)))

def getLambda(world,robot,x,y):
    id=robot.getId()
    subnet={}
    robotList = world.getRobotList()
    subnets = getSubnet(world)
    for subnet in subnets:
        if id in subnet:
            subnet = subnet
    # print('subnet:'+str(subnet))

    # 计算lambda
    lambda_i = 0
    weight = 1
    x1, y1 = x,y
    for r_id in subnet:
        if id != r_id:
            robot = robotList[r_id]
            x2, y2 = robot.xy()
            d = length(x1, y1, x2, y2)
            lambda_i = lambda_i + weight * math.pow(math.e, int(-d / robot.getRC()))
            weight = weight * ALPHA
    # print('lambda:'+str(lambda_i))

def getUnkownRate(map,x,y):
    height,width=map.shape
    xMax=(x+2) if x+2 < (width-2-1) else width
    xMin=(x-2) if x-2 >0 else 0
    yMax = (y + 2) if y + 2 < (height - 2 - 1) else height
    yMin = (y - 2) if y - 2 > 0 else 0
    cnt=0
    for y_ in range(yMin, yMax+1):
        for x_ in range(xMin,xMax+1):
            if map[y][x]==-1:
                cnt+=1
            else:
                pass
    return cnt/((xMax-xMin+1)*(yMax-yMin+1))

def getFronter(map):
    """Get the fronter grid"""
    height,width=map.shape
    fronterList=[]
    for y,line in enumerate(map):
        for x,block in enumerate(line):
            if map[y][x]==0:
                if y+1<=height-1 and map[y+1][x] == -1:
                    fronterList.append((x,y,getUnkownRate(map,x,y)))
                    continue
                if y-1>=0 and map[y-1][x] == -1:
                    fronterList.append((x,y,getUnkownRate(map,x,y)))
                    continue
                if x+1<=width-1 and map[y][x+1] == -1:
                    fronterList.append((x,y,getUnkownRate(map,x,y)))
                    continue
                if x-1>=0 and map[y][x-1] == -1:
                    fronterList.append((x,y,getUnkownRate(map,x,y)))
                    continue
                else:
                    continue
    if len(fronterList)==0:
        print("Hello")
    return fronterList

def Unexploration(map):
    return np.sum(map==-1)


# -------------------------------------------------------
#
#                        画图工具
#
# -------------------------------------------------------
def drawWorld(world):
    # draw the obstacles
    height,width=world.map.shape
    for x,y in world.obstacle:
        ny=height-y-1
        turtle.up()
        turtle.color('black')
        turtle.setposition(x,ny)
        turtle.down()
        turtle.setheading(0)
        turtle.begin_fill()
        for _ in range(4):
            turtle.fd(1)
            turtle.right(90)
        turtle.end_fill()
        turtle.up()

    # draw the free spaces
    for x, y in world.free:
        ny=height-y-1
        turtle.up()
        turtle.color('white')
        turtle.setposition(x, ny)
        turtle.down()
        turtle.setheading(0)
        turtle.begin_fill()
        for _ in range(4):
            turtle.forward(1)
            turtle.right(90)
        turtle.end_fill()
        turtle.up()

    # draw the unknown area
    for x, y in world.unknown:
        ny=height-y-1
        turtle.up()
        turtle.color('green')
        turtle.setposition(x, ny)
        turtle.down()
        turtle.setheading(0)
        turtle.begin_fill()
        for _ in range(4):
            turtle.forward(1)
            turtle.right(90)
        turtle.end_fill()
        turtle.up()

    turtle.update()


def drawMap(map):
    height,width=map.shape
    for y,line in enumerate(map):
        for x,block in enumerate(line):
            if block == -1:
                ny = height - y - 1
                turtle.up()
                turtle.color('green')
                turtle.setposition(x, ny)
                turtle.down()
                turtle.setheading(90)
                turtle.begin_fill()
                for _ in range(4):
                    turtle.fd(1)
                    turtle.right(90)
                turtle.end_fill()
                turtle.up()
            elif block == 0:
                ny = height - y - 1
                turtle.up()
                turtle.color('white')
                turtle.setposition(x, ny)
                turtle.down()
                turtle.setheading(90)
                turtle.begin_fill()
                for _ in range(4):
                    turtle.fd(1)
                    turtle.right(90)
                turtle.end_fill()
                turtle.up()
            elif block == 1:
                ny = height - y - 1
                turtle.up()
                turtle.color('black')
                turtle.setposition(x, ny)
                turtle.down()
                turtle.setheading(90)
                turtle.begin_fill()
                for _ in range(4):
                    turtle.fd(1)
                    turtle.right(90)
                turtle.end_fill()
                turtle.up()
            else:
                raise Exception('Unknown Block State at (%d,%d)'%(x,y))

    turtle.update()


def drawRobots(map,robotList):
    """call this function after the drawWorld() function"""
    robotList=robotList
    height,width=map.shape
    for robot in robotList:
        x,y=robot.xy()
        y=height-y-1
        turtle.up()
        turtle.color('blue')
        turtle.setposition(x,y)
        turtle.begin_fill()
        for _ in range(4):
            turtle.fd(1)
            turtle.right(90)
        turtle.end_fill()
    turtle.update()

def drawOneBlock(map,x,y,color):
    height,width=map.shape
    y=height-y-1
    turtle.up()
    turtle.color(color)
    turtle.setheading(0)
    turtle.setposition(x, y)
    turtle.down()
    turtle.setheading(90)
    turtle.begin_fill()
    for _ in range(4):
        turtle.forward(1)
        turtle.right(90)
    turtle.end_fill()
    turtle.up()
    turtle.update()

def drawSense(map,robot):
    rowNum, colNum = map.shape
    x, y = robot.xy()
    rs = robot.getRS()
    xMin = (x - rs) if (x - rs >= 0) else (0)
    xMax = (x + rs) if (x + rs < colNum - rs) else colNum - 1

    yMin = (y - rs) if (y - rs >= 0) else (0)
    yMax = (y + rs) if (y + rs < rowNum - rs) else rowNum - 1

    # -1表示未探索，0表示以探索为free,1表示已探索被占用
    for i in range(xMin, xMax + 1, 1):
        for j in range(yMin, yMax + 1, 1):
            try:
                if map[j][i] == 0:
                    drawOneBlock(map,i, j, 'white')
                elif map[j][i] == 1:
                    drawOneBlock(map,i, j, 'black')
                else:
                    pass
            except Exception as err:
                print('DrawSense Exception at Robot%d'%(robot.getId()))
                raise err


# -------------------------------------------------------
#
#                        通信模拟
#
# -------------------------------------------------------

def communicate(world):
    robotList=world.getRobotList()
    subnets=getSubnet(world)
    print('subnets:', subnets)
    for subnet in subnets:
        subRobotList=[]
        for id in subnet:
            subRobotList.append(robotList[id])
        mMap=mergeMap(subRobotList)
        print('communication between : ',subnet)
        for robot in subRobotList:
            robot.setLocalMap(mMap)







if __name__ == '__main__':
    r1=Robot.Robot(0,1,1)
    r2=Robot.Robot(1,2,2)
    r3=Robot.Robot(3,25,25)
    robot_list=[]

    robot_list.append(r1)
    robot_list.append(r2)
    robot_list.append(r3)
    r1.updateKnownRobots(robot_list)
    r2.updateKnownRobots(robot_list)
    r3.updateKnownRobots(robot_list)
    getSubnet(robot_list)



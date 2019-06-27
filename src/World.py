from __future__ import  absolute_import

import math
import turtle
import numpy as np
from Robot import *
from Navigation import *
import AStar


turtle.tracer(50000, delay=0)
turtle.register_shape("dot", ((-3,-3), (-3,3), (3,3), (3,-3)))
turtle.register_shape("tri", ((-3, -2), (0, 3), (3, -2), (0, 0)))
turtle.speed(0)
turtle.title("DMCAE")

UPDATE_EVERY = 0
DRAW_EVERY = 2

class TestRobot():
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.rs=2
        self.rc=3

    def xy(self):
        return self.x,self.y

    def getRS(self):
        return self.rs

    def getRC(self):
        return self.rc



class World(object):
    def __init__(self, map):
        self.map = map
        self.height,self.width=map.shape
        turtle.setworldcoordinates(0, 0, self.width, self.height)
        self.blocks = []
        self.unknown = []
        self.free = []
        self.obstacle = []
        self.update_cnt = 0
        self.one_px = float(turtle.window_width()) / float(self.width) / 2
        self.robot_list=[]

        for y, line in enumerate(self.map):
            for x, block in enumerate(line):
                # nb_y = self.height - y - 1
                self.blocks.append((x, y))
                if block == -1:  # 未知区域
                    # nb_y = self.height - y - 1
                    self.unknown.append((x, y))
                elif block == 0: # free区域
                    # nb_y = self.height - y - 1
                    self.free.append((x, y))
                elif block == 1: # 障碍区域
                    # nb_y = self.height - y - 1
                    self.obstacle.append((x, y))
                else:
                    print('Exception at (' + x +',' +y + ')')
                    raise Exception

    def addRobot(self,robot):
        self.robot_list.append(robot)

    def getRobotList(self):
        return self.robot_list

    def draw(self):
        for x,y in self.obstacle:
            turtle.up()
            turtle.color('black')
            turtle.setposition(x, y)
            turtle.down()
            turtle.setheading(90)
            turtle.begin_fill()
            for _ in range(4):
                turtle.forward(1)
                turtle.right(90)
            turtle.end_fill()
            turtle.up()

        for x,y in self.free:
            turtle.up()
            turtle.color('white')
            turtle.setposition(x, y)
            turtle.down()
            turtle.setheading(90)
            turtle.begin_fill()
            for _ in range(4):
                turtle.forward(1)
                turtle.right(90)
            turtle.end_fill()
            turtle.up()

        for x,y in self.unknown:
            turtle.up()
            turtle.color('green')
            turtle.setposition(x, y)
            turtle.down()
            turtle.setheading(90)
            turtle.begin_fill()
            for _ in range(4):
                turtle.forward(1)
                turtle.right(90)
            turtle.end_fill()
            turtle.up()

        # for i in range(50):
        #     # 画横格
        #     turtle.up()
        #     turtle.setposition(i,0)
        #     turtle.color('black')
        #     turtle.down()
        #     turtle.fd(50)
        #
        # for j in range(50):
        #     # 画纵格
        #     turtle.up()
        #     turtle.setheading(270)
        #     turtle.setposition(0,j)
        #     turtle.color('black')
        #     turtle.down()
        #     turtle.fd(50)




        # for x,y in self.blocks:
        #     turtle.up()
        #     turtle.color('black')
        #     turtle.setposition(x, y)
        #     turtle.down()
        #     turtle.setheading(90)
        #     for _ in range(4):
        #         turtle.forward(1)
        #         turtle.right(90)
        #     turtle.up()

        # for y, line in enumerate(self.world):
        #     for x, block in enumerate(line):
        #         ny=self.height-y-1
        #         turtle.up()
        #         turtle.setposition(x,ny)
        #         turtle.color('black')
        #         turtle.down()
        #         for _ in range(4):
        #             turtle.fd(1)
        #             turtle.right(90)




    def show_robots(self,robotList):
        for robot in robotList:
            (x,y) = robot.xy()
            y=self.height-y-1;
            # 画感知范围
            # turtle.up()
            # turtle.color('blue')
            # turtle.setposition(x-0.5+robot.getRS(),y-0.5)
            # turtle.down()
            # turtle.circle(robot.getRS())
            # 在这里为了方便，我们将感知范围设置为robot所在的block
            turtle.up()
            turtle.color('blue')
            turtle.setposition(x,y)
            turtle.begin_fill()
            for _ in range(4):
                turtle.fd(1)
                turtle.right(90)
            turtle.end_fill()
            # 画通信范围
            # turtle.up()
            # turtle.color('yellow')
            # turtle.setposition(x + 0.5 + robot.getRC(), y + 0.5)
            # turtle.down()
            # turtle.circle(robot.getRC())

            # 标注智能体
            turtle.up()
            turtle.setposition(x + 0.5, y + 0.5)
            turtle.down()
            turtle.color('red')
            turtle.dot(10)
        turtle.update()

    def draw_one_black(self,x,y,color):
        y=self.height-y-1
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

    def drawSense(self,robot):
        map = robot.getLocalMap()
        rowNum, colNum = map.shape
        x,y=robot.xy()
        rs=robot.getRS()
        xMin = (x - rs) if (x - rs >= 0) else (0)
        xMax = (x + rs) if (x + rs < colNum-rs) else colNum-1

        yMin = (y - rs) if (y - rs >= 0) else (0)
        yMax = (y + rs) if (y + rs < rowNum-rs) else rowNum-1

        # -1表示未探索，0表示以探索为free,1表示已探索被占用
        for i in range(xMin, xMax+1, 1):
            for j in range(yMin, yMax+1, 1):
                try:
                    if map[j][i] == 0:
                        self.draw_one_black(i,j,'white')
                    elif map[j][i] == 1:
                        self.draw_one_black(i,j,'black')
                    else:
                        pass
                except Exception as err:
                    print('Exception at drawSense')
                    raise err

    def drawSenses(self,robotList):
        for robot in robotList:
            self.drawSense(robot)
        turtle.update()










if __name__ == '__main__':
    worldData=np.array((
        (0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    ))

    world = World(worldData)
    # world.draw()
    r0=Robot(world,0,1,1)
    r1=Robot(world,1,25,25)
    r2=Robot(world,2,16,16)
    r3=Robot(world,3,1,25)

    utils.drawWorld(world)

    r0.sense()
    r1.sense()
    r2.sense()
    r3.sense()
    utils.drawMap(r1.getLocalMap())
    utils.drawRobots(r1.getLocalMap(),world.getRobotList())
    for i in range(1500):
        print('***********STEP',i,'***********')
        x1, y1 = r0.xy()
        dest1 = r0.chooseDest()
        cmds1 = AStar.navigate(worldData, x1, y1, dest1[0], dest1[1])
        for cmd in cmds1:
            r0.move(cmd)
            r0.sense()
            r0.updateKnownRobots()
            utils.communicate(world)

        x2,y2=r1.xy()
        dest2=r1.chooseDest()
        cmds2=AStar.navigate(worldData,x2,y2,dest2[0],dest2[1])

            # utils.drawSense(r0.getLocalMap(), r0)
            # utils.drawRobots(r0.getLocalMap(), world.getRobotList())
        for cmd in cmds2:
            r1.move(cmd)
            r1.sense()
            r1.updateKnownRobots()
            utils.communicate(world)
            # utils.drawRobots(r1.getLocalMap(), world.getRobotList())
            utils.drawSense(r1.getLocalMap(),r1)
            # utils.drawSense(r1.getLocalMap(), r1)
            # utils.drawRobots(r1.getLocalMap(), world.getRobotList())

        x3, y3 = r2.xy()
        dest3 = r2.chooseDest()
        cmds3 = AStar.navigate(worldData, x3, y3, dest3[0], dest3[1])
        for cmd in cmds1:
            r2.move(cmd)
            r2.sense()
            r2.updateKnownRobots()
            utils.communicate(world)

        x4, y4 = r3.xy()
        dest4 = r3.chooseDest()
        cmds4 = AStar.navigate(worldData, x4, y4, dest4[0], dest4[1])
        for cmd in cmds1:
            r3.move(cmd)
            r3.sense()
            r3.updateKnownRobots()
            utils.communicate(world)

        if(utils.Unexploration(r1.getLocalMap())==0):
            print('Iteration : ',i)
            break

        # utils.drawMap(r0.getLocalMap())



        # local_world.drawSenses(world.getRobotList())
        # local_world.show_robots(world.getRobotList())

    print('Done')

    # r1=Robot(1,1)
    # r2=Robot(1,5)
    # r3=Robot(25,25)
    # robotList=[]
    # robotList.append(r1)
    # robotList.append(r2)
    # robotList.append(r3)
    # # world.show_robots(robotList)
    # world2 = World(r1.getLocalMap())
    # world2.draw()
    # r1.sense(world)
    # r2.sense(world)
    # r3.sense(world)
    # world2.drawSenses(robotList)
    # world2.show_robots(robotList)
    # for i in range(10000):
    #     print(i)
    #     r1.move(world)
    #     r1.sense(world)
    #     r2.move(world)
    #     r2.sense(world)
    #     r3.move(world)
    #     r3.sense(world)
    #     world2.drawSenses(robotList)
    #     world2.show_robots(robotList)
    #     turtle.delay(50)
    # turtle.mainloop()

    # r0=Robot(world,0,1,1)
    # r1=Robot(world,1,2,2)
    # r2=Robot(world,2,25,25)
    # for robot in world.getRobotList():
    #     robot.updateKnownRobots(world.getRobotList())
    #
    # print(r0.chooseDest())


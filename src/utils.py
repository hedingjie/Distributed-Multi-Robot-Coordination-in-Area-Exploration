import random
import math

def add_noise(level, *coords):
    return [x+random.uniform(-level,level) for x in coords]

def add_some_noise(*coords):
    return add_noise(0.1,coords)

def length(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def mergeMap(map1,map2):
    mergedMap=[
        [-1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1],
        [-1, -1, -1, -1, -1, -1],
    ]
    for j,_ in enumerate(map1):
        for i,_ in enumerate(map1[0]):
            if map1[j][i] != -1:
                mergedMap[j][i]=map1[j][i]
            elif map2[j][i] != -1:
                mergedMap[j][i]=map2[j][i]
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
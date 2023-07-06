import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import math
import shapely
from shapely.geometry import LineString,Point, Polygon
import pandas as pd
import numpy as np








def init_map(address):

    xslx = pd.read_excel(address)
    rects = []
    centers = []
    
    for i in range(len(xslx)):
        data = xslx.iloc[i]
        p1 = [data.x1 , data.y1]
        p2 = [data.x2 , data.y2]
        p3 = [data.x3 , data.y3]
        p4 = [data.x4 , data.y4]
        rect=[ p1 , p3 , p4, p2 ]
        rects.append(rect)

        
    X1=xslx.loc[:,"x1"].to_numpy()
    X2=xslx.loc[:,"x2"].to_numpy()
    X3=xslx.loc[:,"x3"].to_numpy()
    X4=xslx.loc[:,"x4"].to_numpy()
    X=np.concatenate([X1,X2,X3,X4])
    
    Y1=xslx.loc[:,"y1"].to_numpy()
    Y2=xslx.loc[:,"y2"].to_numpy()
    Y3=xslx.loc[:,"y3"].to_numpy()
    Y4=xslx.loc[:,"y4"].to_numpy()
    Y=np.concatenate([Y1,Y2,Y3,Y4])
    
    map_boundry = (min(X) , max(X) , min(Y) , max(Y))
    global_map_pose = ['0.0', '0.0', '0.05', '0', '-0', '0']
    
    
    
    
    return rects,global_map_pose,map_boundry


def find_intersection(p1,p2,p3,p4):

    line1 = LineString([tuple(p1), tuple(p2)])
    line2 = LineString([tuple(p3), tuple(p4)])

    int_pt = line1.intersection(line2)
    if int_pt:
        point_of_intersection = int_pt.x, int_pt.y
        return point_of_intersection
    else:
        return False


def convert_point_to_line(rects):
    lines = []
    for points in rects:
        lines.append([ points[0] , points[1]] )
        lines.append([ points[1] , points[3]] )
        lines.append([ points[3] , points[2]] )
        lines.append([ points[2] , points[0]] )
    return lines


def add_offset(rects,offset):
    new_rects = []
    for points in rects: 
        new_rects.append(
            [  
                [points[0][0] + offset[0] , points[0][1] + offset[1]  ] ,
                [points[1][0] + offset[0] , points[1][1] + offset[1]  ] ,
                [points[2][0] + offset[0] , points[2][1] + offset[1]  ] ,
                [points[3][0] + offset[0] , points[3][1] + offset[1]  ] 
            ]
        )
    return new_rects


def convert_to_poly(rects):
    polygons = []
    for points in rects:
        polygons.append(Polygon(
            [tuple(points[0]) ,
            tuple(points[1]),
            tuple(points[3]),
            tuple(points[2])
            ]))
    return polygons

def check_is_collition(point , rects):
    p = Point(tuple(point))
    for rect in rects:
        if rect.contains(p):
            return True
    return False


def map_boundry(centers):
    X = []
    Y = [] 

    for item in centers:
        X.append(float(item[0]))
        Y.append(float(item[1]))

    return min(X),max(X),min(Y),max(Y)
    



def out_of_range(particle,offset,map_boundry):
    if particle[0] - offset[0] > map_boundry[1] or particle[0] - offset[0] < map_boundry[0]:
        return True
    elif particle[1] - offset[1] > map_boundry[3] or particle[1] - offset[1] < map_boundry[2]:
        return True
    else:
        return False

def plot_map(rects):
    for rect in rects:
        rect = list(zip(*rect))
        plt.plot(rect[1], rect[0], c='black')







#!/usr/bin/env python3

import os
import sys
import time
import math
import random
import pickle
from sympy import Point3D, Line3D, Segment3D, Point2D, Line2D, Segment2D
from datetime import datetime
import util
import copy
import carla

     
class npcSearch:

    def __init__(self, ego_info):
        self.sim = None 
        self.world = None
        self.junction_point_list = ego_info['junction_point']
        self.resultDic = {}
        self.initSimulator()
        self.ego_path=[[] for i in range(2)]


    def initSimulator(self):
        client = carla.Client('localhost', 2000)
        self.sim = client
        self.world = client.get_world()
        self.map = self.world.get_map()
        
        
    def generate_waypoints(self, pos):
        return self.map.get_waypoint(carla.Location(x=pos[0], y=pos[1], z=pos[2])), pos[3]

    def calculate_distance(self, loc1, loc2):
        dx = loc1.x - loc2.x
        dy = loc1.y - loc2.y
        dz = loc1.z - loc2.z
        return math.sqrt(math.pow(dx , 2) + math.pow(dy, 2) + math.pow(dz, 2))
    
    def get_distance_threshold(self, time_difference):
        base_threshold = 45.0 
        min_threshold = 5.0  
        max_time_difference = 30.0  
        return max(min_threshold, base_threshold - (base_threshold - min_threshold) * (time_difference / max_time_difference))

    def generate_npc_spawn(self):
        spawn_points = self.map.get_spawn_points()
        found_spawn_points = []
        for spawn_point in spawn_points:
            for i in range(len(self.junction_point_list)):
                waypos, time = self.generate_waypoints(self.junction_point_list[i])
                distance = self.calculate_distance(spawn_point.location, waypos.transform.location)
                time_difference = abs(time - distance / 5)
                threshold = self.get_distance_threshold(time_difference)
                if distance < threshold:
                    found_spawn_points.append([spawn_point.location.x, spawn_point.location.y, spawn_point.location.z, spawn_point.rotation.pitch, spawn_point.rotation.yaw, spawn_point.rotation.roll, i])
                    break
        self.resultDic['pos'] = found_spawn_points
        return self.resultDic


##################################### MAIN ###################################
egoPath = 'ego_path.obj'
egoF = open(egoPath, 'rb')
ego_info = pickle.load(egoF)
egoF.close()
resultDic = {}
sim = npcSearch(ego_info)
resultDic = sim.generate_npc_spawn()
resPath = 'npc_spawn.obj'
if os.path.isfile(resPath) == True:
    os.system("rm " + resPath)
f_f = open(resPath, 'wb')
pickle.dump(resultDic, f_f)
f_f.truncate() 
f_f.close() 


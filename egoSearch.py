#!/usr/bin/env python3

import os
import sys
import time
import math
import random
import pickle
from datetime import datetime
import util
import copy
import carla

     
class egoSearch:

    def __init__(self):
        self.sim = None 
        self.world = None
        self.tm = None
        self.ego = None 
        self.resultDic = {}
        self.numOfTimeSlice = 30
        self.initSimulator()
        self.initEV()
        self.ego_path=[[] for i in range(2)]
        self.junction_point_list = []


    def initSimulator(self):
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        self.sim = client
        self.world = client.get_world()
        self.tm = client.get_trafficmanager()
        self.tm.set_synchronous_mode(True)
        
        

    def initEV(self):
        sim = self.sim
        world = self.world
        world.tick()
        actor_list = world.get_actors()
        vehicles = actor_list.filter('vehicle.*')
        if len(vehicles) != 0:
            for vehicle in vehicles:
                sensors = actor_list.filter('sensor.*')
                for sensor in sensors:
                    if sensor.parent and sensor.parent.id == vehicle.id:
                        while(sensor.is_alive == True):
                            sensor.destroy()
                            print("more sensor destoried")
                while(vehicle.is_alive == True):
                    vehicle.destroy()
                    print("more vehicle destoried")

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.toyota.prius')
        vehicle_bp.set_attribute('role_name', 'hero')
        
        spawn_points = world.get_map().get_spawn_points()
        spawn_ego = random.choice(spawn_points)
        destination_ego = random.choice(spawn_points)
        while(destination_ego == spawn_ego):
            destination_ego = random.choice(spawn_points)

        
        ego = world.spawn_actor(vehicle_bp, spawn_ego)
        ego.set_autopilot(True)
        self.tm.set_path(ego, [destination_ego.location])
        self.ego = ego
        return spawn_ego, destination_ego


    def find_best(self):
        count = 0
        max_score = 0
        while(count<100):
            spawn_ego, destination_ego = self.initEV()
            world = self.world
            ego = self.ego
            junction = []
            junction_point = []
            for t in range(0, int(self.numOfTimeSlice)):
                for j in range(0, 20):
                    world.tick()
                ego_position = ego.get_location()
                ego_waypoint = world.get_map().get_waypoint(ego_position)
                if ego_waypoint.is_junction:
                    if ego_waypoint.get_junction().id not in junction:
                        junction.append(ego_waypoint.get_junction().id)
                    ego_pos = [ego_position.x, ego_position.y, ego_position.z, t]
                    if ego_pos not in junction_point:
                        junction_point.append(ego_pos)
            score = len(junction)
            if score > max_score:
                max_score=score
                path = [
                    [
                        spawn_ego.location.x,
                        spawn_ego.location.y,
                        spawn_ego.location.z,
                        spawn_ego.rotation.pitch,
                        spawn_ego.rotation.yaw,
                        spawn_ego.rotation.roll
                    ],
                    [
                        destination_ego.location.x,
                        destination_ego.location.y,
                        destination_ego.location.z
                    ]
                ]
                self.ego_path = copy.deepcopy(path)
                self.junction_point_list = copy.deepcopy(junction_point)
            count += 1
        self.resultDic['ego_pos'] = self.ego_path
        self.resultDic['junction_point'] = self.junction_point_list
        self.resultDic['score'] = max_score
                    
        return self.resultDic


##################################### MAIN ###################################

resultDic = {}
sim = egoSearch()
resultDic = sim.find_best()
resPath = 'ego_path.obj'
if os.path.isfile(resPath) == True:
    os.system("rm " + resPath)
f_f = open(resPath, 'wb')
pickle.dump(resultDic, f_f)
f_f.truncate() 
f_f.close() 


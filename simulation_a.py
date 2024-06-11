#!/usr/bin/env python3

import os
import lgsvl
import sys
import time
import math
import random
import pickle
from sympy import Point3D, Line3D, Segment3D, Point2D, Line2D, Segment2D
import liability
from datetime import datetime
import util
import copy
import carla

     
class LgApSimulation:

    def __init__(self):
        ################################################################
        self.bridgeLogPath = "/home/av-input/Workplace/apollo-lg/apollo-3.5/data/log/cyber_bridge.INFO"
        ################################################################
        self.sim = None 
        self.world = None
        self.tm = None
        self.ego = None # There is only one ego
        self.isHit = False
        self.initEvPos = lgsvl.Vector(1699, 88.38, -607.9)  # Initial position of ego vehicle
        self.npcList = [] # The list contains all the npc added
        self.initSimulator()
        self.loadMap()
        self.initEV()
        self.isEgoFault = False
        self.isHit = False
        self.connectEvToApollo()
        self.maxint = 130 
        self.egoFaultDeltaD = 0

    def initSimulator(self):
        client = carla.Client('localhost', 2000)
        client.start_recorder('recording.log')
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        self.sim = client
        self.world = client.get_world()
        self.tm = client.get_trafficmanager()
        self.tm.set_synchronous_mode(True)
        
        # sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181) 
        # self.sim = sim

    def loadMap(self, mapName="SanFrancisco"):
        sim = self.sim
        if self.world.get_map() == mapName:
           sim.reload_world()
        else:
           sim.load_world(mapName)
           
        # sim = self.sim
        # if sim.current_scene == mapName:
        #    sim.reset()
        # else:
        #    sim.load(mapName)

    def initEV(self):
        sim = self.sim
        world = self.world
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
        vehicle_bp.set_attribute('role_name', 'hero')
        spawn_points = world.get_map().get_spawn_points()
        spawn_ego = random.choice(spawn_points)
        ego = world.spawn_actor(vehicle_bp, spawn_ego)
        
        ego.set_autopilot(True)
        
        # egoState = lgsvl.AgentState()
        # egoState.transform = sim.map_point_on_lane(self.initEvPos)  # put the ego on the lane
        # ego = sim.add_agent("XE_Rigged-apollo_3_5", lgsvl.AgentType.EGO, egoState)  # connect ego to Apollo
        
        # self.camera_rgb = world.spawn_actor(world.get_blueprint_library().find('sensor.camera.rgb'), spawn_ego, attach_to=ego)
        self.detector_collision = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=ego)
        # self.detector_invasion = world.spawn_actor(world.get_blueprint_library().find('sensor.other.lane_invasion'), spawn_ego, attach_to=ego)
        # self.IMU = world.spawn_actor(world.get_blueprint_library().find('sensor.other.imu'), spawn_ego, attach_to=ego)
        # self.camera_rgb.enable_for_ros()
        # self.detector_collision.enable_for_ros()
        # self.detector_invasion.enable_for_ros()
        # self.IMU.enable_for_ros()
        
        def collision_callback(event):
            self.isHit = True

            # Check if the ego vehicle is involved in the collision
            other_actor = event.other_actor
            ego_velocity = self.ego.get_velocity()
            other_velocity = other_actor.get_velocity()

            # Calculate relative velocity and impact force
            relative_velocity = ego_velocity.length() - other_velocity.length()
            impact_force = event.normal_impulse.length()


            # Determine fault (simple heuristic based on relative velocity)
            if relative_velocity > 0:
                self.isEgoFault = True
            else:
                self.isEgoFault = False
        self.detector_collision.listen(collision_callback)
        
        # sensors = ego.get_sensors()
        # for s in sensors:
        #     if s.name in ['velodyne', 'Main Camera', 'Telephoto Camera', 'GPS', 'IMU']:
        #         s.enabled = True
        
        self.ego = ego

    def connectEvToApollo(self):
        ego = self.ego
        print("Connecting to bridge")
        ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)
        while not ego.bridge_connected:
            time.sleep(1)
        print("Bridge connected")

    def addNpcVehicle(self):
        sim = self.sim
        world = self.world
        npcList = self.npcList
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
        # random
        spawn_points = world.get_map().get_spawn_points()
        npc = world.spawn_actor(vehicle_bp, random.choice(spawn_points))
        npc.set_autopilot(True)
        destination = random.choice(spawn_points)
        if destination.location == npc.get_location():
            destination = random.choice(spawn_points)
        self.tm.set_path(npc, [destination.location])
        
        # # manual
        # Location = carla.Location(posVector.x, posVector.y, posVector.z)
        # npc = world.spawn_actor(vehicle_bp, carla.Transform(Location))
        
        # lgsvl
        # npcState = lgsvl.AgentState()
        # npcState.transform = sim.map_point_on_lane(posVector)  # put the npc on the lane
        # npc = sim.add_agent(vehicleType, lgsvl.AgentType.NPC, npcState)
        npcList.append(npc)

    def addFixedMovingNpc(self, posVector, vehicleType="SUV"):
        sim = self.sim
        npcState = lgsvl.AgentState()
        npcState.transform = sim.map_point_on_lane(posVector)
        npc = sim.add_agent(vehicleType, lgsvl.AgentType.NPC, npcState)
        npc.follow_closest_lane(True, 13.4)

    # This function send an instance action command to the NPC at the current time instance
    def setNpcSpeed(self, npc, speed):
        self.tm.set_desired_speed(npc, speed)

    # Direction is either "LEFT" or "RIGHT"
    def setNpcChangeLane(self, npc, direction):
        self.tm.force_lane_change(npc, direction)

    def setEvThrottle(self, throttle):
        ego = self.ego
        c = carla.VehicleControl()
        c.throttle = throttle
        ego.apply_control(c)

    def brakeDist(self, speed):                          # brake distance with respect to speed
        dBrake = 0.0467 * pow(speed, 2.0) + 0.4116 * speed - 1.9913 + 0.5
        if dBrake < 0:
            dBrake = 0
        return dBrake
    
    def findCollisionDeltaD(self, ego, npc):
        d = liability.findDistance(ego, npc) - 4.6 # 4.6 is the length of a car
        return d - self.brakeDist(ego.state.speed)

    def findDeltaD(self, ego, npc):
        d = liability.findDistance(ego, npc) - 4.6 # 4.6 is the length of a car
        deltaD = self.maxint # The smaller delta D, the better
        deltaDFront = self.maxint
        deltaDSide = self.maxint

        # When npc is in front
        if npc.state.transform.position.x + 4.6 < ego.state.transform.position.x and npc.state.transform.position.x + 20 > ego.state.transform.position.x:  # The ego is within a range of 4.6 to 20 units in front of the NPC.  may have mistake
            if npc.state.transform.position.z > ego.state.transform.position.z - 2 and npc.state.transform.position.z < ego.state.transform.position.z + 2:   # The NPC is within a range of 2 units to the left or right of the ego.
                deltaDFront = d - self.brakeDist(ego.state.speed)
                util.print_debug(" --- Delta D Front: " + str(deltaDFront))

        # When ego is changing line to npc's front
        if npc.state.transform.position.x - 4.6 > ego.state.transform.position.x and npc.state.transform.position.x - 20 < ego.state.transform.position.x:  # The NPC is within a range of 4.6 to 20 units in front of the egp. may have mistake
            if npc.state.transform.position.z + 2 > ego.state.transform.position.z and npc.state.transform.position.z - 2 < ego.state.transform.position.z and (ego.state.rotation.y < 269 or ego.state.rotation.y > 271):
                deltaDSide = d - self.brakeDist(npc.state.speed)
                util.print_debug(" --- Delta D Side: " + str(deltaDSide))
   
        deltaD = min(deltaDSide, deltaDFront)

        return deltaD

    def findFitness(self, deltaDlist, dList, isEgoFault, isHit, hitTime):
       # The higher the fitness, the better.

       minDeltaD = self.maxint
       for npc in deltaDlist: # ith NPC
            hitCounter = 0
            for deltaD in npc:
                if isHit == True and hitCounter == hitTime:
                   break
                if deltaD < minDeltaD:
                    minDeltaD = deltaD # Find the min deltaD over time slices for each NPC as the fitness
                hitCounter += 1
       util.print_debug(deltaDlist)
       util.print_debug(" *** minDeltaD is " + str(minDeltaD) + " *** ")

       minD = self.maxint
       for npc in dList: # ith NPC
            hitCounter = 0
            for d in npc:
                if isHit == True and hitCounter == hitTime:
                   break
                if d < minD:
                    minD = d
                hitCounter += 1
       util.print_debug(dList)
       util.print_debug(" *** minD is " + str(minD) + " *** ")

       fitness = 0.5 * minD + 0.5 * minDeltaD

       return fitness * -1

    def runSimulation(self, scenarioObj):

        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        util.print_debug("\n === Run simulation === [" + date_time + "]")
        sim = self.sim
        world = self.world
        tm = self.tm
        npcList = self.npcList
        ego = self.ego
        init_degree = ego.state.rotation.y
        numOfTimeSlice = len(scenarioObj[0])
        numOfNpc = len(scenarioObj)
        # deltaDList = [[self.maxint for i in range(numOfTimeSlice)] for j in range(numOfNpc)] # 1-D: NPC; 2-D: Time Slice
        # dList = [[self.maxint for i in range(numOfTimeSlice)] for j in range(numOfNpc)] # 1-D: NPC; 2-D: Time Slice
        # spawns = sim.get_spawn()

        # Add NPCs: Hard code for now, the number of npc need to be consistent.
        # Add NPCs: Hard code for now, the number of npc need to be consistent.
        ################################################################
        self.addNpcVehicle()
        self.addNpcVehicle()
        ################################################################

        # for npc in npcList:
        #     npc.follow_closest_lane(True, random.randint(1,9))
        
        self.isEgoFault = False
        self.isHit = False

        # def on_collision(agent1, agent2, contact):
        #     #util.print_debug(" --- On Collision, ego speed: " + str(agent1.state.speed) + ", NPC speed: " + str(agent2.state.speed))
        #     if self.isHit == True:
        #        return
        #     self.isHit = True
        #     if agent2 is None or agent1 is None:
        #         self.isEgoFault = True
        #         util.print_debug(" --- Hit road obstacle --- ")
        #         return

        #     apollo = agent1
        #     npcVehicle = agent2
        #     if agent2.name == "XE_Rigged-apollo_3_5":
        #         apollo = agent2
        #         npcVehicle = agent1
        #     util.print_debug(" --- On Collision, ego speed: " + str(apollo.state.speed) + ", NPC speed: " + str(npcVehicle.state.speed))
        #     if apollo.state.speed <= 0.005:
        #        self.isEgoFault = False
        #        return 
        #     self.isEgoFault = liability.isEgoFault(apollo, npcVehicle, sim, init_degree)
        #     # Compute deltaD when it is ego fault
        #     if self.isEgoFault == True:
        #         self.egoFaultDeltaD = self.findCollisionDeltaD(apollo, npcVehicle)
        #         util.print_debug(" >>>>>>> Ego fault delta D is " + str(self.egoFaultDeltaD))
                    
        # ego.on_collision(on_collision)

        # Frequency of action change of NPCs
        actionChangeFreq = 20
        hitTime = numOfNpc
        
        scenario_pos = [[[] for i in range(numOfTimeSlice)] for j in range(numOfNpc)] 
        
        for t in range(0, int(numOfTimeSlice)):
            # For every npc
            scenario_pos[0][t].append(ego.get_location())
            i = 1
            for npc in npcList:	
                self.setNpcSpeed(npc, scenarioObj[i][t][0])
                turnCommand = scenarioObj[i][t][1]
                #<0: no turn; 1: left; 2: right>
                if turnCommand == 1:
                    direction = False
                    self.tm.force_lane_change(npc, direction)
                elif turnCommand == 2:
                    direction = True
                    self.tm.force_lane_change(npc, direction)
                scenario_pos[i][t].append(npc.get_location())
                i += 1

            # Stop if there is accident
            if self.isEgoFault == True or liability.isHitEdge(ego, sim, init_degree):
               self.isHit = True
               self.isEgoFault = True
            if self.isHit == True:
               hitTime = t
               break

            for j in range(0, actionChangeFreq):

                # # Check if bridge is disconnected or if there is failure in log's last line
                # if self.isHit == True:
                #     time.sleep(10)
                # fbr = open(self.bridgeLogPath, 'r')
                # fbrLines = fbr.readlines()
                # for line in fbrLines:
                #     pass
            
                # while not ego.bridge_connected or "fail" in line or "Fail" in line or "overflow" in line:
                #     time.sleep(5)
                #     resultDic = {}
                #     resultDic['fitness'] = ''
                #     resultDic['fault'] = ''
                #     util.print_debug(" ---- Bridge is cut off ----")
                #     return resultDic

                world.tick()
                if self.isHit == True:
                    break
                
            if self.isHit == True:
                break


        resultDic = {}
        resultDic['pos'] = scenario_pos
        resultDic['fault'] = ''
        if self.isEgoFault == True:
                resultDic['fault'] = 'ego'
        util.print_debug(" === Finish simulation === ")
        util.print_debug(resultDic)

        return resultDic


##################################### MAIN ###################################
# Read scenario obj 
objPath = sys.argv[1]
resPath = sys.argv[2]

objF = open(objPath, 'rb')
scenarioObj = pickle.load(objF)
objF.close()

resultDic = {}
try:
    sim = LgApSimulation()
    resultDic = sim.runSimulation(scenarioObj)
except Exception as e:
    util.print_debug(e.message)
    resultDic['fitness'] = ''
    resultDic['fault'] = ''

# Send fitness score int object back to ge
if os.path.isfile(resPath) == True:
    os.system("rm " + resPath)
f_f = open(resPath, 'wb')
pickle.dump(resultDic, f_f)
f_f.truncate() 
f_f.close() 


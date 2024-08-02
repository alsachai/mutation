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

     
class LgApSimulation:

    def __init__(self, numOfTimeSlice, numOfNpc, ego_info=None, npc_spawn=None, is_init=None):
        ################################################################
        # self.bridgeLogPath = "/home/av-input/Workplace/apollo-lg/apollo-3.5/data/log/cyber_bridge.INFO"
        ################################################################
        self.sim = None 
        self.world = None
        self.tm = None
        self.ego = None # There is only one ego
        self.isHit = False
        self.npcList = [] # The list contains all the npc added
        self.numOfTimeSlice = numOfTimeSlice
        self.numOfNpc = numOfNpc
        self.is_init = is_init
        self.ego_pos = ego_info['ego_pos']
        self.junction_point = ego_info['junction_point']
        self.npc_spawn_list = npc_spawn['pos']
        self.other_npc = npc_spawn['other']
        self.scenario_pos = [[[] for i in range(numOfTimeSlice)] for j in range(numOfNpc + 1)] 
        self.resultDic = {}
        self.initSimulator()
        self.initEV()
        self.isEgoFault = False
        self.isHit = False
        # self.egoFaultDeltaD = 0

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
        spawn_ego = carla.Transform(carla.Location(x=self.ego_pos[0][0], y=self.ego_pos[0][1], z=self.ego_pos[0][2]), carla.Rotation(pitch=self.ego_pos[0][3], yaw=self.ego_pos[0][4], roll=self.ego_pos[0][5]))
        destination_ego = carla.Transform(carla.Location(x=self.ego_pos[1][0], y=self.ego_pos[1][1], z=self.ego_pos[1][2]))
        ego = world.spawn_actor(vehicle_bp, spawn_ego)
        ego.set_autopilot(True)
        self.tm.set_path(ego, [destination_ego.location])
        
        self.scenario_pos[0][0].append(spawn_ego.location.x)
        self.scenario_pos[0][0].append(spawn_ego.location.y)
        self.scenario_pos[0][0].append(spawn_ego.location.z)
        self.scenario_pos[0][0].append(spawn_ego.rotation.pitch)
        self.scenario_pos[0][0].append(spawn_ego.rotation.yaw)
        self.scenario_pos[0][0].append(spawn_ego.rotation.roll)
        
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
            util.print_debug("***** There is an accident! *****")

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

        self.spectator = world.get_spectator()
        self.spectator.set_transform(carla.Transform(spawn_ego.location + carla.Location(x=0, y=0, z=80), carla.Rotation(pitch=270)))


    def connectEvToApollo(self):
        ego = self.ego
        print("Connecting to bridge")
        ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)
        while not ego.bridge_connected:
            time.sleep(1)
        print("Bridge connected")

    def cal_intersect(self, p1_s, p1_g, p2_s, p2_g):
        dx1 = p1_g.x - p1_s.x
        dy1 = p1_g.y - p1_s.y
        dx2 = p2_g.x - p2_s.x
        dy2 = p2_g.y - p2_s.y
    
        d = dx1 * dy2 - dy1 * dx2
        if d == 0:
            if (p2_s.x - p1_s.x) * dy1 == (p2_s.y - p1_s.y) * dx1:
                t0 = ((p2_s.x - p1_s.x) * dx1 + (p2_s.y - p1_s.y) * dy1) / (dx1**2 + dy1**2)
                t1 = ((p2_g.x - p1_s.x) * dx1 + (p2_g.y - p1_s.y) * dy1) / (dx1**2 + dy1**2)
                if (0 <= t0 <= 1) or (0 <= t1 <= 1) or (t0 <= 0 and t1 >= 1) or (t1 <= 0 and t0 >= 1):
                    return True
            return False

        t2 = ((p2_s.x - p1_s.x) * dy2 - (p2_s.y - p1_s.y) * dx2) / d
        t3 = ((p2_s.x - p1_s.x) * dy1 - (p2_s.y - p1_s.y) * dx1) / d

        if 0 <= t2 <= 1 and 0 <= t3 <= 1:
            return True
    
        return False

    def addNpcVehicleJuntion(self, scenario_npc, npc_spawn, first_flag, num):
        sim = self.sim
        world = self.world
        npcList = self.npcList
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
        if first_flag == True:
            npc_pos = carla.Transform(carla.Location(x=npc_spawn[0], y=npc_spawn[1], z=npc_spawn[2]), carla.Rotation(pitch=npc_spawn[3], yaw=npc_spawn[4], roll=npc_spawn[5]))
            npc = world.spawn_actor(vehicle_bp, npc_pos)
            self.scenario_pos[num+1][0].append(npc_pos.location.x)
            self.scenario_pos[num+1][0].append(npc_pos.location.y)
            self.scenario_pos[num+1][0].append(npc_pos.location.z)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.pitch)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.yaw)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.roll)
        else:
            npc_pos = carla.Transform(carla.Location(x=scenario_npc[0][2], y=scenario_npc[0][3], z=scenario_npc[0][4]), carla.Rotation(pitch=scenario_npc[0][5], yaw=scenario_npc[0][6], roll=scenario_npc[0][7]))
            npc = world.spawn_actor(vehicle_bp, npc_pos)
            self.scenario_pos[num+1][0].append(npc_pos.location.x)
            self.scenario_pos[num+1][0].append(npc_pos.location.y)
            self.scenario_pos[num+1][0].append(npc_pos.location.z)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.pitch)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.yaw)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.roll)
            
        npc.set_autopilot(True)

        if first_flag == True:
            destination = random.choice(world.get_map().get_spawn_points())
            junction_point = self.junction_point[npc_spawn[6]]
            junction = carla.Location(x=junction_point[0], y=junction_point[1], z=junction_point[2])
            self.tm.set_path(npc, [junction, destination.location])
        else:
            route = []
            for t in range(1, len(scenario_npc)):
                route.append(carla.Location(x=scenario_npc[t][2], y=scenario_npc[t][3], z=scenario_npc[t][4]))
            self.tm.set_path(npc, route)
        npcList.append(npc)
        

    def addNpcVehicle(self, scenario_npc, npc_spawn, first_flag, num, ego_path):
        sim = self.sim
        world = self.world
        npcList = self.npcList
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
        # random
        spawn_points = world.get_map().get_spawn_points()
        if first_flag == True:
            npc_pos = carla.Transform(carla.Location(x=npc_spawn[0], y=npc_spawn[1], z=npc_spawn[2]), carla.Rotation(pitch=npc_spawn[3], yaw=npc_spawn[4], roll=npc_spawn[5]))
            npc = world.spawn_actor(vehicle_bp, npc_pos)
            self.scenario_pos[num+1][0].append(npc_pos.location.x)
            self.scenario_pos[num+1][0].append(npc_pos.location.y)
            self.scenario_pos[num+1][0].append(npc_pos.location.z)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.pitch)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.yaw)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.roll)
        else:
            npc_pos = carla.Transform(carla.Location(x=scenario_npc[0][2], y=scenario_npc[0][3], z=scenario_npc[0][4]), carla.Rotation(pitch=scenario_npc[0][5], yaw=scenario_npc[0][6], roll=scenario_npc[0][7]))
            npc = world.spawn_actor(vehicle_bp, npc_pos)
            self.scenario_pos[num+1][0].append(npc_pos.location.x)
            self.scenario_pos[num+1][0].append(npc_pos.location.y)
            self.scenario_pos[num+1][0].append(npc_pos.location.z)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.pitch)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.yaw)
            self.scenario_pos[num+1][0].append(npc_pos.rotation.roll)
            
        npc.set_autopilot(True)

        if first_flag == True:
            c_flag = False
            ego_s = carla.Location(x=ego_path[0][0], y=ego_path[0][1], z=ego_path[0][2])
            ego_g = carla.Location(x=ego_path[1][0], y=ego_path[1][1], z=ego_path[1][2])
            while(c_flag == False):
                destination = random.choice(spawn_points)
                if destination.location == npc.get_location():
                    continue
                c_flag = self.cal_intersect(ego_s, ego_g, npc.get_location(), destination.location)
                
            self.tm.set_path(npc, [destination.location])
        else:
            route = []
            for t in range(1, len(scenario_npc)):
                route.append(carla.Location(x=scenario_npc[t][2], y=scenario_npc[t][3], z=scenario_npc[t][4]))
            self.tm.set_path(npc, route)
            # self.tm.random_left_lanechange_percentage(npc, 0)
            # self.tm.random_right_lanechange_percentage(npc, 0)
            
        
        # # manual
        # Location = carla.Location(posVector.x, posVector.y, posVector.z)
        # npc = world.spawn_actor(vehicle_bp, carla.Transform(Location))
        
        # lgsvl
        # npcState = lgsvl.AgentState()
        # npcState.transform = sim.map_point_on_lane(posVector)  # put the npc on the lane
        # npc = sim.add_agent(vehicleType, lgsvl.AgentType.NPC, npcState)
        npcList.append(npc)

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



    def runSimulation(self, scenarioObj):
        sim = self.sim
        world = self.world
        tm = self.tm
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        util.print_debug("\n === Run simulation === [" + date_time + "]")
        record_path = "home/tieriv/alsachai/mutation/recording" + date_time + ".log"
        self.sim.start_recorder(record_path)
        npcList = self.npcList
        ego = self.ego
        # init_degree = ego.state.rotation.y

        # Add NPCs: Hard code for now, the number of npc need to be consistent.
        # Add NPCs: Hard code for now, the number of npc need to be consistent.
        ################################################################
        ego_path = self.ego_pos
        
        if len(self.other_npc) < self.numOfNpc:
            npc_first_spawn = random.sample(self.other_npc, len(self.other_npc))
        else:
            npc_first_spawn = random.sample(self.other_npc, self.numOfNpc)
        if len(self.npc_spawn_list) < self.numOfNpc:
            npc_junction = random.sample(self.npc_spawn_list, len(self.npc_spawn_list))
        else:
            npc_junction = random.sample(self.npc_spawn_list, self.numOfNpc)
        if self.is_init:
            used = 0
            for n in range(self.numOfNpc):
                p = random.random()
                if p <= 0.5:
                    if used < len(npc_junction):
                        self.addNpcVehicleJuntion(scenarioObj[n], npc_junction[used], self.is_init, n)
                        used += 1
                    else:
                        self.addNpcVehicle(scenarioObj[n], npc_first_spawn[n], self.is_init, n, ego_path)
                else:
                    self.addNpcVehicle(scenarioObj[n], npc_first_spawn[n], self.is_init, n, ego_path)
        else:
            for n in range(self.numOfNpc):
                self.addNpcVehicle(scenarioObj[n], npc_first_spawn[n], self.is_init, n, ego_path)
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
        hitTime = self.numOfNpc
        
        
        for t in range(0, int(self.numOfTimeSlice)):
            # For every npc
            
            if t != 0:
                self.scenario_pos[0][t].append(ego.get_location().x)
                self.scenario_pos[0][t].append(ego.get_location().y)
                self.scenario_pos[0][t].append(ego.get_location().z)
            i = 1

            for npc in npcList:	
                self.setNpcSpeed(npc, scenarioObj[i-1][t][0])
                turnCommand = scenarioObj[i-1][t][1]
                #<0: no turn; 1: left; 2: right>
                if turnCommand == 1:
                    direction = False
                    self.tm.force_lane_change(npc, direction)
                elif turnCommand == 2:
                    direction = True
                    self.tm.force_lane_change(npc, direction)
                if t != 0:
                    self.scenario_pos[i][t].append(npc.get_location().x)
                    self.scenario_pos[i][t].append(npc.get_location().y)
                    self.scenario_pos[i][t].append(npc.get_location().z)
                i += 1

            # # Stop if there is accident
            # if self.isEgoFault == True or liability.isHitEdge(ego, sim, init_degree):
            #    self.isHit = True
            #    self.isEgoFault = True
            # if self.isHit == True:
            #    hitTime = t
            #    break

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
                ego_position = self.ego.get_location()
                self.spectator.set_transform(carla.Transform(ego_position + carla.Location(x=0, y=0, z=80), carla.Rotation(pitch=270)))
                world.tick()
                if self.isHit == True:
                    break
                
            if self.isHit == True:
                break


        self.sim.stop_recorder()
        self.resultDic['pos'] = self.scenario_pos
        self.resultDic['fault'] = ''
        self.resultDic['time'] = date_time
        if self.isHit == True:
            if self.isEgoFault == True:
                self.resultDic['fault'] = 'ego'
            else:
                self.resultDic['fault'] = 'npc'
                if os.path.exists(record_path):
                    os.remove(record_path)
        else:
            self.resultDic['fault'] = None
            if os.path.exists(record_path):
                os.remove(record_path)
        util.print_debug(" === Finish simulation === ")
        
        util.print_debug("{} NPCs are waiting to be destroyed".format(len(npcList)))
        npc_count = 0
        for npc in npcList:
            while(npc.is_alive == True):
                npc.destroy()
            npc_count += 1
        util.print_debug("{} NPCs are destroyed".format(npc_count))
        while(self.ego.is_alive == True):
            self.ego.destroy()
            util.print_debug("destroy ego")
        while(self.detector_collision.is_alive == True):
            self.detector_collision.destroy()
            util.print_debug("destroy collision sensor")
        return self.resultDic


##################################### MAIN ###################################
# Read scenario obj 
objPath = sys.argv[1]
resPath = sys.argv[2]
egoPath = sys.argv[3]
npcPath = sys.argv[4]
numOfNpc = int(sys.argv[5])
numOfTimeSlice = int(sys.argv[6])
is_init = sys.argv[7]
if is_init == 'True':
    is_init = True
else:
    is_init = False

objF = open(objPath, 'rb')
scenarioObj = pickle.load(objF)
objF.close()

egoF = open(egoPath, 'rb')
ego_info = pickle.load(egoF)
egoF.close()

npcS = open(npcPath, 'rb')
npc_spawn = pickle.load(npcS)
npcS.close()

resultDic = {}
# try:
#     sim = LgApSimulation()
#     resultDic = sim.runSimulation(scenarioObj)
# except Exception as e:
#     util.print_debug(e.message)
#     resultDic['fitness'] = ''
#     resultDic['fault'] = ''

sim = LgApSimulation(numOfTimeSlice=numOfTimeSlice, numOfNpc=numOfNpc, ego_info=ego_info, npc_spawn=npc_spawn, is_init=is_init)
resultDic = sim.runSimulation(scenarioObj)

# Send fitness score int object back to ge
if os.path.isfile(resPath) == True:
    os.system("rm " + resPath)
f_f = open(resPath, 'wb')
pickle.dump(resultDic, f_f)
f_f.truncate() 
f_f.close() 


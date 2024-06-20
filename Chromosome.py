
import math
import random
import pprint
import pickle
import sys
import os
from datetime import datetime
import numpy as np
import util

class Chromosome:
    def __init__(self, bounds, NPC_size, time_size, conflict_t, conflict_d, period):
        self.y = 0
        self.scenario = [[[] for i in range(time_size)] for j in range(NPC_size)] # start with npc, scenario_pos starts with ego
        self.scenario_pos = None
        self.period_conflicts = None
        self.bounds = bounds
        self.code_x1_length = NPC_size 
        self.code_x2_length = time_size
        self.conflict_t = conflict_t
        self.conflict_d = conflict_d
        self.period = period
        self.timeoutTime = 300 # in seconds, timeout timer for simulator execution per each scenario simulation
        self.ego_path = None

    def fix_init(self):
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                v = (self.bounds[0][0] + self.bounds[0][1]) / float(2) #random.uniform(self.bounds[0][0], self.bounds[0][1])        # Init velocity
                a = 3  # Keep straight #random.randrange(self.bounds[1][0], self.bounds[1][1])      # Init action
                self.scenario[i][j].append(v)
                self.scenario[i][j].append(a)

    def rand_init(self):
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                v = random.uniform(self.bounds[0][0], self.bounds[0][1])        # Init velocity
                a = random.randrange(self.bounds[1][0], self.bounds[1][1])      # Init action
                self.scenario[i][j].append(v)
                self.scenario[i][j].append(a)

    def foo_obj_func(self):
        speedSum = 0
        for npc in self.scenario:
            for nt in npc:
                speedSum += nt[0]
                speedSum += nt[0] * 34
        return speedSum

    def decoding(self):
        fitness_score = 0

        # Send scenario object to simulation script
        s_f = open('scenario.obj', 'wb')
        pickle.dump(self.scenario, s_f)  # Pickle the scenario object
        s_f.truncate()
        s_f.close()       

        e_f = open('ego_path.obj', 'wb')
        pickle.dump(self.ego_path, e_f)  # Pickle the ego path object
        e_f.truncate()
        e_f.close()
        
        for x in range(0, 100):	

            if os.path.isfile('result.obj') == True:
                os.remove("result.obj")

            os.system("python3 simulation_a.py scenario.obj result.obj ego_path.obj {} {}".format(self.code_x1_length, self.code_x2_length))
            resultObj = None

            # Read fitness score
            if os.path.isfile('result.obj') == True:
                f_f = open('result.obj', 'rb')
                resultObj = pickle.load(f_f)
                f_f.close()

            if resultObj != None:
                return resultObj
                break
            else:
                util.print_debug(" ***** " + str(x) + "th/10 trial: Fail to get fitness, try again ***** ")

        return None

    # Get fitness score of the scenario
    def func(self, gen=None, lisFlag=False):
        resultObj = self.decoding()
        self.ego_path = resultObj['ego_pos']
        get_pos = resultObj['pos']
        for i in range(self.code_x1_length):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                self.scenario[i][j].append(get_pos[i+1][j][0])
                self.scenario[i][j].append(get_pos[i+1][j][1])
                self.scenario[i][j].append(get_pos[i+1][j][2])
        self.scenario_pos = [[[] for i in range(self.code_x2_length)] for j in range(self.code_x1_length+1)]
        for i in range(self.code_x1_length + 1):        # For every NPC
            for j in range(self.code_x2_length):    # For every time slice
                self.scenario_pos[i][j].append(get_pos[i][j][0])
                self.scenario_pos[i][j].append(get_pos[i][j][1])
                self.scenario_pos[i][j].append(get_pos[i][j][2])
        self.period_conflicts = self.findConflicts()
        self.y = sum(conflict['score'] for conflict in self.period_conflicts if conflict is not None)
        if resultObj['fault'] == 'ego':
                # An accident        
                util.print_debug(" ***** Found an accident where ego is at fault ***** ")
                # Dump the scenario where causes the accident
                if os.path.exists('AccidentScenario') == False:
                    os.mkdir('AccidentScenario')
                now = datetime.now()
                date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
                ckName = 'AccidentScenario/accident-gen' + str(gen) + '-' + date_time
                if lisFlag == True:
                    ckName = ckName + "-LIS"
                a_f = open(ckName, 'wb')
                pickle.dump(self, a_f)
                a_f.truncate() 
                a_f.close()
    

        
    def findConflicts(self):
        num_periods = (self.code_x2_length + self.period - 1) // self.period

        period_conflicts = []

        for period_idx in range(num_periods):
            start_time = period_idx * self.period
            end_time = min(start_time + self.period, self.code_x2_length)
        
            for t in range(start_time, end_time):
                ego_pos = self.scenario_pos[0][t]
                conflict_found = False
                # Check future positions
                for dt in range(0, self.conflict_t + 1):
                    if t + dt < end_time: 
                        future_pos_index = t + dt - start_time
                        future_positions = [self.scenario_pos[m][t + dt] for m in range(1, self.code_x1_length + 1)]
                        distances = np.linalg.norm(np.array(future_positions) - np.array(ego_pos), axis=1)
                        min_distance_idx = np.argmin(distances)

                        if distances[min_distance_idx] < self.conflict_d:
                            period_conflicts.append({
                                "ego_time": t,
                                "npc_time": t + dt,
                                "npc": min_distance_idx, 
                                "distance": distances[min_distance_idx],
                                "score": self.conflict_t - dt
                            })
                            conflict_found = True
                            break
                    # Check past positions
                    if t - dt >= start_time: 
                        past_pos_index = t - dt - start_time
                        past_positions = [self.scenario_pos[m][t - dt] for m in range(1, self.code_x1_length + 1)]
                        distances = np.linalg.norm(np.array(past_positions) - np.array(ego_pos), axis=1)
                        min_distance_idx = np.argmin(distances)

                        if distances[min_distance_idx] < self.conflict_d:
                            period_conflicts.append({
                                "ego_time": t,
                                "npc_time": t - dt,
                                "npc": min_distance_idx,
                                "distance": distances[min_distance_idx],
                                "score": self.conflict_t - dt
                            })
                            conflict_found = True
                            break

            if conflict_found:
                break
        if len(period_conflicts) == 0: 
            print("No conflict!!")

        return period_conflicts


# Test
if __name__ == '__main__':
    a = [[10, 30], [0, 2]]
    chromosome = Chromosome(a, 5, 10)
    pprint.pprint(chromosome.scenario)


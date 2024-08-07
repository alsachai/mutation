#! /usr/local/bin/python3
import os
import sys
import copy
import random
import numpy as np
import pickle
from Chromosome_b import Chromosome
from datetime import datetime
import util
import tools
import generateRestart
import carla

class GeneticAlgorithm:
    def __init__(self, bounds, pm, pc, pop_size, NPC_size, time_size, conflict_t, conflict_d, period, max_gen):
        self.bounds = bounds                # The value ranges of the inner most elements
        self.pm = pm
        self.pc = pc
        self.pop_size = pop_size            # Number of scenarios in the population
        self.NPC_size = NPC_size            # Number of NPC in each scenario
        self.time_size = time_size          # Number of time slides in each NPC
        self.conflict_t = conflict_t
        self.conflict_d = conflict_d
        self.period = period
        self.max_gen = max_gen
        self.pop = []
        self.bests = [0] * max_gen          # Best scenario in each generation
        self.bestIndex = 0
        self.g_best = None                  # Best scenario across all generations
        self.ck_path = None                 # Checkpoint path, if set, GE will start from the checkpoint (population object)
        self.touched_chs = []               # Record which chromosomes have been touched in each generation

        self.isInLis = False                # Set flag for local iterative search (LIS)
        self.minLisGen = 2                  # Min gen to start LIS
        self.numOfGenInLis = 5              # Number of gens in LIS
        self.hasRestarted = False
        self.lastRestartGen = 0
        self.bestYAfterRestart = 0
        self.count_dict = []
        
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        self.map = world.get_map()
        
        

    def set_checkpoint(self, ck_path):
        self.ck_path = ck_path

    def take_checkpoint(self, obj, ck_name):
        if os.path.exists('GaCheckpoints') == False:
            os.mkdir('GaCheckpoints')
        ck_f = open('GaCheckpoints/' + ck_name, 'wb')
        pickle.dump(obj, ck_f)
        ck_f.truncate() 
        ck_f.close()
   
    def setLisFlag(self):
        self.isInLis = True

    def setLisPop(self, singleChs):
        for i in range(self.pop_size):
            self.pop.append(copy.deepcopy(singleChs))

        # Add some entropy
        tempPm = self.pm
        self.pm = 1
        self.mutation_fuzz()
        self.pm = tempPm
        self.g_best, bestIndex = self.find_best()

    def ga(self):

        # Load from checkpoint if not none
        if self.ck_path != None:
            ck = open(self.ck_path, 'rb')
            self.pop = pickle.load(ck)
            ck.close()
        elif self.isInLis == False:
            self.init_pop()
        best, bestIndex = self.find_best()
        self.g_best = copy.deepcopy(best)

        # Start evolution
        for i in range(self.max_gen):                       # i th generation.

            util.print_debug(" \n\n*** " + str(i) + "th generation ***")

            # Make sure we clear touched_chs history book every gen
            self.touched_chs = []
            if self.isInLis == False:
                self.mutation_search()                                   # Mutation
            else:
                self.mutation_fuzz()                                   # Mutation
            #self.cross()                                       # Crossover
            self.simulate_touched(i)
            self.select_roulette()                             # Selection scenarios for the next generation

            best, bestIndex = self.find_best()                     # Find the scenario with the best fitness score in current generation 
            self.bests[i] = best                        # Record the scenario with the best fitness score in i th generation

            ########### Update noprogressCounter #########
            noprogress = False  # Flag to indicate whether there is no progress in the last 5 generations
            ave = 0  # Average of the best 5 scenarios
            if i >= self.lastRestartGen + 5:  
                for j in range(i - 5, i):
                    ave +=  self.bests[j].y
                ave /= 5
                if ave >= best.y:
                    self.lastRestarGen = i
                    noprogress = True

            util.print_debug(" ###### Best score of the generation: " + str(best.y) + " ###### ")
            if self.g_best.y < best.y:                  # Record the best fitness score across all generations
                self.g_best = copy.deepcopy(best)

            util.print_debug(" ###### Best score overall: " + str(self.g_best.y) + " ###### ")

            N_generation = self.pop
            N_b = self.g_best                           # Record the scenario with the best score over all generations

            # Update the checkpoint of the best scenario so far
            self.take_checkpoint(N_b, 'best_scenario.obj')                       

            # Checkpoint this generation
            self.take_checkpoint(N_generation, 'last_gen.obj')

            # Checkpoint every generation
            now = datetime.now()
            date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
            self.take_checkpoint(N_generation, 'generation-' + str(i) + '-at-' + date_time)

            #################### Start the Restart Process ################### 
            if noprogress == True and not self.isInLis:
                util.print_debug(" ###### Restart Based on Generation: " + str(i) + " ###### ")
                oldCkName = 'GaCheckpoints'
                newPop = generateRestart.generateRestart(oldCkName, 1000, self.bounds, self.conflict_d, self.conflict_t, self.period)
                self.pop = copy.deepcopy(newPop)
                self.hasRestarted = True
                best, self.bestIndex = self.find_best()
                self.bestYAfterRestart = best.y
                self.lastRestartGen = i
            #################### End the Restart Process ################### 

            if os.path.exists("GaCheckpoints") == True:
                prePopPool = generateRestart.getAllCheckpoints('GaCheckpoints') 
                simiSum = 0
                for eachChs in self.pop:
                    eachSimilarity =  generateRestart.getSimularityOfScenarioVsPrevPop(eachChs, prePopPool)
                    simiSum += eachSimilarity
                util.print_debug(" ==== Similarity compared with all prior generations: " + str(simiSum/float(self.pop_size)))

            # Log fitness etc
            f = open('Progress.log' ,'a')
            f.write(str(i) + " " + str(best.y) + " " + str(self.g_best.y) + " " + str(simiSum/float(self.pop_size)) + " " + date_time + "\n")
            f.close()

            if best.y > self.bestYAfterRestart:
                self.bestYAfterRestart = best.y
                if i > (self.lastRestartGen + self.minLisGen) and self.isInLis == False: # Only allow one level of recursion
                    ################## Start LIS #################
                    util.print_debug(" \n\n === Start of Local Iterative Search === \n\n")
                    # Increase mutation rate a little bit to jump out of local maxima
                    lis = GeneticAlgorithm(self.bounds, (self.pm * 1.5), self.pc, self.pop_size, self.NPC_size, self.time_size, self.conflict_t, self.conflict_d, self.period, self.numOfGenInLis)
                    lis.setLisPop(self.g_best)                
                    lis.setLisFlag()
                    lisBestChs = lis.ga()
                    util.print_debug(" --- Best fitness in LIS: " + str(lisBestChs.y))
                    if lisBestChs.y > self.g_best.y:
                        # Let's replace this
                        self.pop[bestIndex] = copy.deepcopy(lisBestChs)
                        util.print_debug(" --- Find better scenario in LIS: LIS->" + str(lisBestChs.y) + ", original->" + str(self.g_best.y))
                    else:
                        util.print_debug(" --- LIS does not find any better scenarios")
                    util.print_debug(" \n\n === End of Local Iterative Search === \n\n")
                    ################## End LIS ################ 
        if self.isInLis == False:
            s_f = open('count_dict.obj', 'wb')
            pickle.dump(self.count_dict, s_f)  # Pickle the scenario object
            s_f.truncate()
            s_f.close()  
            print(self.count_dict)
        return self.g_best

    def init_pop(self):
        i = 0
        while(i<self.pop_size):
            # A chromosome is a scenario
            chromosome = Chromosome(self.bounds, self.NPC_size, self.time_size, self.conflict_t, self.conflict_d, self.period)
            chromosome.rand_init()
            chromosome.func()
            if chromosome.is_accident == False and len(chromosome.potential_conflicts) != 0:
                print(" --- The scenario is not an accident, add to the population. ---")
                self.pop.append(chromosome)
                i += 1

    def cross(self):
        # Implementation of random crossover (exchange npc)

        for i in range(int(self.pop_size / 2.0)):
            # Check crossover probability
            if self.pc > random.random():
            # randomly select 2 chromosomes(scenarios) in pops
                i = 0
                j = 0
                while i == j:
                    i = random.randint(0, self.pop_size-1)
                    j = random.randint(0, self.pop_size-1)
                pop_i = self.pop[i]
                pop_j = self.pop[j]


                # select cross index
                swap_index = random.randint(0, pop_i.code_x1_length - 1)
                if pop_i.scenario_pos[swap_index][0] == pop_j.scenario[swap_index][0] and pop_i.scenario[swap_index][1] == pop_j.scenario[swap_index][1] and pop_i.scenario[swap_index][2] == pop_j.scenario[swap_index][2]:
                    continue
                
                # Record which chromosomes have been touched
                self.touched_chs.append(self.pop[i])
                self.touched_chs.append(self.pop[j])
                temp = copy.deepcopy(pop_j.scenario[swap_index])
                pop_j.scenario[swap_index] = copy.deepcopy(pop_i.scenario[swap_index])
                pop_i.scenario[swap_index] = temp

                
                temp = copy.deepcopy(pop_j.scenario_pos[swap_index])
                pop_j.scenario_pos[swap_index] = copy.deepcopy(pop_i.scenario_pos[swap_index])
                pop_i.scenario_pos[swap_index] = temp
                

    def isStraight(self, ego_pos, npc_pos, npc_pos_last, ego_time, npc_time):
        if ego_time > npc_time:
            ego_position = self.map.get_waypoint(carla.Location(x=ego_pos[0], y=ego_pos[1], z=ego_pos[2]))
            npc_position = self.map.get_waypoint(carla.Location(x=npc_pos[0], y=npc_pos[1], z=npc_pos[2]))
            if npc_pos_last is not None:
                npc_position_last = self.map.get_waypoint(carla.Location(x=npc_pos_last[0], y=npc_pos_last[1], z=npc_pos_last[2]))
                if npc_position_last.road_id == npc_position.road_id and npc_position_last.lane_id != ego_position.lane_id:
                    return False
            if ego_position.road_id == npc_position.road_id and ego_position.lane_id == npc_position.lane_id:
                return True
            else:
                return False
        else:
            return False
                
    def mutation_fuzz(self):
        i = 0
        while(i<len(self.pop)) :
            eachChs = self.pop[i]
            i += 1
            if len(eachChs.period_conflicts) == 0:
                continue
            if self.pm >= random.random():

                beforeMutation = copy.deepcopy(eachChs)
                # select mutation index (npc, time)
                index = random.randint(0, len(eachChs.period_conflicts)-1)
                ego_time = eachChs.period_conflicts[index]['ego_time']
                npc_index = eachChs.period_conflicts[index]['npc']
                npc_time = eachChs.period_conflicts[index]['npc_time']
                ego_pos = eachChs.scenario_pos[0][npc_time]            # check the position of ego when npc arrives the conflict point
                npc_pos = eachChs.scenario_pos[npc_index + 1][npc_time]
                if npc_time - 1 >= 0:
                    npc_pos_last = eachChs.scenario_pos[npc_index + 1][npc_time - 1]
                else:
                    npc_pos_last = None

                if self.isStraight(ego_pos, npc_pos, npc_pos_last, ego_time, npc_time):
                    if ego_time > npc_time:
                        prob = random.randint(0, 1)
                        if prob == 0:  # decelerate
                            count = 0
                            temp = 0
                            for t_s in range(npc_time - self.conflict_t if npc_time - self.conflict_t >= 0 else 0, npc_time):
                                if count == 0:
                                    temp = eachChs.scenario[npc_index][t_s][0]
                                    count += 1
                                v_s = random.uniform(self.bounds[3][0], self.bounds[3][1])
                                
                                if temp - v_s > 0:
                                    temp -= v_s
                                eachChs.scenario[npc_index][t_s][0] = temp
                        else:          # brake
                            count = 0
                            temp = 0
                            for t_s in range(npc_time - self.conflict_t if npc_time - self.conflict_t >= 0 else 0, npc_time):
                                if count == 0:
                                    temp = eachChs.scenario[npc_index][t_s][0]
                                    count += 1
                                v_s = random.uniform(self.bounds[4][0], self.bounds[4][1])
                                if temp - v_s > 0:
                                    temp -= v_s
                                eachChs.scenario[npc_index][t_s][0] = temp
                    
                else:
                    if ego_time > npc_time:
                        prob = random.randint(0, 2)
                        if prob == 0:       # decelerate
                            count = 0
                            temp = 0
                            for t_s in range(npc_time - self.conflict_t if npc_time - self.conflict_t >= 0 else 0, npc_time):
                                if count == 0:
                                    temp = eachChs.scenario[npc_index][t_s][0]
                                    count += 1
                                v_s = random.uniform(self.bounds[3][0], self.bounds[3][1])
                                if temp - v_s > 0:
                                    temp -= v_s
                                eachChs.scenario[npc_index][t_s][0] = temp
                        elif prob == 1:          # brake
                            count = 0
                            temp = 0
                            for t_s in range(npc_time, ego_time):
                                if count == 0:
                                    temp = eachChs.scenario[npc_index][t_s][0]
                                    count += 1
                                v_s = random.uniform(self.bounds[4][0], self.bounds[4][1])
                                if temp - v_s > 0:
                                    temp -= v_s
                                eachChs.scenario[npc_index][t_s][0] = temp
                        else:
                            count = 0
                            temp = 0
                            for t_s in range(npc_time - self.conflict_t if npc_time - self.conflict_t >= 0 else 0, npc_time):
                                if count == 0:
                                    temp = eachChs.scenario[npc_index][t_s][0]
                                    count += 1
                                v_s = random.uniform(self.bounds[3][0], self.bounds[3][1])
                                if temp - v_s > 0:
                                    temp -= v_s
                                eachChs.scenario[npc_index][t_s][0] = temp
                            count = 0
                            temp = 0
                            for t_s in range(npc_time, ego_time):
                                if count == 0:
                                    temp = eachChs.scenario[npc_index][t_s][0]
                                    count += 1
                                v_s = random.uniform(self.bounds[4][0], self.bounds[4][1])
                                if temp - v_s > 0:
                                    temp -= v_s
                                eachChs.scenario[npc_index][t_s][0] = temp
                    elif ego_time < npc_time:
                        count = 0
                        temp = 0
                        for t_s in range(ego_time - self.conflict_t if ego_time - self.conflict_t >= 0 else 0, ego_time): # accelerate
                            if count == 0:
                                temp = eachChs.scenario[npc_index][t_s][0]
                                count += 1
                            v_s = random.uniform(self.bounds[2][0], self.bounds[2][1])
                            temp += v_s
                            eachChs.scenario[npc_index][t_s][0] = temp
                    
                # Record which chromosomes have been touched
                self.touched_chs.append(eachChs)

        
    def mutation_search(self):
        i = 0
        while(i<len(self.pop)) :
            eachChs = self.pop[i]
            changed = False
            i += 1
            if len(eachChs.potential_conflicts) != 0:
                for conflict in eachChs.potential_conflicts:
                    if self.pm >= random.random():
                        ego_time = conflict['ego_time']
                        npc_index = conflict['npc']
                        npc_time = conflict['npc_time']
                        if npc_time > ego_time:
                            for dt in range(0, ego_time+1):
                                eachChs.scenario[npc_index][dt][0] += 0.5
                                changed = True
                        elif npc_index < ego_time:
                            for dt in range(0, ego_time+1):
                                eachChs.scenario[npc_index][dt][0] -= 0.5
                                changed = True
                        
                    
            for actor in range(0, eachChs.code_x1_length):
                if actor not in eachChs.saved_c_npcs and actor not in eachChs.saved_p_npcs:
                    if self.pm >= random.random():
                        changed = True
                        time_index = random.randint(0, eachChs.code_x2_length-1)
                        actionIndex = random.randint(0, 2)
                        if actionIndex == 0:
                            # Change Speed
                            eachChs.scenario[actor][time_index][0] = random.uniform(self.bounds[0][0], self.bounds[0][1])
                        elif actionIndex == 1:
                            # Change direction
                            eachChs.scenario[actor][time_index][1] = random.randrange(self.bounds[1][0], self.bounds[1][1])
                    
            # Record which chromosomes have been touched
            if changed == True:
                self.touched_chs.append(eachChs)


    def simulate_touched(self, gen):
        i = 0
        while(i < len(self.pop)):
            eachChs = self.pop[i]
            i += 1
            if eachChs in self.touched_chs:
                eachChs.func(gen, self.isInLis)
                if self.isInLis == False:
                    self.count_dict.append([len(eachChs.potential_conflicts), len(eachChs.period_conflicts)])
            else:
                util.print_debug(" --- The chromosome has not been touched in this generation, skip simulation. ---")
            util.print_debug(" --- In mutation: Current scenario has y = " + str(eachChs.y))
            # util.print_debug(eachChs.scenario)


    def select_top2(self):

        util.print_debug(" +++ Before select() +++ ")
        for i in range(0, self.pop_size):
            util.print_debug(" === Fitness result of the scenario is " + str(self.pop[i].y) + " === ")
    
        maxFitness = 0
        v = []
        for i in range(0, self.pop_size):
            if self.pop[i].y > maxFitness:
                maxFitness = self.pop[i].y

        for i in range(0, self.pop_size):
            if self.pop[i].y == maxFitness:
                for j in range(int(self.pop_size / 2.0)):
                    selectedChromosome = Chromosome(self.bounds, self.NPC_size, self.time_size, self.conflict_t, self.conflict_d, self.period)
                    selectedChromosome.scenario = self.pop[i].scenario
                    selectedChromosome.y = self.pop[i].y
                    v.append(selectedChromosome)
                break

        max2Fitness = 0
        for i in range(0, self.pop_size):
            if self.pop[i].y > max2Fitness and self.pop[i].y != maxFitness:
                max2Fitness = self.pop[i].y

        for i in range(0, self.pop_size):
            if self.pop[i].y == max2Fitness:
                for j in range(int(self.pop_size / 2.0)):
                    selectedChromosome = Chromosome(self.bounds, self.NPC_size, self.time_size, self.conflict_t, self.conflict_d, self.period)
                    selectedChromosome.scenario = self.pop[i].scenario
                    selectedChromosome.y = self.pop[i].y
                    v.append(selectedChromosome)
                break

        self.pop = copy.deepcopy(v)
        util.print_debug(" +++ After select() +++ ")
        for i in range(0, self.pop_size):
            util.print_debug(" === Fitness result of the scenario is " + str(self.pop[i].y) + " === ")


    def select_roulette(self):   # generate new population by roulette selection

        sum_f = 0

        util.print_debug(" +++ Before select() +++ ")
        for i in range(0, self.pop_size):  
            if self.pop[i].y == 0:  # If the fitness score is 0, set it to a small value  y:fitness score
                self.pop[i].y = 0.001
            util.print_debug(" === Fitness result of the scenario is " + str(self.pop[i].y) + " === ")

        ############################################################
        min = self.pop[0].y
        for k in range(0, self.pop_size):
            if self.pop[k].y < min:
                min = self.pop[k].y
        if min < 0: # If the minimum fitness score is less than 0, add the absolute value of the minimum fitness score to all fitness scores
            for l in range(0, self.pop_size):
                self.pop[l].y = self.pop[l].y + (-1) * min 

        # roulette
        for i in range(0, self.pop_size):
            sum_f += self.pop[i].y
        p = [0] * self.pop_size              # initialize probability
        for i in range(0, self.pop_size):
            if sum_f == 0:
                sum_f = 1
            p[i] = self.pop[i].y / sum_f     # calculate probability (precentage of the fitness score of the scenario in the total fitness score of the population)
        q = [0] * self.pop_size              # initialize cumulative probability
        q[0] = 0
        for i in range(0, self.pop_size):
            s = 0
            for j in range(0, i+1):
                s += p[j]                    
            q[i] = s                         # q[i] is the cumulative probability of the first i scenarios

        # start roulette
        v = []                               # new population
        for i in range(0, self.pop_size):
            r = random.random()              # generate a random number between 0 and 1
            if r < q[0]:
                selectedChromosome = Chromosome(self.bounds, self.NPC_size, self.time_size, self.conflict_t, self.conflict_d, self.period)
                selectedChromosome.scenario = self.pop[0].scenario
                selectedChromosome.y = self.pop[0].y
                selectedChromosome.is_accident = False
                selectedChromosome.scenario_pos = self.pop[0].scenario_pos
                selectedChromosome.period_conflicts = self.pop[0].period_conflicts
                selectedChromosome.saved_c_npcs = self.pop[0].saved_c_npcs
                selectedChromosome.potential_conflicts = self.pop[0].potential_conflicts
                selectedChromosome.saved_p_npcs = self.pop[0].saved_p_npcs
                v.append(selectedChromosome)
            for j in range(1, self.pop_size):
                if q[j - 1] < r <= q[j]:
                    selectedChromosome = Chromosome(self.bounds, self.NPC_size, self.time_size, self.conflict_t, self.conflict_d, self.period)
                    selectedChromosome.scenario = self.pop[j].scenario
                    selectedChromosome.y = self.pop[j].y
                    selectedChromosome.is_accident = False
                    selectedChromosome.scenario_pos = self.pop[j].scenario_pos
                    selectedChromosome.period_conflicts = self.pop[j].period_conflicts
                    selectedChromosome.saved_c_npcs = self.pop[j].saved_c_npcs
                    selectedChromosome.potential_conflicts = self.pop[j].potential_conflicts
                    selectedChromosome.saved_p_npcs = self.pop[j].saved_p_npcs
                    v.append(selectedChromosome)
        self.pop = copy.deepcopy(v)
        ############################################################

        util.print_debug(" +++ After select() +++ ")
        for i in range(0, self.pop_size):
            util.print_debug(" === Fitness result of the scenario is " + str(self.pop[i].y) + " === ")

    def find_best(self):
        best = copy.deepcopy(self.pop[0])
        bestIndex = 0
        for i in range(self.pop_size):
            if best.y < self.pop[i].y:
                best = copy.deepcopy(self.pop[i])
                bestIndex = i
        return best, bestIndex
        
    def __inverse(self, i):
        r = '1'
        if i == '1':	
            r = '0'

        return r

if __name__ == '__main__':
    bounds = [[0, 15], [0, 3], [0, 2], [0, 1], [2, 3]]
    algorithm = GeneticAlgorithm(bounds,0.4, 0.8, 4, 20, 30, 3, 3, 3, 100)
    algorithm.ga()
    pass

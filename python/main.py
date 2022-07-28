from common import *
from settings import *
import greedy
import random_assign
#import genetic
import genetici
from mip import ip_solve

def to_map(assignment):
    type_bin=[[] for i in range(types)]
    for b,c in enumerate(assignment):
        type_bin[c].append(bin_positions[b])
    return type_bin


if __name__=="__main__":
    t0=time.time()
    assignment_random=random_assign.random_assignment(prob)
    # assignment_greedy=greedy.greedy_allocating(prob)
    # 
    # genetic_algorithm =genetici.GeneticAlgorithm(iterations=800, population_size=100,elites_num=20, mutation_rate=0.008,  roulette_selection=False, plot_progress=False)
    # genetic_algorithm.run()
    # assignment_genetic=genetic_algorithm.best_chromosome()
    # print(evaluateAssignment(assignment_greedy),evaluateAssignment(assignment_random),evaluateAssignment(assignment_genetic))
    # #print(assignment_greedy)
    # print(to_map(assignment_greedy))
    # save_assignment_as_json("./greedy.json",to_map(assignment_greedy),station_position,xmax,ymax,prob)
    # save_assignment_as_json("./random.json",to_map(assignment_random),station_position,xmax,ymax,prob)
    # save_assignment_as_json("./genetic.json",to_map(assignment_genetic),station_position,xmax,ymax,prob)
    # assignment_ip=ip_solve()
    
    # assignment_hungarian=greedy.hungarian(prob)
    t1=time.time()
    print("soc=",random_assign.evaluateAssignment(assignment_random))
    print("runtime=",t1-t0)
    # greedyi.min_cost_matching()
    # save_assignment_as_json("./huge_genetic.json",to_map(assignment_genetic),station_position,xmax,ymax,prob)
    
    
    pass
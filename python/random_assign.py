from common import *
from settings import *


def evaluateAssignment(assignment):
    type_bin=dict()
    for i in range(num_bins):
        if assignment[i] not in type_bin:
            type_bin[assignment[i]]=[]
        type_bin[assignment[i]].append(i)
    min_dist=np.zeros([num_station,types])
    for i in range(num_station):
        for j in range(types):
            min_dist[i][j]=min([distance(station_position[i],bin_positions[k]) for k in type_bin[j]])
    soc=0
    for i in range(num_station):
        # print(len(prob[0]),types)
        for c,nc in enumerate(prob[i]):
            
            soc=soc+nc*min_dist[i][c]
    return soc/num_station


def random_assignment(stations):
    bin_array=[i for i in range(num_bins)]
    np.random.shuffle(bin_array)
    type_bin_map=dict()
    bin_type_map=dict()
    for i in range(types):
        type_bin_map[i]=[bin_array[i]]
        bin_type_map[bin_array[i]]=i
    for j in range(types,num_bins):
        random_type=np.random.randint(0,types)
        type_bin_map[random_type]=bin_array[j]
        bin_type_map[bin_array[j]]=random_type
    assignment=[bin_type_map[i] for i in range(num_bins)]
    return assignment

if __name__=="__main__":
    stations=[]
    
    # print(order)
    for i in range(num_station):
        order=generate_order(100,np.random.randint(1,5,types))
        stations.append(order)
    print(stations)
    result=random_assignment(stations)
    type_bin=[[] for i in range(types)]
    for b,c in enumerate(result):
        type_bin[c].append(bin_positions[b])
    save_assignment_as_json("./random.json",type_bin,station_position,xmax,ymax,prob)
    print(result)
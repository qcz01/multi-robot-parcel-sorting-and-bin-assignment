from common import *

from settings import *


def min_cost_matching():
    """_summary_
    
    calculate the min-cost matching
    """
    cost=np.zeros([types,num_bins])
    for c in range(types):
        for b in range(num_bins):
            wbc=0
            for k in range(num_station):
                wbc=wbc+prob[k][c]*distance(bin_positions[b],station_position[k])
            cost[c][b]=wbc
  # print(cost)
    row_ind,col_ind=linear_sum_assignment(cost)
    final_cost=cost[row_ind, col_ind].sum()
    cost_bins=[cost[c][b] for c,b in zip(row_ind,col_ind)]
    return row_ind,col_ind,cost_bins,final_cost




def evaluateCost(bins,c):
    """_summary_

    Args:
        bins (_type_): _description_
        c (_type_): _description_

    Returns:
        _type_: _description_
    """
    cost_map=dict()
    soc=0
    for s in range(num_station):
        ws=99999999999
        min_b=0
        for b in bins:
            if b not in cost_map:
                cost_map[b]=0
        wbs=prob[s][c]*distance(station_position[s],bin_positions[b])
        if ws>wbs:
            ws=wbs
            min_b=b
    cost_map[min_b]=cost_map[min_b]+ws
    soc=soc+ws
    return soc,cost_map


def greedy_allocating():
    row_ind, col_ind, cost_bins, soc = min_cost_matching()
    print(soc)
    allocation_map=dict()
    bin_cost_map=dict()
    type_bin=dict()
    for c,b in zip(row_ind,col_ind):
        allocation_map[b]=c
        bin_cost_map[b]=cost_bins[c]
        if c not in type_bin:
            type_bin[c]=[]
        type_bin[c].append(b)
    unallocated_bins=[b for b in range(num_bins) if b not in col_ind]
    while len(unallocated_bins)!=0:
        b,max_cost=max(bin_cost_map.items(), key=lambda k: k[1])
        # print(b)
        c=allocation_map[b]
        old_soc=bin_cost_map[b]
        min_soc=9999999999999999999
        new_cost_map=None
        for ub in unallocated_bins:
            cbins=type_bin[c]
            cbins.append(ub)
            new_soc,cost_map=evaluateCost(cbins,c)
            if new_soc<min_soc:
                min_b=ub
                min_soc=new_soc
                new_cost_map=cost_map
      
        unallocated_bins.remove(min_b)
        type_bin[c].append(min_b)
        bin_cost_map.update(new_cost_map)
        allocation_map[min_b]=c
        if min_soc>old_soc:
            print(allocation_map)
            print("duplicated bins")
        while len(unallocated_bins)!=0:
            bin=unallocated_bins[0]
            tb=np.random.randint(0,types)
            type_bin[tb].append(bin)
            allocation_map[bin]=tb
            unallocated_bins.pop(0)

        break
    soc=soc-old_soc+min_soc
  
   
        # print(soc)
    result=[allocation_map[b] for b in range(num_bins)]
    return result

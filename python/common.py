import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import bisect
import random
from scipy.optimize import linear_sum_assignment
import time
from settings import *
from typing import List
import os
import errno

def generate_order(num_parcels,prob):
    k=0
    cumulative_prob=[]
    total=sum(prob)
    soc=0
    orders=[0 for i in range(types)]
    for p in prob:
        soc=soc+p/total
        cumulative_prob.append(soc)
    while k<num_parcels:
        pk=np.random.random()
        i=bisect.bisect(cumulative_prob,pk)
        orders[i]=orders[i]+1
        k=k+1
    return orders


def distance(pa,pb):
    """_summary_

    Args:
        pa (tuple): position a
        pb (tuple): position b

    Returns:
        _type_: _distance
    """
    return abs(pa[0]-pb[0])+abs(pa[1]-pb[1])


def evaluateAssignment(assignment:List[int]):
    """_summary_

    Args:
        assignment (List[int]): _description_

    Returns:
        int: _description_
    """
    
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
      for c,nc in enumerate(prob[i]):
        soc=soc+nc*min_dist[i][c]
    return soc

def save_assignment_as_txt(file_name:str,type_bin:List):
    """_summary_

    Args:
        file_name (str): _description_
        type_bin (List): _description_
    """
    with open(file_name, "w") as file_content:
        file_content.write("num_types="+str(len(type_bin)))
        for i in range(len(type_bin)):
            file_content.write("i:")
            for b in type_bin[i]:
                file_content.write(str(bin_positions[i][0])+" "+str(bin_positions[i][1]))
                file_content.write(",");
            file_content.write("\n")
from isort import file
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
import json

def generate_order(num_parcels,prob):
    """_summary_

    Args:
        num_parcels (_type_): _description_
        prob (_type_): _description_

    Returns:
        _type_: _description_
    """
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

def save_assignment_as_json(file_name:str,type_bin:List,station_positions:List,xmax:int,ymax:int,prob:List[List]):
    """_summary_

    Args:
        file_name (str): _description_
        type_bin (List): _description_
        station_positions (List): _description_
        xmax (int): _description_
        ymax (int): _description_
        prob (List[List]): _description_
    """
    data_dict=dict()
    data_dict["num_types"]=len(type_bin)
    data_dict["xmax"]=xmax
    data_dict["ymax"]=ymax
    data_dict["num_stations"]=len(station_positions)
    data_dict["station_positions"]=[]
    for sp in station_positions:
        data_dict["station_positions"].append([sp[0],sp[1]])
    assignment=dict()
    for i in range(len(type_bin)):
        assignment[i]=[]
        for b in type_bin[i]:
       
            assignment[i].append([b[0],b[1]])
    data_dict["assignment"]=assignment
    data_dict["probability"]=prob
    with open(file_name,"w") as fp:
        json.dump(data_dict,fp)


if __name__=="__main__":
    type_bin=[[(1,2),(3,4)],[(5,6)]]
    station_positions=[(1,1),(2,3),(1,2)]
    xmax=4
    ymax=5
    save_assignment_as_json("./test.json",type_bin,station_positions,xmax,ymax)
    

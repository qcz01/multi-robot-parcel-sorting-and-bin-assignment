import numpy as np
import json
# small settings
def normalize(prob):
    norm_prob=[]
    for p in prob:
        sum_p=sum(p)
        p=[x/sum_p for x in p]
        norm_prob.append(p)
    return norm_prob


xmax=29
ymax=14
station_position=[(3,0),(7,0),(11,0),(15,0),(19,0),(23,0),(3,13),(7,13),(11,13),(15,13),(19,13),(23,13)]
num_station=len(station_position)
orders=[]
bin_positions=[(x,y) for x in range(2,xmax,3) for y in range(2,ymax,3)]
num_bins=len(bin_positions)
types=36
file_name="./settings/type_36.txt"
try:
    prob=(np.loadtxt(file_name)).tolist()
except:
    prob=np.random.randint(types,size=(num_station,types)).tolist()
    np.savetxt(file_name,prob,fmt='%i')
prob=normalize(prob)
# prob=[[7, 7, 5, 6, 5, 3, 5, 3, 6, 2, 8, 4, 12, 3, 6, 9, 4, 2, 3, 0], [1, 8, 8, 2, 2, 2, 2, 3, 5, 10, 14, 2, 3, 5, 4, 5, 5, 10, 6, 3], [8, 1, 12, 8, 0, 4, 4, 3, 3, 2, 5, 1, 7, 4, 13, 10, 0, 8, 2, 5], [6, 8, 5, 11, 4, 8, 3, 7, 3, 5, 0, 5, 2, 9, 2, 7, 1, 5, 5, 4], [4, 6, 2, 5, 8, 5, 5, 3, 3, 1, 11, 11, 10, 2, 2, 8, 4, 0, 2, 8], [2, 6, 4, 3, 5, 3, 2, 10, 6, 7, 7, 7, 2, 4, 5, 4, 5, 7, 5, 6], [5, 5, 5, 1, 1, 10, 4, 7, 5, 3, 3, 6, 4, 13, 5, 7, 3, 7, 5, 1], [5, 5, 3, 10, 3, 8, 6, 6, 3, 3, 3, 1, 5, 5, 4, 7, 5, 10, 4, 4], [2, 11, 10, 9, 3, 2, 3, 4, 2, 3, 4, 8, 4, 6, 5, 0, 6, 9, 2, 7], [3, 2, 4, 1, 1, 2, 8, 7, 5, 10, 9, 12, 5, 4, 5, 3, 4, 7, 2, 6], [1, 2, 4, 5, 1, 6, 8, 9, 3, 5, 14, 3, 11, 2, 1, 3, 5, 0, 10, 7], [1, 8, 11, 8, 6, 7, 3, 5, 1, 7, 7, 1, 3, 3, 8, 9, 2, 5, 1, 4]]



# large settings
# xmax=92
# ymax=46
# station_position=[(x,0) for x in range(3,xmax-1,6)]+[(x,ymax-1) for x in range(3,xmax-1,6)]

# num_station=len(station_position)
# print("num_station",num_station)
# orders=[]
# bin_positions=[(x,y) for x in range(2,xmax,3) for y in range(2,ymax,3)]
# num_bins=len(bin_positions)
# print(num_bins)
# types=200

# file_name="./settings/huge_type_200.txt"
# try:
#     prob=(np.loadtxt(file_name)).tolist()
# except:
#     prob=np.random.randint(types,size=(num_station,types)).tolist()
#     np.savetxt(file_name,prob,fmt='%i')
# prob=normalize(prob)

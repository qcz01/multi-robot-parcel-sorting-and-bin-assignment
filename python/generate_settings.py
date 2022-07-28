import json
import io

data_dict=dict()




xmax=29
ymax=14
station_position=[(3,0),(7,0),(11,0),(15,0),(19,0),(23,0),(3,13),(7,13),(11,13),(15,13),(19,13),(23,13)]
num_station=len(station_position)
orders=[]
bin_positions=[(x,y) for x in range(2,xmax,3) for y in range(2,ymax,3)]
num_bins=len(bin_positions)
types=20
# prob=[[7, 7, 5, 6, 5, 3, 5, 3, 6, 2, 8, 4, 12, 3, 6, 9, 4, 2, 3, 0], [1, 8, 8, 2, 2, 2, 2, 3, 5, 10, 14, 2, 3, 5, 4, 5, 5, 10, 6, 3], [8, 1, 12, 8, 0, 4, 4, 3, 3, 2, 5, 1, 7, 4, 13, 10, 0, 8, 2, 5], [6, 8, 5, 11, 4, 8, 3, 7, 3, 5, 0, 5, 2, 9, 2, 7, 1, 5, 5, 4], [4, 6, 2, 5, 8, 5, 5, 3, 3, 1, 11, 11, 10, 2, 2, 8, 4, 0, 2, 8], [2, 6, 4, 3, 5, 3, 2, 10, 6, 7, 7, 7, 2, 4, 5, 4, 5, 7, 5, 6], [5, 5, 5, 1, 1, 10, 4, 7, 5, 3, 3, 6, 4, 13, 5, 7, 3, 7, 5, 1], [5, 5, 3, 10, 3, 8, 6, 6, 3, 3, 3, 1, 5, 5, 4, 7, 5, 10, 4, 4], [2, 11, 10, 9, 3, 2, 3, 4, 2, 3, 4, 8, 4, 6, 5, 0, 6, 9, 2, 7], [3, 2, 4, 1, 1, 2, 8, 7, 5, 10, 9, 12, 5, 4, 5, 3, 4, 7, 2, 6], [1, 2, 4, 5, 1, 6, 8, 9, 3, 5, 14, 3, 11, 2, 1, 3, 5, 0, 10, 7], [1, 8, 11, 8, 6, 7, 3, 5, 1, 7, 7, 1, 3, 3, 8, 9, 2, 5, 1, 4]]
prob=np.random.randint(types,size=(num_station,types))



data_dict["xmax"]=xmax
data_dict["ymax"]=14
data_dict["station_position"]=station_position
data_dict["num_station"]=num_station
data_dict["num_bins"]=num_bins
data_dict["num_types"]=types
data_dict["prob"]=prob


file_name="small_warehouse.json"
with open(file_name,"w") as fp:
    json.dump(data_dict,fp)
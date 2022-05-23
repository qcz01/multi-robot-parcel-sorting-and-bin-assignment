station_position=[(4,0),(8,0),(12,0),(16,0),(20,0),(24,0),(4,15),(8,15),(12,15),(16,15),(20,15),(24,15)]
num_station=len(station_position)
orders=[]
bin_positions=[(x,y) for x in range(3,28,3) for y in range(3,13,3)]
num_bins=len(bin_positions)
types=30
prob=[[]]
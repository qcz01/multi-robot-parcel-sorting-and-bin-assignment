import numpy as np
import matplotlib.pyplot as plt


LINE_WIDTH=3
MARKER_SIZE=10
CAPSIZE=3
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
plt.rcParams['font.serif'] = "Times"
plt.rcParams.update(
        {
            'xtick.labelsize': 22,
            'ytick.labelsize': 22,
            "legend.borderpad": 0.2,  ## border whitespace
            "legend.labelspacing": 0.2,  ## the vertical space between the legend entries
            "legend.handlelength": 1,  ## the length of the legend lines
            "legend.handleheight": 0.7,  ## the height of the legend handle
            "legend.handletextpad": 0.2,  ## the space between the legend line and legend text
            "legend.borderaxespad": 0.5,  ## the border between the axes and legend edge
            "legend.columnspacing": 1.0,  ## column separation
            "legend.framealpha": 0.5
        }
    )
font1 = {'family' : 'Serif',
'weight' : 'normal',
'size'   : 22,
}


PURPLE=(102/255.,0,255/255)
BLUE=(0,193/255,232/255)
GREEN=(0,176/255,80/255)
ORANGE=(255/255,192/255,0)
PINK=(255/255,102/255,153/255)
RED=(0.9,0,0)


num_types=[10,20,30,36]

#average_dist
avg_dist_genetic=[6.415,9.352,11.843,0]
avg_dist_greedy=[7.9762,11.119,12.6167,0]
avg_dist_random=[9.3045,12.016,13.9809,15.1055]
avg_dist_hungarian=[0,0,0,13.074]
avg_dist_mip=[6.4,9.21,11.8,0]

#comp_time
comp_time_genetic=[14.219,19.962,25.115,0]# set width of bar
comp_time_greedy=[3e-3,4e-3,5e-3,0]
comp_time_random=[1e-4,1e-4,1e-4,2.5e-5]
comp_time_hungarian=[0,0,0,2e-3]
comp_time_mip=[300,300,300,0]
barWidth = 0.13
fig = plt.subplots(figsize =(16, 5))
 
# set height of bar
#IT = [12, 30, 1, 8, 22]
#ECE = [28, 6, 16, 5, 10]
#CSE = [29, 3, 24, 25, 17]

 
# Set position of bar on X axis
br1 = np.arange(len(num_types))
br2 = [x + barWidth for x in br1]
br3 = [x + barWidth for x in br2]
br4 = [x + barWidth for x in br3]
br5 = [x + barWidth for x in br4]

plt.subplot(1,2,1)
# Make the plot
plt.bar(br2, avg_dist_random, color =RED, width = barWidth,
        edgecolor ='grey', label ='Random')
plt.bar(br3, avg_dist_greedy, color =GREEN, width = barWidth,
        edgecolor ='grey', label ='Greedy')
plt.bar(br4, avg_dist_genetic, color =BLUE, width = barWidth,
        edgecolor ='grey', label ='GA')
plt.bar(br5, avg_dist_mip, color =ORANGE, width = barWidth,
        edgecolor ='grey', label ='MIP')

plt.bar(br1, avg_dist_hungarian, color =PURPLE, width = barWidth,
        edgecolor ='grey', label ='Hungarian') 
 
# Adding Xticks
plt.xlabel('Number of types',fontsize = 28)
plt.ylabel('Average  distance',  fontsize = 28)
plt.legend(prop=font1,bbox_to_anchor=(0.35,1.15), loc="upper left", borderaxespad=0,ncol=5)
plt.xticks([r + 2*barWidth for r in range(len(num_types))],
        ['10', '20', '30', '36'])
 

plt.subplot(1,2,2)
plt.bar(br2, comp_time_random, color =RED, width = barWidth,
        edgecolor ='grey', label ='Random')
plt.bar(br3, comp_time_greedy, color =GREEN, width = barWidth,
        edgecolor ='grey', label ='Greedy')
plt.bar(br4, comp_time_genetic, color =BLUE, width = barWidth,
        edgecolor ='grey', label ='GA')
plt.bar(br5, comp_time_mip, color =ORANGE, width = barWidth,
        edgecolor ='grey', label ='MIP')

plt.bar(br1, comp_time_hungarian, color =PURPLE, width = barWidth,
        edgecolor ='grey', label ='Hungarian') 


        
plt.xlabel('Number of types', fontsize = 28)
plt.ylabel('Computation time (s)',fontsize=28)
plt.yscale('log')
plt.xticks([r + 2*barWidth for r in range(len(num_types))],
        ['10', '20', '30', '36'])
plt.savefig("bin_assignment.pdf",pad_inches=0.05,bbox_inches="tight",)
plt.show()

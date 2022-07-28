from unittest import findTestCases

from torch import double
from common import *
from settings import *
import matplotlib.pyplot as plt
import greedy 

cdef class Fitness:
    cdef list assignment
    cdef double fitness
    def __init__(self,assignment):
        self.assignment=assignment
        self.fitness=0.0


cdef class GeneticAlgorithm:
    cdef int iterations,population_size,elites_num
    cdef double mutation_rate
    cdef bint roulette_selection
    cdef bint plot_progress
    cdef double average_cost
    cdef int greed_seed
    cdef double average_path_cost
    cdef list ranked_population,progress,population
    
    def __init__(self,iterations,population_size,elites_num,mutation_rate,roulette_selection=False,plot_progress=True):
        self.plot_progress=plot_progress
        self.greed_seed=0
        self.roulette_selection=roulette_selection
        self.progress=[]
        self.mutation_rate=mutation_rate
        self.elites_num=elites_num
        self.iterations=iterations
        self.population_size=population_size
        self.population=self.initial_population()
        self.average_cost=1;
        self.ranked_population=None
   


    def best_chromosome(self):
        return self.ranked_population[0][0]

    def best_cost(self):
        return 1/self.ranked_population[0][1]

  

    @staticmethod
    def evaluateAssignment(list assignment):
        cdef dict type_bin
        cdef int i,j,k
        
        type_bin=dict()
        for i in range(num_bins):
            if assignment[i] not in type_bin:
                type_bin[assignment[i]]=[]
            type_bin[assignment[i]].append(i)
        cdef double soc
        min_dist=np.zeros([num_station,types])
        for i in range(num_station):
            for j in range(types):
                min_dist[i][j]=min([distance(station_position[i],bin_positions[k]) for k in type_bin[j]])
        soc=0
        for i in range(num_station):
            for c,nc in enumerate(prob[i]):
                soc=soc+nc*min_dist[i][c]
        return soc

    def random_sample(self):
        # print(types,num_bins)
        # config1=list(range(types))+random.sample(list(range(types)),k=num_bins-types)
        cdef list config1
        config1=list(range(types))+[np.random.randint(0,types)for i in range(num_bins-types)]
        np.random.shuffle(config1)
        return config1

  
    def initial_population(self):
        cdef list p1,greed_config,greedy_population
        p1=[self.random_sample() for _ in range(self.population_size-self.greed_seed)]
        greed_config=greedy.greedy_allocating(prob)
        greedy_population=[greed_config for i in range(self.greed_seed)]
        return [*p1,*greedy_population]

    def rank_population(self):
        cdef list fitness
        fitness=[(chromosome,1.0/self.evaluateAssignment(chromosome)) for chromosome in self.population]
        self.ranked_population=sorted(fitness,key=lambda f:f[1],reverse=True)

    def greedy(self):
        pass
  
    def selection(self):
        cdef list selections
        cdef int pick
        selections = [self.ranked_population[i][0] for i in range(self.elites_num)]
        
        
        if self.roulette_selection:
          
            df = pd.DataFrame(np.array(self.ranked_population), columns=["index", "fitness"])
            self.average_path_cost = sum(1 / df.fitness) / len(df.fitness)
            df['cum_sum'] = df.fitness.cumsum()
            df['cum_perc'] = 100 * df.cum_sum / df.fitness.sum()

            for _ in range(0, self.population_size - self.elites_num):
                pick = 100 * np.random.random()
                for i in range(0, len(self.ranked_population)):
                    if pick <= df.iat[i, 3]:
                        selections.append(self.ranked_population[i][0])
                        break
        else:
            
            for _ in range(0, self.population_size - self.elites_num):
                # t0=time.time()
                pick = np.random.randint(0, self.population_size - 1)
                # t1=time.time()
                # print("generate random", t1-t0)
                selections.append(self.ranked_population[pick][0])
        
        self.population = selections
      
        # print("selection=",t1-t0,"elite=",t2-t1)
      

    @staticmethod
    def produce_child(parent1,parent2):
        cdef int gene_1,gene_2,bk
        cdef list child,unallocated_types,bins
        cdef dict type_bin
        gene_1 = random.randint(0, len(parent1))
        gene_2 = random.randint(0, len(parent1))
        gene_1, gene_2 = min(gene_1, gene_2), max(gene_1, gene_2)
        child=parent1[0:gene_1]+parent2[gene_1:gene_2]+parent1[gene_2:]
        
        type_bin=dict()
    
        for b,c in enumerate(child):
            if c not in type_bin:
                type_bin[c]=[]
            type_bin[c].append(b)
        unallocated_types=[c for c in range(types) if c not in type_bin]
        while len(unallocated_types)!=0:
            c1=unallocated_types.pop(0)
            c2,bins=max(type_bin.items(),key=lambda x: len(x[1]))
            bk=bins[np.random.randint(len(bins))]
            type_bin[c1]=[bk]
            type_bin[c2].remove(bk)
            child[bk]=c1
        return child

    def generate_population(self):
        cdef int length
        cdef list children
        cdef int i
        length = len(self.population) - self.elites_num
        children = self.population[:self.elites_num]
        for i in range(0, length):
            child = self.produce_child(self.population[i],self.population[(i + np.random.randint(1, self.elites_num)) % length])
            children.append(child)
        return children

    def mutate(self, list individual):

        cdef list possible_indexes
        cdef int random_close_index,index
        for index, city in enumerate(individual):
            if np.random.random() < max(0, self.mutation_rate):
                possible_indexes=[id for id in individual if id!=index]
                # sample_size = min(min(max(3, self.population_size // 5), 100), len(individual))
                # random_sample = random.sample(range(len(individual)), sample_size)
                # sorted_sample = sorted(random_sample,key=lambda c_i: individual[c_i].distance(individual[index - 1]))
                random_close_index=random.choice(possible_indexes)
                # random_close_index = np.random.choice(sorted_sample[:max(sample_size // 3, 2)])
                individual[index], individual[random_close_index] = \
                    individual[random_close_index], individual[index]
        return individual

    def next_generation(self):
        # t1=time.time()
        self.rank_population()
        # t2=time.time()
        self.selection()
        # t3=time.time()
        self.population = self.generate_population()
        # t4=time.time()
        self.population[self.elites_num:] = [self.mutate(chromosome)for chromosome in self.population[self.elites_num:]]
        # print("rank popuplation=",t2-t1,"selection=",t3-t2,"generate_population=",t4-t3)
    def run(self):
        cdef int ind
        # if self.plot_progress:
        #     plt.ion()

        for ind in range(0,self.iterations):
            
            self.next_generation()
            
            self.progress.append(self.best_cost())
            # if self.plot_progress and ind %10==0:
            #     self.plot()
            # elif not self.plot_progress and ind%10==0:
            #     print(self.best_cost())
    
  
    def plot(self):
        print(self.best_cost())
        fig=plt.figure(0)
        plt.plot(self.progress,'g')
        fig.suptitle('genetic algorithm generations')
        plt.ylabel("Cost")
        plt.xlabel("Generation")
        if self.plot_progress:
            plt.draw()
            plt.pause(0.05)
        plt.show()


if __name__=="__main__":
    genetic_algorithm = GeneticAlgorithm(iterations=1200, population_size=100,elites_num=20, mutation_rate=0.008,  roulette_selection=False, plot_progress=False)
    genetic_algorithm.run()
    assignment=genetic_algorithm.best_chromosome()
    print(assignment)
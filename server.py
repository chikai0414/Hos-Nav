#!/usr/bin/env python
from pickle import STRING
import rospy
from std_msgs.msg import String , Float32

from math import ceil, sqrt
import random as rd
import copy,time
from matplotlib import pyplot as plt
num = 2
history = []
class Location:
    def __init__(self, name, x, y, x2, y2):
        self.name = name
        self.loc_start = (x, y)
        self.loc_end = (x2, y2)
        self.length = sqrt(pow((self.loc_start[0] - self.loc_end[0]),2) + pow((self.loc_start[1] - self.loc_end[1]),2))
    def distance_between(self, pre):
        assert isinstance(pre, Location)
        #print(self.loc)
        #print(location2.loc)
        #print(sqrt(pow((self.loc[0] - location2.loc[0]),2) + pow((self.loc[1] - location2.loc[1]),2)))
        return (sqrt(pow((self.loc_start[0] - pre.loc_end[0]),2) + pow((self.loc_start[1] - pre.loc_end[1]),2))) + self.length


def create_locations():
    locations = []
    
    xs = [8, 50, 18, 35, 90, 40, 84, 74]
    ys = [3, 62,  0, 25, 89, 71,  7, 29]
    xe = [74, 34, 40, 60, 74,  6, 43]
    ye = [29, 45, 65, 69, 47, 10, 26]
    start = ['Z', 'P', 'A', 'K', 'O', 'Y', 'N']
    end = ['X', 'G', 'Q', 'S', 'J','B', 'C']
    '''
    xs = [ 0, 20, 40, 60, 60, 60, 60, 40, 20,  0,  0,  0]
    ys = [ 0,  0,  0,  0, 20, 40, 60, 60, 60, 60, 40, 20]
    xe = [10, 30, 50, 60, 60, 60, 50, 30, 10,  0,  0,  0]
    ye = [ 0,  0,  0, 10, 30, 50, 60, 60, 60, 50, 30, 10]
    start = ['1', '2', '3', '4', '5', '6', '7','8','9','0','A','B']
    end = ['1', '2', '3', '4', '5', '6', '7','8','9','0','A','B']
    '''
    for x, y, x2, y2, name, name2 in zip(xs, ys, xe, ye, start, end):
        locations.append(Location(name, x, y, x2, y2))
    return locations, xs, ys, xe, ye, start, end


class Route:
    def __init__(self,path):
        # path is a list of Location obj
        self.path = path
        self.l = []
        self.length = self._set_length()
        self.short = self._shortest()
        self.long = self._longest()



    def _set_length(self):
        total_length = 0
        self.l = []
        for i in self.path:
            total_length = 0
            path_copy = i[:]
            if path_copy:
                #print("    ")
                from_here = path_copy.pop(0)
                init_node = copy.deepcopy(from_here)
                total_length += init_node.length
                #print(init_node.name)
                while path_copy:
                    to_there = path_copy.pop(0)
                    total_length += to_there.distance_between(from_here)
                    #print(from_here.name + to_there.name)
                    #print(to_there.distance_between(from_here))
                    from_here = copy.deepcopy(to_there)
                    
            #total_length += from_here.distance_between(init_node)
            #print(total_length)
            self.l.append(total_length)

        self.length = max(self.l)
        #return total_length
        return max(self.l)

    def _shortest(self):
        value = copy.deepcopy(self.l[0])
        index = 0
        for i in range(len(self.l)):
            if(self.l[i] < value) :
                value = copy.deepcopy(self.l[i])
                index = i
        return index

    def _longest(self):
        value = copy.deepcopy(self.l[0])
        index = 0
        for i in range(len(self.l)):
            if(self.l[i] > value) :
                value = copy.deepcopy(self.l[i])
                index = i
        return index

    def _sum(self):
        return sum(self.l)

    def _p(self):
        for i in self.l:
            print(i)

    def _insert(self,loc):
        x = rd.randint(0,len(self.path)-1)
        while len(self.path[x]) < 1 :
            x = rd.randint(0,len(self.path)-1)
        y = rd.randint(0,len(self.path[x])-1)
        self.path[x] = self.path[x][:y] + [loc] + self.path[x][y:]
        self.length = self._set_length()
        self.short = self._shortest()
        self.long = self._longest()

    def _remove(self,loc):
        for i in self.path:
            if loc in i:
                i.remove(loc)
                self.length = self._set_length()
                self.short = self._shortest()
                self.long = self._longest()

class GeneticAlgo:
    def __init__(self, locs, level=40, populations=100, variant=3, mutate_percent=0.1, elite_save_percent=0.1):
        self.locs = locs
        self.level = level
        self.variant = variant
        self.populations = populations
        self.mutates = int(populations * mutate_percent)
        self.elite = int(populations * elite_save_percent)
        self.routes = self._init_routes()

    def _find_path(self):
        # locs is a list containing all the Location obj
        locs_copy = self.locs[:]
        n = len(locs_copy)
        path = []
        sub = []
        for i in range(num):
            path.append([])
        for i in range(n):
            to_there = locs_copy.pop(locs_copy.index(rd.choice(locs_copy)))
            path[i%num].append(to_there)

        #print([[loc.name for loc in p]for p in path])
        #print([loc.name for loc in path2])
        return path
    def _insert(self,loc):
        for i in self.routes:
            i._insert(loc)
        my_locs.append(loc)
        #print(self.locs.name)

    def _remove(self,loc):
        for i in self.routes:
            i._remove(loc)
        my_locs.remove(loc)

    def _init_routes(self):
        routes = []
        for _ in range(self.populations):
            path = self._find_path()
            routes.append(Route(path))
        return routes

    def _get_next_route(self, routes):
        routes.sort(key=lambda x: x.length, reverse=False)
        elites = routes[:self.elite][:]
        crossovers = self._crossover(routes)
        return crossovers[:] + elites

    def _crossover(self, elites):
        # Route is a class type
        normal_breeds = []
        mutate_ones = []
        for _ in range(self.populations - self.mutates):
            variant = rd.randint(1,self.variant)
            father, mother = rd.sample(elites[:], k=2)
            #print([loc.name for loc in father.path1])
            #print([loc.name for loc in father.path2])
            #print([[loc.name for loc in path]for path in father.path])
            #print(father.long)
            #print(len(father.path[father.long]))
            #print(father.l[father.long])
            #print([loc.name for loc in mother.path1])
            #print([loc.name for loc in mother.path2])
            index_start = rd.randint(0, len(father.path[father.long]) - variant)
            # list of Location obj
            father_gene = father.path[father.long][index_start: index_start + variant]
            father_gene_names = [loc.name for loc in father_gene]

        
            #father_gene = father1_gene + father2_gene
            #print([loc.name for loc in father1_gene])
            #print([loc.name for loc in father2_gene])
            #print(father_gene_names)
            mother_gene = []
            for i in range(num):
                mother_gene.append([gene for gene in mother.path[i] if gene.name not in father_gene_names])

            s_index = Route(mother_gene).short

            mother_gene_cut = []
            
            mother_gene_cut = rd.randint(0, len(mother_gene[s_index]))

            #print([loc.name for loc in mother1_gene])
            next_route_path = []
            for i in range(num):
                if i == s_index:
            # create new route path
                    next_route_path.append(mother_gene[i][:mother_gene_cut] + father_gene[:] + mother_gene[i][mother_gene_cut:])
                else:
                    next_route_path.append(mother_gene[i])
            #print([loc.name for loc in next_route_path1])
            #print([loc.name for loc in next_route_path2])
            #print(" -------   ")

            next_route = Route(next_route_path)
            # add Route obj to normal_breeds
            normal_breeds.append(next_route)

            # for mutate purpose
            copy_father = copy.deepcopy(father)

            x1 = rd.randint(0,len(copy_father.path)-1)
            while len(copy_father.path[x1]) < 1 :
                x1 = rd.randint(0,len(copy_father.path)-1)
            y1 = rd.randint(0,len(copy_father.path[x1])-1)

            x2 = rd.randint(0,len(copy_father.path)-1)
            while len(copy_father.path[x2]) < 1 :
                x2 = rd.randint(0,len(copy_father.path)-1)
            y2 = rd.randint(0,len(copy_father.path[x2])-1)
            #print(x1,y1,x2,y2)
            copy_father.path[x1][y1], copy_father.path[x2][y2] = copy_father.path[x2][y2], copy_father.path[x1][y1]

            mutate_ones.append(copy_father)
        for i in range(self.mutates*2):
            mutate_ones.append(Route(self._find_path()))
        mutate_breeds = rd.sample(mutate_ones, k=self.mutates)
        return normal_breeds + mutate_breeds

    def evolution(self):
        for _ in range(self.level):
            self.routes = self._get_next_route( self.routes)
            self.routes.sort(key=lambda x: x._set_length(),reverse=False)
            #history.append( self.routes[0].length)

            #print(self.routes[0].length)
            #print([[loc.name for loc in path]for path in routes[0].path])
            #print(" ")

        #routes.sort(key=lambda x: x.length, reverse=False)

        #print(routes[0]._sum())
        #print(routes[0].l)
        self.routes[0]._set_length()
        #print(routes[0].length)
        #routes.sort(key=lambda x: x.length)
        return self.routes[0].path,self.routes[0].length
    def best(self):
        return self.routes[0].path,self.routes[0].length

def task_index():
    task_index.counter += 1
    return task_index.counter
task_index.counter = 0

def addcallback(data):    
    task = data.data.split(",")
    my_algo._insert(Location("Task"+str(task_index()),int(task[0]),int(task[1]),int(task[2]),int(task[3])))
    rospy.loginfo("Add task %s", data.data)

    best_route,best_route_length = my_algo.best()
    print([[loc.name for loc in path]for path in best_route], best_route_length)


def delcallback(data):
    task = data.data.split(",")
    my_algo._remove(Location(task[0],int(task[1]),int(task[2]),int(task[3]),int(task[4])))
    rospy.loginfo("Del task %s",task[0])

def reqcallback(data):
    best_route,best_route_length = my_algo.best()
    print([[loc.name for loc in path]for path in best_route], best_route_length)
    if best_route[int(data.data)] != [] :
        task = best_route[int(data.data)][0]
    else : 
        pub = "None" 
        pub_task.publish(pub)
        return
    x1 = task.loc_start[0]
    y1 = task.loc_start[1]
    x2 = task.loc_end[0]
    y2 = task.loc_end[1]
    name = task.name
    pub = name + "," + str(x1) + "," + str(y1) + "," + str(x2) + "," + str(y2) 
    pub_task.publish(pub)
    rospy.loginfo("R %s req task %s",int(data.data),name)
    my_algo._remove(task)
    rospy.loginfo("Del task %s",name)
    best_route,best_route_length = my_algo.best()
    print([[loc.name for loc in path]for path in best_route], best_route_length)
    print([loc.name for loc in my_locs])




my_locs, xs, ys, xe, ye, start, end = create_locations() 
my_algo = GeneticAlgo(my_locs, level=40, populations=200, variant=2, mutate_percent=0.3, elite_save_percent=0.1)
best_route = []
best_route_length = []
if __name__ == '__main__':
    rospy.init_node('task_mod', anonymous=True)
    rospy.Subscriber("task/add", String, addcallback)
    rospy.Subscriber("task/del", String, delcallback)
    rospy.Subscriber("task/req", String, reqcallback)
    pub_task = rospy.Publisher('task/pub', String, queue_size=5)
    stop = False
    sch = True
    index = 0
    best_route,best_route_length = my_algo.evolution()
    rospy.spin()
    while stop:
        if sch :
            best_route,best_route_length = my_algo.evolution()
            print([[loc.name for loc in path]for path in best_route], best_route_length)
            
            my_algo._insert(Location("T"+str(index),rd.randint(0,50),rd.randint(0,50),rd.randint(0,50),rd.randint(0,50)))
            my_algo._remove(my_locs[0])
            print([loc.name for loc in my_locs])
            index += 1

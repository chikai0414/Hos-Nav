from math import sqrt
import random as rd
import copy
from matplotlib import pyplot as plt


class Location:
    def __init__(self, name, x, y):
        self.name = name
        self.loc = (x, y)

    def distance_between(self, location2):
        assert isinstance(location2, Location)
        #print(self.loc)
        #print(location2.loc)
        #print(sqrt(pow((self.loc[0] - location2.loc[0]),2) + pow((self.loc[1] - location2.loc[1]),2)))
        return (sqrt(pow((self.loc[0] - location2.loc[0]),2) + pow((self.loc[1] - location2.loc[1]),2)))


def create_locations():
    locations = []
    xs = [8, 50, 18, 35, 90, 40, 84, 74, 34, 40, 60, 74]
    ys = [3, 62, 0, 25, 89, 71, 7, 29, 45, 65, 69, 47]
    cities = ['Z', 'P', 'A', 'K', 'O', 'Y', 'N', 'X', 'G', 'Q', 'S', 'J']
    for x, y, name in zip(xs, ys, cities):
        locations.append(Location(name, x, y))
    return locations, xs, ys, cities


class Route:
    def __init__(self,path1,path2):
        # path is a list of Location obj
        self.path1 = path1
        self.path2 = path2
        self.length = self._set_length()

    def _set_length(self):
        total_length = 0
        path_copy = self.path1[:]
        from_here = path_copy.pop(0)
        init_node = copy.deepcopy(from_here)
        while path_copy:
            to_there = path_copy.pop(0)
            total_length += to_there.distance_between(from_here)
            from_here = copy.deepcopy(to_there)
        #total_length += from_here.distance_between(init_node)

        path_copy = self.path2[:]
        from_here = path_copy.pop(0)
        init_node = copy.deepcopy(from_here)
        while path_copy:
            to_there = path_copy.pop(0)
            total_length += to_there.distance_between(from_here)
            from_here = copy.deepcopy(to_there)
        #total_length += from_here.distance_between(init_node)

        #print(total_length)
        return total_length


class GeneticAlgo:
    def __init__(self, locs, level=40, populations=100, variant=3, mutate_percent=0.1, elite_save_percent=0.1):
        self.locs = locs
        self.level = level
        self.variant = variant
        self.populations = populations
        self.mutates = int(populations * mutate_percent)
        self.elite = int(populations * elite_save_percent)

    def _find_path(self):
        # locs is a list containing all the Location obj
        locs_copy = self.locs[:]
        n = len(locs_copy)
        path1 = []
        path2 = []
        while locs_copy:
            to_there = locs_copy.pop(locs_copy.index(rd.choice(locs_copy)))
            if len(locs_copy) >= n/2:
                path1.append(to_there)
            else:
                path2.append(to_there)
        #print([loc.name for loc in path1])
        #print([loc.name for loc in path2])
        return path1,path2

    def _init_routes(self):
        routes = []
        for _ in range(self.populations):
            path1,path2 = self._find_path()
            routes.append(Route(path1,path2))
        return routes

    def _get_next_route(self, routes):
        routes.sort(key=lambda x: x.length, reverse=False)
        elites = routes[:self.elite][:]
        crossovers = self._crossover(elites)
        return crossovers[:] + elites

    def _crossover(self, elites):
        # Route is a class type
        normal_breeds = []
        mutate_ones = []
        for _ in range(self.populations - self.mutates):
            father, mother = rd.sample(elites[:4], k=2)
            #print([loc.name for loc in father.path1])
            #print([loc.name for loc in father.path2])

            #print([loc.name for loc in mother.path1])
            #print([loc.name for loc in mother.path2])
            index_start = rd.randrange(0, len(father.path1) - self.variant - 1)
            # list of Location obj
            father1_gene = father.path1[index_start: index_start + self.variant]
            father2_gene = father.path2[index_start: index_start + self.variant]

            father_gene_names = [loc.name for loc in father1_gene + father2_gene]


            father_gene = father1_gene + father2_gene
            #print([loc.name for loc in father1_gene])
            #print([loc.name for loc in father2_gene])
            #print(father_gene_names)


            mother1_gene = [gene for gene in mother.path1 if gene.name not in father_gene_names]
            mother2_gene = [gene for gene in mother.path2 if gene.name not in father_gene_names]

            mother1_gene_cut = rd.randrange(1, len(mother1_gene))
            mother2_gene_cut = rd.randrange(1, len(mother2_gene))

            #print([loc.name for loc in mother1_gene])
            index = len(mother.path1)-len(mother1_gene)

            # create new route path
            next_route_path1 = mother1_gene[:mother1_gene_cut] + father_gene[:index] + mother1_gene[mother1_gene_cut:]
            next_route_path2 = mother2_gene[:mother2_gene_cut] + father_gene[index:] + mother2_gene[mother2_gene_cut:]

            #print([loc.name for loc in next_route_path1])
            #print([loc.name for loc in next_route_path2])
            #print(" -------   ")

            next_route = Route(next_route_path1,next_route_path2)
            # add Route obj to normal_breeds
            normal_breeds.append(next_route)

            # for mutate purpose
            copy_father = copy.deepcopy(father)
            idx = range(len(copy_father.path1))

            gene1, gene2 = rd.sample(idx, 2)

            n = rd.randint(0,2)
            if n == 1:
                copy_father.path1[gene1], copy_father.path1[gene2] = copy_father.path1[gene2], copy_father.path1[gene1]
            elif n == 2:
                copy_father.path2[gene1], copy_father.path2[gene2] = copy_father.path2[gene2], copy_father.path2[gene1]
            else:
                copy_father.path1[gene1], copy_father.path2[gene2] = copy_father.path2[gene2], copy_father.path1[gene1]

            mutate_ones.append(copy_father)

        mutate_breeds = rd.sample(mutate_ones, k=self.mutates)
        return normal_breeds + mutate_breeds

    def evolution(self):
        routes = self._init_routes()
        for _ in range(self.level):
            routes = self._get_next_route(routes)
            routes.sort(key=lambda x: x.length)
            print(routes[0].length)
        routes.sort(key=lambda x: x.length)
        return routes[0].path1,routes[0].path2,routes[0].length


if __name__ == '__main__':
    my_locs, xs, ys, cities = create_locations()
    my_algo = GeneticAlgo(my_locs, level=500, populations=300, variant=2, mutate_percent=0.3, elite_save_percent=0.05)
    best_route1,best_route2, best_route_length = my_algo.evolution()
    #best_route.append(best_route[0])
    print([loc.name for loc in best_route1], best_route_length)
    print([loc.name for loc in best_route2], best_route_length)

    #print([(loc.loc[0], loc.loc[1]) for loc in best_route], best_route_length)

    fig, ax = plt.subplots()
    ax.plot([loc.loc[0] for loc in best_route1], [loc.loc[1] for loc in best_route1], 'red', linestyle='-', marker='')
    ax.plot([loc.loc[0] for loc in best_route2], [loc.loc[1] for loc in best_route2], 'blue', linestyle='-', marker='')

    ax.scatter(xs, ys)
    for i, txt in enumerate(cities):
        ax.annotate(txt, (xs[i], ys[i]))
    plt.show()
    print(best_route_length)

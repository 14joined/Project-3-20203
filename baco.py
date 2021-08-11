import random
from pheromone_matrix import PheromoneMatrix


def roulette_wheel_selection(customer_set, ci):
    pass


def trace_back(nodes, r):
    pass


def local_search(route):
    pass


def route_construct(pheromone_mtx, arc_set, customer_set, max_capacity):
    r = []
    routes = []
    c_set = customer_set

    i = random.choice(c_set)
    r.append(i)
    c_set.remove(i)

    while len(c_set) > 0:
        last_node = r[-1]
        j = roulette_wheel_selection(c_set, last_node)
        r.append(j)
        c_set.remove(j)

    r.insert(0, 0)
    v = []
    p = []
    for i in range(len(customer_set) + 1):
        p[i] = 0
        if i == 0:
            v[i] = 0
            continue
        v[i] = float('inf')

    for i in range(1, len(customer_set) + 1):
        j = i
        total_capacity = 0
        total_distance = 0
        while j <= len(customer_set) || total_capacity <= max_capacity:
            total_capacity += capacities[r[j]]
            if i == j:
                total_distance = distances[0][r[i]] + distances[r[j]][0]
            else:
                total_distance = total_distance - distances[0][r[j - 1]] + distances[r[j - 1]][r[j]] + distances[r[j]][0]

            if total_capacity <= max_capacity:
                j += 1
                if v[i - 1] + total_distance < v[j]:
                    v[j] = v[i - 1] + total_distance

    routes = trace_back(p, r)

    for i in range(len(routes)):
        local_search(routes[i])
    
    return routes


def removal_heuristic():
    pass


def restricted_enumeration():
    
    pass


class BACO():

    def __init__(self, pheromone_mtx, lower_bound, upper_bound, x):
        self.pheromone_mtx = pheromone_mtx
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.x = x

    def calcFitnessValues():
        pass
        
    def update():
        pass

#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np
import sys



np.random.seed(444)
# Rapidly-exploring Random Trees*
class RRT_star(GridMap):
    def __init__(self):
        super(RRT_star, self).__init__()
        self.parent_distance = {}
        self.step = 0.2
        self.neighborhood_step = 0.3

    def random_point(self):
        x = self.width * np.random.random(1)
        y = self.height * np.random.random(1)
        # x = round(x, 4)
        # y = round(y, 4)
        point = (x[0], y[0])
        return point

    def find_closest(self, pos):
        pkt = pos
        punkt = (pkt[0], pkt[1])
        min = 1000  # duza wartosc
        x = 0
        y = 0
        for i, wartosc in enumerate(self.parent):
            odleglosc = np.sqrt(
                (punkt[0] - wartosc[0]) * (punkt[0] - wartosc[0]) + (punkt[1] - wartosc[1]) * (punkt[1] - wartosc[1]))

            if (min > odleglosc):
                min = odleglosc
                x = wartosc[0]
                y = wartosc[1]

        # x = round(x, 4)
        # y = round(y, 4)
        closest = (x, y)
        return closest

    def check_if_valid(self, a, b):
        ilosc_probek = 100
        dlugosc = np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        lista = []
        Yc = a[1]
        Xc = a[0]
        step_x = abs(a[0] - b[0]) / ilosc_probek
        step_y = abs(a[1] - b[1]) / ilosc_probek

        if (b[1] > Yc):
            Yc = ((step_y * abs(b[1] - Yc)) / (dlugosc)) + Yc

        if (b[1] < Yc):
            Yc = (-1 * ((step_y * abs(b[1] - Yc)) / (dlugosc))) + Yc

        if (b[0] > Xc):
            Xc = ((step_x * abs(b[0] - Xc)) / (dlugosc)) + Xc

        if (b[0] < Xc):
            Xc = (-1 * ((step_x * abs(b[0] - Xc)) / (dlugosc))) + Xc
        lista.append([Xc, Yc])

        for i in range(ilosc_probek - 1):
            if (b[1] > Yc):
                Yc = Yc + step_y
            if (b[1] < Yc):
                Yc = Yc - step_y
            if (b[0] > Xc):
                Xc = Xc + step_x
            if (b[0] < Xc):
                Xc = Xc - step_x

            lista.append([Xc, Yc])

        for i in range(ilosc_probek):
            i=i+1
            punkt = lista[-i]
            Xc = int(punkt[0] * 10)
            Yc = int(punkt[1] * 10)

            if (self.map[Yc][Xc] == 100):
                in_free_space = False
                return in_free_space

        in_free_space = True
        return in_free_space

    def new_pt(self, pt, closest):
        b = pt
        a = closest
        dlugosc = np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
        Yc = 0
        Xc = 0
        if (b[1] > a[1]):
            Yc = ((self.step * abs(b[1] - a[1])) / (dlugosc)) + a[1]
        if (b[1] < a[1]):
            Yc = (-1 * ((self.step * abs(b[1] - a[1])) / (dlugosc))) + a[1]
        if (b[0] > a[0]):
            Xc = ((self.step * abs(b[0] - a[0])) / (dlugosc)) + a[0]
        if (b[0] < a[0]):
            Xc = (-1 * ((self.step * abs(b[0] - a[0])) / (dlugosc))) + a[0]

        # Xc = round(Xc, 4)
        # Yc = round(Yc, 4)
        pt = (Xc, Yc)
        return pt


    def checking_connections(self, random_point_in_self_step):
        neighbour_list = []
        for i, neighbour in enumerate(self.parent):
            # calculate distance/cost from random point to neighborhod
            distance = np.sqrt((random_point_in_self_step[0] - neighbour[0]) ** 2 + (random_point_in_self_step[1] - neighbour[1]) ** 2)
            # choice points in self.neighborhood_step
            if distance < self.neighborhood_step:
                if rrt_star.check_if_valid(random_point_in_self_step, neighbour) and neighbour != random_point_in_self_step:
                    # calculate cost from start to neighborhod
                    cost = distance + self.parent_distance[neighbour]
                    # add cost to cost list
                    neighbour_list.append((cost, random_point_in_self_step, neighbour))
        # print("number of neighbours ", len(neighbour_list))
        # find lowes cost
        neighbour_list = sorted(neighbour_list)
        # if rrt_star.check_if_valid(cost_list[0][1], cost_list[0][2]):
        lowest_cost = neighbour_list[0][0]
        # add lowest cost do cost dict
        self.parent_distance[neighbour_list[0][1]] = lowest_cost
        # print("random point",neighbour_list[0][1], "cost to start", lowest_cost, "distance to neighbour", lowest_cost - self.parent_distance[neighbour_list[0][2]])
        # add point with lowest cost to parent dict
        self.parent[neighbour_list[0][1]] = neighbour_list[0][2]
        # print("rodzic", self.parent[neighbour_list[0][1]], "tego wierzcholka ", neighbour_list[0][1] , "neighbour", neighbour_list[0][2])
        # print("1", neighbour_list)

        rrt_star.rewire(neighbour_list)


    def rewire(self, neighbour_list):
        '''
        Rewire every 20 points
        '''
        # print(self.parent_distance)
        # print(self.parent)
        print("test")
        print(len(self.parent))
        if (len(self.parent)%20) == 0:
            for i1, parent1 in enumerate(self.parent):
                for i2, parent2 in enumerate(self.parent):
                    if parent1 != self.start and parent2 != self.start:
                        distance = np.sqrt((parent1[0] - parent2[0]) ** 2 + (
                                    parent1[1] - parent2[1]) ** 2) + self.parent_distance[parent2]
                        # choice points in self.neighborhood_step
                        # print(distance, self.parent_distance[parent2], self.parent_distance[parent1])
                        if distance < self.parent_distance[parent1]\
                                and parent1 != parent2 \
                                and distance < self.neighborhood_step\
                                and rrt_star.check_if_valid(parent1, parent2):
                            print("--------------------------------------")
                            print(parent1, parent2, "new dist", distance, "old dist",self.parent_distance[parent1])
                            self.parent_distance[parent1] = distance
                            self.parent[parent1] = parent2

        print("--------------------------------------")



    def search(self):
        print("RRT*")
        self.parent[self.start] = None
        self.parent_distance[self.start] = 0
        # print("test")
        # print(self.parent)
        # print(self.parent_distance)
        path = []

        test_dic = {}
        test_dic[(1, 2)] = 0
        test_dic[(1, 3)] = 2
        test_dic[(2, 2)] = 1
        test_dic[(3, 2)] = 2
        # print(test_dic)

        # self.parent { klucz,                wartosc   }
        # self.parent { wierzcholek dziecko , wiezcholek rodzic   }
        # self.parent[key] = value
        # self.parent[wierzcholek dziecko] = wiezcholek rodzic
        while not rp.is_shutdown():


            random_point = rrt_star.random_point()
            closest_point = rrt_star.find_closest(random_point)
            random_point_in_self_step = rrt_star.new_pt(random_point, closest_point)

            if rrt_star.check_if_valid(random_point_in_self_step, closest_point):
                rrt_star.checking_connections(random_point_in_self_step)

                # rrt_star.rewire(random_point_in_self_step)

            self.publish_search()
            rp.sleep(0.050)


if __name__ == '__main__':
    rrt_star = RRT_star()
    rrt_star.search()

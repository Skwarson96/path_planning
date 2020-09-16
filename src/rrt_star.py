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
        self.neighborhood_step = 0.25

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
        cost_list = []
        for i, value in enumerate(self.parent):
            # calculate distance/cost from random point to every point in parent dict
            distance = np.sqrt((random_point_in_self_step[0] - value[0]) ** 2 + (random_point_in_self_step[1] - value[1]) ** 2)
            if distance < self.neighborhood_step:
                # calculate cost
                cost = distance + self.parent_distance[value]
                # add cost to cost list
                cost_list.append((cost, random_point_in_self_step, value))

        # find lowes cost
        cost_list = sorted(cost_list)
        lowest_cost = cost_list[0][0]
        # add lowest cost do cost dict
        self.parent_distance[cost_list[0][1]] = lowest_cost
        # add point with lowest cost to parent dict
        self.parent[cost_list[0][1]] = cost_list[0][2]



    def chose_parent(self):
        pass

    def search(self):
        print("RRT*")
        self.parent[self.start] = None
        self.parent_distance[self.start] = 0
        path = []

        # self.parent { klucz,                wartosc   }
        # self.parent { wierzcholek dziecko , wiezcholek rodzic   }

        while not rp.is_shutdown():


            random_point = rrt_star.random_point()
            closest_point = rrt_star.find_closest(random_point)
            random_point_in_self_step = rrt_star.new_pt(random_point, closest_point)

            if rrt_star.check_if_valid(random_point_in_self_step, closest_point):
                rrt_star.checking_connections(random_point_in_self_step)

            self.publish_search()
            rp.sleep(0.50)




if __name__ == '__main__':
    rrt_star = RRT_star()
    rrt_star.search()

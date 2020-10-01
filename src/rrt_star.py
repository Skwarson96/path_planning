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
        self.rewire_number = 20

    def random_point(self):
        """
        Draws random point in 2D

        :return: point in 2D
        """
        x = self.width * np.random.random(1)
        y = self.height * np.random.random(1)
        point = (x[0], y[0])
        return point

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
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
        """
        Checks if the segment connecting a and b lies in the free space.

        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
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
        """
        Finds last point in the free space on the segment connecting closest with pt

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
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


        pt = (Xc, Yc)
        return pt

    def checking_connections(self, random_point_in_self_step):
        """
        Checks the connection between points in the graph and random_point_in_self_step.
        Adds the point with the lowest cost of getting to start to the graph

        :param random_point_in_self_step: point in 2D in distance self_step from closest point in graph
        """
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
        # find lowes cost
        neighbour_list = sorted(neighbour_list)
        # if rrt_star.check_if_valid(cost_list[0][1], cost_list[0][2]):
        lowest_cost = neighbour_list[0][0]
        # add lowest cost do cost dict
        self.parent_distance[neighbour_list[0][1]] = lowest_cost
        # add point with lowest cost to parent dict
        self.parent[neighbour_list[0][1]] = neighbour_list[0][2]

        # rewire
        rrt_star.rewire()


    def rewire(self):
        '''
        Every 20 new points, it checks the connections between the points in the graph.
        If there is a connection with a lower cost of getting to start than the present one,
        it changes the order of the points in the graph.
        '''

        # print("Number of points:", len(self.parent))
        if (len(self.parent)%self.rewire_number) == 0:
            for i1, parent1 in enumerate(self.parent):
                for i2, parent2 in enumerate(self.parent):
                    if parent1 != self.start and parent2 != self.start:
                        distance_to_parent = np.sqrt((parent1[0] - parent2[0]) ** 2 + (
                                    parent1[1] - parent2[1]) ** 2)
                        distance_to_start = distance_to_parent + self.parent_distance[parent2]
                        if distance_to_start < self.parent_distance[parent1]\
                                and parent1 != parent2 \
                                and distance_to_parent < self.neighborhood_step\
                                and rrt_star.check_if_valid(parent1, parent2):
                            self.parent_distance[parent1] = distance_to_start
                            self.parent[parent1] = parent2



    def search(self):
        """
        RRT star search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """

        print("RRT*")
        self.parent[self.start] = None
        self.parent_distance[self.start] = 0
        path = []
        while not rp.is_shutdown():
            random_point = rrt_star.random_point()
            closest_point = rrt_star.find_closest(random_point)
            random_point_in_self_step = rrt_star.new_pt(random_point, closest_point)

            if rrt_star.check_if_valid(random_point_in_self_step, closest_point):
                rrt_star.checking_connections(random_point_in_self_step)

            last_point = closest_point
            if rrt_star.check_if_valid(self.end, closest_point):
                self.parent[random_point_in_self_step] = self.end
                print("End point found")
                break

            self.publish_search()

            # delay for better view
            rp.sleep(0.050)

        # Path
        path.append(self.end)
        path.append(last_point)
        while last_point != self.start:
            last_point = self.parent[last_point]
            path.append(last_point)
            if last_point == self.start:
                break
        self.publish_path(path)
        print("Path printed")



if __name__ == '__main__':
    rrt_star = RRT_star()
    rrt_star.search()

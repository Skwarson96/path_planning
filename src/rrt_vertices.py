#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.1

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.

        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        ilosc_probek = 100
        dlugosc = np.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))

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
            punkt = lista[i]
            Xc = int(punkt[0] * 10)
            Yc = int(punkt[1] * 10)

            #print("Mapa: ", np.shape(self.map))
            #print(self.map[0][0])
            #print("Szerokosc: ", self.width)
            #print("Wysokosc: ", self.height)

            if (self.map[Yc][Xc] == 100):
                in_free_space = False
                return in_free_space

        in_free_space = True
        return in_free_space

    def random_point(self):
        """
        Draws random point in 2D

        :return: point in 2D
        """
        x = self.width * np.random.random(1)
        y = self.height * np.random.random(1)
        x = round(x, 4)
        y = round(y, 4)
        punkt = (x, y)

        return punkt

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
        pkt = pos
        punkt = (pkt[0], pkt[1])

        min = 1000 # duza wartosc
        x = 0
        y = 0
        for i, wartosc in enumerate(self.parent):
            odleglosc = np.sqrt((punkt[0] - wartosc[0]) * (punkt[0] - wartosc[0]) + (punkt[1] - wartosc[1]) * (punkt[1] - wartosc[1]))

            if (min > odleglosc):
                min = odleglosc
                x = wartosc[0]
                y = wartosc[1]

        closest = (x, y)
        return closest

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

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

    def search(self):
        """
        RRT (vertices) search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """

        self.parent[self.start] = None
        path = []
        while not rp.is_shutdown():
            # Child
            randomPoint = rrt.random_point()
            # Parent
            closestPoint = rrt.find_closest(randomPoint)

            punkt_w_odleglosci_self_step = rrt.new_pt(randomPoint, closestPoint)

            if rrt.check_if_valid(punkt_w_odleglosci_self_step, closestPoint):
                self.parent[punkt_w_odleglosci_self_step] = closestPoint

            self.publish_search()
            last = closestPoint
            if rrt.check_if_valid(self.end, closestPoint):
                self.parent[punkt_w_odleglosci_self_step] = self.end
                break

            rp.sleep(0.05)

        print("Search complete")
        path.append(self.end)
        path.append(last)
        while last != self.start:
            last = self.parent[last]
            path.append(last)
            if last == self.start:
                break
        self.publish_path(path)
        print("Path printed")

if __name__ == '__main__':
    rrt = RRT()
    rrt.search()



























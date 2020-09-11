#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)
# Probabilistic Road Map
class PRM(GridMap):
    def __init__(self):
        super(PRM, self).__init__()
        self.step = 0.1

    def sampling(self, n):
        # print("Mapa: ", np.shape(self.map))
        # print(self.map[0][0])
        # print("Szerokosc: ", self.width)
        # print("Wysokosc: ", self.height)

        for i in range(n):
            x = self.width * np.random.random(1)
            y = self.height * np.random.random(1)
            # print(x[0], y[0])

            Xc = int(x[0] * 10)
            Yc = int(y[0] * 10)
            if (self.map[Yc][Xc] != 100):
                self.points_list.append((x[0], y[0]))

    def find_closest(self,r):
        distance_list = []
        for index1, point1 in enumerate(self.points_list):
            for index2, point2 in enumerate(self.points_list):
                distance = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

                # checking if the point is in range
                # does the point not equal itself
                # there is no obstacle between the points
                # whether the distance between points was not repeated
                # (prevents the same points from connecting a second time
                if (distance <= r) and \
                        (point1 != point2) and \
                        prm.check_if_valid(point1, point2) and \
                        (distance not in distance_list):
                    # print("point1", point1, "point2", point2, "distance", distance)
                    self.prm_points_to_connection.append((point1,point2))
                distance_list.append(distance)

    def check_if_valid(self, point1, point2):
        ilosc_probek = 100
        dlugosc = np.sqrt((point1[0] - point2[0]) * (point1[0] - point2[0]) + (point1[1] - point2[1]) * (point1[1] - point2[1]))

        lista = []
        Yc = point1[1]
        Xc = point1[0]
        step_x = abs(point1[0] - point2[0]) / ilosc_probek
        step_y = abs(point1[1] - point2[1]) / ilosc_probek

        if (point2[1] > Yc):
            Yc = ((step_y * abs(point2[1] - Yc)) / (dlugosc)) + Yc

        if (point2[1] < Yc):
            Yc = (-1 * ((step_y * abs(point2[1] - Yc)) / (dlugosc))) + Yc

        if (point2[0] > Xc):
            Xc = ((step_x * abs(point2[0] - Xc)) / (dlugosc)) + Xc

        if (point2[0] < Xc):
            Xc = (-1 * ((step_x * abs(point2[0] - Xc)) / (dlugosc))) + Xc
        lista.append([Xc, Yc])

        for i in range(ilosc_probek - 1):

            if (point2[1] > Yc):
                Yc = Yc + step_y
            if (point2[1] < Yc):
                Yc = Yc - step_y
            if (point2[0] > Xc):
                Xc = Xc + step_x
            if (point2[0] < Xc):
                Xc = Xc - step_x

            lista.append([Xc, Yc])

        for i in range(ilosc_probek):
            punkt = lista[i]
            Xc = int(punkt[0] * 10)
            Yc = int(punkt[1] * 10)

            if (self.map[Yc][Xc] == 100):
                in_free_space = False
                return in_free_space

        in_free_space = True
        return in_free_space



    def search(self,):
        print("PRM")

        self.parent[self.start] = None
        path = []

        # number of samples
        n = 200
        prm.sampling(n)

        # show points on map
        self.publish_points()


        r = 0.50
        prm.find_closest(r)
        self.prm_publish_connections()

        while not rp.is_shutdown():
            # Search for neighbors only in a limited neighborhood, radius r

            rp.sleep(0.50)
            pass


if __name__ == '__main__':
    prm = PRM()
    prm.search()

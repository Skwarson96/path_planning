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
        for point1 in self.points_list:
            for point2 in self.points_list:
                distance = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
                if (distance <= r) and (point1 != point2):
                    print("point1", point1, "point2", point2, "distance", distance)
                    self.prm_points_to_connection.append((point1,point2))

        # print(self.parent)



    def search(self,):
        print("PRM")

        self.parent[self.start] = None
        path = []

        # number of samples
        n = 30
        prm.sampling(n)
        # print(self.start)

        # show points on map
        self.publish_points()


        r = 1.0
        prm.find_closest(r)
        self.prm_publish_connections()
        while not rp.is_shutdown():
            # Search for neighbors only in a limited neighborhood, radius r

            rp.sleep(0.50)
            pass


if __name__ == '__main__':
    prm = PRM()
    prm.search()

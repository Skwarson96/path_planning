#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)
# Probabilistic Road Map
class PRM(GridMap):
    def __init__(self):
        super(PRM, self).__init__()

    def sampling(self, n):
        # print("Mapa: ", np.shape(self.map))
        # print(self.map[0][0])
        # print("Szerokosc: ", self.width)
        # print("Wysokosc: ", self.height)

        for i in range(n):
            x = self.width * np.random.random(1)
            y = self.height * np.random.random(1)
            # print(x[0], y[0])
            self.points_list.append((x[0], y[0]))


    def search(self,):
        print("PRM")

        # number of samples
        n = 300

        prm.sampling(n)
        # show points on map
        self.publish_points()
        while not rp.is_shutdown():

            pass


if __name__ == '__main__':
    prm = PRM()
    prm.search()

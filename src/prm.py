#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)

class PRM(GridMap):
    def __init__(self):
        super(PRM, self).__init__()

   

    def search(self,):



if __name__ == '__main__':
    prm = PRM()
    prm.search()

#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np
import sys



p.random.seed(444)

class RRT_star(GridMap):
    def __init__(self):
        super(RRT_star, self).__init__()

    def search(self):



        while not rp.is_shutdown():

            rp.sleep(0.10)




if __name__ == '__main__':
    rrt_star = RRT_star()
    rrt_star.search()

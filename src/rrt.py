#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)

class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
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
        #point = (x, y)

        return np.array([x, y])

    def find_closest(self, pos):
        """
        Finds the closest point in the graph (closest vertex or closes point on edge) to the pos argument
        If the point is on the edge, modifies the graph to obtain the valid graph with the new point and two new edges
        connecting existing vertices

        :param pos: point id 2D
        :return: point from graph in 2D closest to the pos
        """


        #print("POINT: ", pos)
        min = 1000 # duza wartosc
        x = 0
        y = 0
        for i, wartosc in enumerate(self.parent):
            odleglosc = np.sqrt((pos[0] - wartosc[0])**2 + (pos[1] - wartosc[1])**2)

            if (min > odleglosc):
                min = odleglosc
                x = wartosc[0]
                y = wartosc[1]

        x = round(x, 4)
        y = round(y, 4)
        #print("CLOSEST POINT: ", closest)
        closest = (x, y)

        return closest

    def new_pt(self, pt, closest):
        """
        Finds last point in the free space on the segment connecting closest with pt

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        return pt


    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        self.parent[self.start] = None

        while not rp.is_shutdown():

            randomPoint = rrt.random_point()
            closestPoint = rrt.find_closest(randomPoint)
            print(randomPoint, closestPoint)
            self.parent[(randomPoint[0], randomPoint[1])] = closestPoint

            rp.sleep(1.0)

if __name__ == '__main__':
    rrt = RRT()
    rrt.search()

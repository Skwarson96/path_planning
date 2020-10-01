#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np
import sys

np.random.seed(444)

# Probabilistic Road Map
class PRM(GridMap):
    def __init__(self):
        super(PRM, self).__init__()
        self.radius = 2.0
        self.k = 6
        self.number_of_points = 100

    def sampling(self,):
        """
        Draws a given number of points
        """

        for i in range(self.number_of_points):
            x = self.width * np.random.random(1)
            y = self.height * np.random.random(1)
            # print(x[0], y[0])

            Xc = int(x[0] * 10)
            Yc = int(y[0] * 10)
            if (self.map[Yc][Xc] != 100):
                self.points_list.append((x[0], y[0]))

    def find_closest_in_radius(self,):
        """
        Finds points within a given radius
        """

        distance_list = []
        for index1, point1 in enumerate(self.points_list):
            for index2, point2 in enumerate(self.points_list):
                distance = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

                # checking if the point is in range
                # does the point not equal itself
                # there is no obstacle between the points
                # whether the distance between points was not repeated
                # (prevents the same points from connecting a second time
                if (distance <= self.radius) and \
                        (point1 != point2) and \
                        prm.check_if_valid(point1, point2) and \
                        (distance not in distance_list):
                    self.prm_points_to_connection.append((point1,point2))
                distance_list.append(distance)

    def find_k_nearest(self,):
        """
        Finds the k nearest points
        """

        distance_list_with_points = []
        checked_points = []
        for index1, point1 in enumerate(self.points_list):
            for index2, point2 in enumerate(self.points_list):
                distance = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
                if distance != 0 and prm.check_if_valid(point1, point2) and point2 not in checked_points:
                    distance_list_with_points.append((distance, point1, point2))
                    checked_points.append(point1)
            if len(distance_list_with_points) > self.k-1:
                distance_list_with_points = (sorted(distance_list_with_points))
                for i in range(self.k):
                    self.prm_points_to_connection.append((distance_list_with_points[i][1],distance_list_with_points[i][2]))

            distance_list_with_points = []

    def check_if_valid(self, point1, point2):
        """
        Checks if the segment connecting a and b lies in the free space.

        :param point1: point in 2D
        :param point2: point in 2D
        :return: boolean
        """
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

    def closest_start(self):
        """
        Find closest points to start point
        """
        for point1 in self.points_list:
            distance = np.sqrt((point1[0] - self.start[0]) ** 2 + (point1[1] - self.start[1]) ** 2)
            if (distance <= self.radius) and \
                    (point1 != self.start) and \
                    prm.check_if_valid(point1, self.start):
                self.prm_points_to_connection.append((point1, self.start))

    def closest_end(self):
        """
        Find closest points to end point
        """
        for point1 in self.points_list:
            distance = np.sqrt((point1[0] - self.end[0]) ** 2 + (point1[1] - self.end[1]) ** 2)
            if (distance <= self.radius) and \
                    (point1 != self.end) and \
                    prm.check_if_valid(point1, self.end):
                self.prm_points_to_connection.append((point1, self.end))

    def get_neighbours(self, current_point):
        """
        Make a list of all neighbors of current_point in the graph

        :param current_point: point in 2D
        :return: list of neighbors
        """

        neighbours = []
        for point1, point2 in self.prm_points_to_connection:
            if current_point == point1:
                neighbours.append(point2)
            if current_point == point2:
                neighbours.append(point1)
        return neighbours

    def find_shortest_path(self):
        """
        Finds the shortest path using the A * algorithm
        """

        G = {}
        F = {}
        G[self.start] = 0
        F[self.start] = np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2)
        closed_points = set()
        open_points = set([self.start])
        came_from = {}

        while len(open_points) > 0:
            current_point = None
            current_F = None
            for point in open_points:
                if current_point is None or F[point] < current_F:
                    current_F = F[point]
                    current_point = point

            if current_point == self.end:
                path = [current_point]
                while current_point in came_from:
                    current_point = came_from[current_point]
                    path.append(current_point)
                path.reverse()

                return path, F[self.end]

            open_points.remove(current_point)
            closed_points.add(current_point)

            for neighbour in prm.get_neighbours(current_point):
                if neighbour in closed_points:
                    continue
                candidate_G = G[current_point] + np.sqrt((current_point[0] - neighbour[0]) ** 2 + (current_point[1] - neighbour[1]) ** 2)
                if neighbour not in open_points:
                    open_points.add(neighbour)
                elif candidate_G >= G[neighbour]:
                    continue
                came_from[neighbour] = current_point
                G[neighbour] = candidate_G
                H = np.sqrt((neighbour[0] - self.end[0]) ** 2 + (neighbour[1] - self.end[1]) ** 2)
                F[neighbour] = G[neighbour] + H

    def search(self,):
        """
        PRM search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_points() to publish points, self.prm_publish_connections() to publish the search tree
        and self.publish_path(path) to publish path.
        The algorithm has two possibilities, it can search for neighbors within a given radius or for the k nearest neighbors
        """

        print("PRM")

        self.parent[self.start] = None

        prm.sampling()

        # show points on map
        self.publish_points()
        rp.sleep(5.0)

        # find closest points
        # prm.find_closest_in_radius()
        # k nearest points
        prm.find_k_nearest()

        # add start and end points to list
        prm.closest_start()
        prm.closest_end()

        # draw lines between points in radius
        self.prm_publish_connections()
        rp.sleep(5.0)

        # A star algorithm to find shortest path
        path = prm.find_shortest_path()

        self.publish_path(path[0])
        print("Path printed")

if __name__ == '__main__':
    prm = PRM()
    prm.search()






#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np

np.random.seed(444)
# Probabilistic Road Map
class PRM(GridMap):
    def __init__(self):
        super(PRM, self).__init__()
        self.radius = 1.0

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

    def find_closest(self,):
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

    # find closest points to start point
    def closest_start(self):
        for point1 in self.points_list:
            distance = np.sqrt((point1[0] - self.start[0]) ** 2 + (point1[1] - self.start[1]) ** 2)
            if (distance <= self.radius) and \
                    (point1 != self.start) and \
                    prm.check_if_valid(point1, self.start):
                self.prm_points_to_connection.append((point1, self.start))

    # find closest points to end point
    def closest_end(self):
        for point1 in self.points_list:
            distance = np.sqrt((point1[0] - self.end[0]) ** 2 + (point1[1] - self.end[1]) ** 2)
            if (distance <= self.radius) and \
                    (point1 != self.end) and \
                    prm.check_if_valid(point1, self.end):
                self.prm_points_to_connection.append((point1, self.end))


    def get_neighbours(self, current_point):
        neighbours = []
        for point1, point2 in self.prm_points_to_connection:
            if current_point == point1:
                neighbours.append(point2)
            if current_point == point2:
                neighbours.append(point1)
        return neighbours

    def find_shortest_path(self):

        G = {}
        F = {}
        G[self.start] = 0
        F[self.start] = np.sqrt((self.start[0] - self.end[0]) ** 2 + (self.start[1] - self.end[1]) ** 2)
        closed_pionts = set()
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
            closed_pionts.add(current_point)


            for neighbour in prm.get_neighbours(current_point):
                # print("test3")
                if neighbour in closed_pionts:
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
        print("PRM")

        self.parent[self.start] = None

        # number of samples
        n = 100
        prm.sampling(n)

        # show points on map
        self.publish_points()


        # find closest points
        prm.find_closest()

        # add start and end points to list
        prm.closest_start()
        prm.closest_end()

        # draw lines between points in radius
        self.prm_publish_connections()


        # A star algorithm to find shortest path
        path = prm.find_shortest_path()

        self.publish_path(path[0])
        print("Path printed")


if __name__ == '__main__':
    prm = PRM()
    prm.search()

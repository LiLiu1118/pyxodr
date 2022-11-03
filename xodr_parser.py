import xml.etree.ElementTree as ET
from roadParser import RoadParser
import numpy as np
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
import math

class XODRParser:

    def __init__(self, file_path):
        self.file_path = file_path
        self.tree = ET.parse(self.file_path)
        self.root = self.tree.getroot()
        self.roads = {}

    def find_id(self, id):
        if (id in self.roads):
            return self.roads[id]
        else:
            roads = self.root.findall('.//road')
            for road in roads:
                id_temp = road.get('id')
                if (id_temp == id):
                    road_founded = RoadParser(road)
                    self.roads[id] = road_founded
                    return road_founded # return a parser of road

        return None

    def find_road_from_gps(self, gps):
        roads = self.root.findall('.//road')
        # point = Point(455256.7, 4724715.08)
        # point = Point(455268.82, 5724720.70)
        point = Point(gps[0], gps[1])
        for road in roads:
            road_parser = RoadParser(road)
            road_parser.parse_geometry_parameters()
            road_parser.calculate_3d_coordinate()
            keys_positive = list(np.arange(1, road_parser.max_lane_id+1))
            keys_negative = list(np.arange(road_parser.min_lane_id, 0))
            keys_positive.reverse()
            right_boundary = []
            left_boundary = []

            if len(keys_negative) > 0:
                for i in range(len(road_parser.minus_coordiantes[str(road_parser.min_lane_id)])):
                    flag_right = 0
                    for j in keys_negative:
                        if not math.isnan(road_parser.minus_coordiantes[str(j)][i][0]):
                            right_boundary.append(road_parser.minus_coordiantes[str(j)][i])
                            flag_right = 1
                            break
                    if flag_right == 0:
                        right_boundary.append(road_parser.center_line_coordinates[i])

            if len(keys_positive) > 0:
                for i in range(len(road_parser.minus_coordiantes[str(road_parser.max_lane_id)])):
                    flag_left = 0
                    for j in keys_positive:
                        if not math.isnan(road_parser.minus_coordiantes[str(j)][i][0]):
                            left_boundary.append(road_parser.minus_coordiantes[str(j)][i])
                            flag_left = 1
                            break
                    if flag_left == 0:
                        left_boundary.append(road_parser.center_line_coordinates[i])

            if len(right_boundary) == 0 and len(left_boundary) == 0:
                continue

            center = np.array(road_parser.center_line_coordinates)[:,0:2]
            if len(right_boundary) > 0:
                coors_right = np.array(right_boundary)[:, 0:2]
                right_boundary1 = np.vstack((coors_right[:, 0:2][::-1], center))
                right_boundary = Polygon(right_boundary1)
                if right_boundary.contains(point):
                    return True, road_parser.id, -1, road_parser

            if len(left_boundary) > 0:
                coors_left = np.array(left_boundary)[:, 0:2]
                left_boundary1 = np.vstack((coors_left[:, 0:2][::-1], center))
                left_boundary = Polygon(left_boundary1)
                if left_boundary.contains(point):
                    return True, road_parser.id, 1, road_parser
        return False, '0', 0, None

    # The parameters of road_parser in function below are expected to be
    # already calculated in function find_road_from_gps
    def calculate_projection_geometry(self, road_parser, gps, lane, distance):
        ref_line = np.array(road_parser.ref_line_coordinates)[:,0:2]
        # print(center_line[:,0:2]-gps)
        index = np.argmin(np.linalg.norm(ref_line-gps, axis=1))
        if lane == -1:
            projection = ref_line[index:index + distance * 10] - (ref_line[index] - gps)
        elif lane == 1:
            projection = ref_line[index:index - distance * 10:-1] - (ref_line[index] - gps)

        return projection


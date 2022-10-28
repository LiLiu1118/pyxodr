import math
import time
import numpy as np
from road import Road
from lanes import *
from choose_parameters import *
from transformation import rotate
import matplotlib.pyplot as plt
from plot_adjustment import axisEqual3D
from planView import PlanView, Geometry, ParamPoly3
from elevationProfile import Elevation, ElevationProfile

class RoadParser:

    def __init__(self, road_node):
        self.road_node = road_node
        self.geometry_nodes = []
        self.id = float(road_node.get('id'))
        self.road = None
        self.already_gathered = False
        self.road_length = float(self.road_node.get("length"))
        self.s_coordinates = np.linspace(0, self.road_length, num=int(self.road_length * 10),
                                         endpoint=True, retstep=False, dtype=None, axis=0)
        self.ref_line_coordinates = []
        self.center_line_coordinates = []
        self.minus_coordiantes = {}
        self.laneoffset_s = []
        self.lane_section_s = []
        self.geometry_s = []
        self.elevation_s = []
        self.u_lane = {"0": []}
        self.v_lane = {"0": []}
        self.max_lane_id = 0
        self.min_lane_id = 0

    def get_road_length(self):

        return float(self.road_node.get("length"))

    def parse_geometry_parameters(self):
        if self.already_gathered:
            return
        if self.road_node is None:
            print("Road node is None!")
            raise TypeError

        # save information about plan_view, elevation, id and length
        plan_view = PlanView()
        geometry_nodes = self.road_node.findall('.//geometry')
        self.geometry_nodes = geometry_nodes
        for geometry_node in geometry_nodes:
            s = float(geometry_node.get('s'))
            x = float(geometry_node.get('x'))
            y = float(geometry_node.get('y'))
            hdg = float(geometry_node.get('hdg'))
            length = float(geometry_node.get('length'))
            geometry = Geometry(s, x, y, hdg, length)
            self.geometry_s.append(s)

            if (geometry_node[0].tag == "paramPoly3"):
                poly_params = geometry_node.findall('.//paramPoly3')
                for poly_param in poly_params:
                    aU = float(poly_param.get('aU'))
                    bU = float(poly_param.get('bU'))
                    cU = float(poly_param.get('cU'))
                    dU = float(poly_param.get('dU'))
                    aV = float(poly_param.get('aV'))
                    bV = float(poly_param.get('bV'))
                    cV = float(poly_param.get('cV'))
                    dV = float(poly_param.get('dV'))
                    paramPoly3 = ParamPoly3(aU, bU, cU, dU, aV, bV, cV, dV)
                    geometry.model = paramPoly3
                plan_view.geometrys.append(geometry)
        road = Road()
        road.length = self.road_length
        road.id = float(self.road_node.get("id"))
        road.planView = plan_view
        elevationProfile = ElevationProfile()
        elevation_nodes = self.road_node.findall('.//elevation')

        for elevation_node in elevation_nodes:
            s = float(elevation_node.get('s'))
            a = float(elevation_node.get('a'))
            b = float(elevation_node.get('b'))
            c = float(elevation_node.get('c'))
            d = float(elevation_node.get('d'))
            elevation = Elevation(s, a, b, c, d)
            elevationProfile.elevations.append(elevation)
            self.elevation_s.append(s)
        road.elevationProfile = elevationProfile
        self.road = road

        # save information about lanes
        laneOffset_nodes = self.road_node.findall('.//laneOffset')
        lanes = Lanes()
        for laneOffset_node in laneOffset_nodes:
            s = float(laneOffset_node.get('s'))
            a = float(laneOffset_node.get('a'))
            b = float(laneOffset_node.get('b'))
            c = float(laneOffset_node.get('c'))
            d = float(laneOffset_node.get('d'))
            self.laneoffset_s.append(s)
            lane_offset = LaneOffset(s, a, b, c, d)
            lanes.laneOffsets.append(lane_offset)


        laneSection_nodes = self.road_node.findall('.//laneSection')
        for laneSection_node in laneSection_nodes:
            s = float(laneSection_node.get('s'))
            lane_section = LaneSection(s)
            self.lane_section_s.append(s)
            lanes_all_num_node = laneSection_node.findall('.//lane')
            for lane_all_num_node in lanes_all_num_node:
                lane_id = lane_all_num_node.get("id")
                if int(lane_id) > self.max_lane_id:
                    self.max_lane_id = int(lane_id)
                if int(lane_id) < self.min_lane_id:
                    self.min_lane_id = int(lane_id)
                lane = Lane(float(lane_id))
                width_items_node = lane_all_num_node.findall('.//width')
                for width_item_node in width_items_node:
                    soffset = float(width_item_node.get('sOffset'))
                    a = float(width_item_node.get('a'))
                    b = float(width_item_node.get('b'))
                    c = float(width_item_node.get('c'))
                    d = float(width_item_node.get('d'))
                    width_item = Width_item(soffset, a, b, c, d)
                    lane.width_items.append(width_item)
                lane_section.lane_with_id[lane_id] = lane
            lanes.laneSections.append(lane_section)
        self.road.lanes = lanes
        self.already_gathered = True

    def calculate_3d_coordinate(self):
        coor_index = 0
        for s_coordinate in self.s_coordinates:
            s_xy, x, y, hdg, length, geometry_index = choose_ref_line_parameters(self.road, s_coordinate,
                                                                                 self.geometry_s)

            s_z, a, b, c, d = choose_elevation_parameters(self.road, s_coordinate, self.elevation_s)

            s_lane_offset, a_lane_offset, b_lane_offset, c_lane_offset, d_lane_offset = choose_lane_offset_parameters(
                self.road, s_coordinate, self.laneoffset_s)

            if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                gradient_u = self.road.planView.geometrys[geometry_index].model.bU + 2 *\
                    self.road.planView.geometrys[geometry_index].model.cU * (s_coordinate - s_xy) + 3 *\
                    self.road.planView.geometrys[geometry_index].model.dU * (s_coordinate - s_xy) ** 2
                gradient_v = self.road.planView.geometrys[geometry_index].model.bV + 2 *\
                    self.road.planView.geometrys[geometry_index].model.cV * (s_coordinate - s_xy) + 3 *\
                    self.road.planView.geometrys[geometry_index].model.dV * (s_coordinate - s_xy) ** 2

                hdg_this_s = math.atan2(gradient_v, gradient_u)
                u = self.road.planView.geometrys[geometry_index].model.aU + \
                    self.road.planView.geometrys[geometry_index].model.bU * (s_coordinate - s_xy) +\
                    self.road.planView.geometrys[geometry_index].model.cU * (s_coordinate - s_xy) ** 2 +\
                    self.road.planView.geometrys[geometry_index].model.dU * (s_coordinate - s_xy) ** 3
                v = self.road.planView.geometrys[geometry_index].model.aV +\
                    self.road.planView.geometrys[geometry_index].model.bV * (s_coordinate - s_xy) +\
                    self.road.planView.geometrys[geometry_index].model.cV * (s_coordinate - s_xy) ** 2 +\
                    self.road.planView.geometrys[geometry_index].model.dV * (s_coordinate - s_xy) ** 3
                offset = a_lane_offset + b_lane_offset * (s_coordinate - s_lane_offset) +\
                    c_lane_offset * (s_coordinate - s_lane_offset) ** 2 + d_lane_offset * (
                                 s_coordinate - s_lane_offset) ** 3
                u_center = u - offset * math.sin(hdg_this_s)
                v_center = v + offset * math.cos(hdg_this_s)
                self.u_lane['0'].append(u_center)
                self.v_lane['0'].append(v_center)


                coordinates_before_rotate = [u, v]
                middle_coordinates_before_rotate = [u_center, v_center]
                # minus1_coordinates_before_rotate = [u_minus1, v_minus1]

                coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])
                middle_coordinates_after_rotate = rotate(middle_coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])
                # minus1_coordinates_after_rotate = rotate(minus1_coordinates_before_rotate, hdg, rotation_around=[
                #     self.road.planView.geometrys[geometry_index].model.aU,
                #     self.road.planView.geometrys[geometry_index].model.aV])

                coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
                middle_coordinates_after_rotate_and_translation = middle_coordinates_after_rotate + np.array([x, y])
                # minus1_coordinates_after_rotate_and_translation = minus1_coordinates_after_rotate + np.array([x, y])

                x_coor = coordinates_after_rotate_and_translation[0]
                y_coor = coordinates_after_rotate_and_translation[1]

                middle_x_coor = middle_coordinates_after_rotate_and_translation[0]
                middle_y_coor = middle_coordinates_after_rotate_and_translation[1]

                # minus1_x_coor = minus1_coordinates_after_rotate_and_translation[0]
                # minus1_y_coor = minus1_coordinates_after_rotate_and_translation[1]
                z_coor = a + b * (s_coordinate - s_z) + c * (s_coordinate - s_z) ** 2 + d * (s_coordinate - s_z) ** 3
                self.ref_line_coordinates.append([x_coor, y_coor, z_coor])
                self.center_line_coordinates.append([middle_x_coor, middle_y_coor, z_coor])

            f1 = list(filter(lambda x: x <= s_coordinate, self.lane_section_s))
            index_lanesection = len(f1) - 1

            keys = list(self.road.lanes.laneSections[index_lanesection].lane_with_id.keys())
            keys = list(map(int, keys))
            keys_negativ = [i for i in keys if i < 0]
            keys_positive = [i for i in keys if i > 0]

            keys_positive = list(reversed(keys_positive))
            for id in keys_negativ:
                s_offset, width_a, width_b, width_c, width_d = choose_width_parameters(self.road, s_coordinate, index_lanesection, str(id))
                width = width_a + width_b * s_offset + width_c * s_offset ** 2 +\
                    width_d * s_offset ** 3

                u_minus = self.u_lane[str(id + 1)][-1] + width * math.sin(hdg_this_s)
                v_minus = self.v_lane[str(id + 1)][-1] - width * math.cos(hdg_this_s)
                if str(id) not in self.u_lane:
                    self.u_lane[str(id)] = []
                    self.v_lane[str(id)] = []
                self.u_lane[str(id)].append(u_minus)
                self.v_lane[str(id)].append(v_minus)
                minus_coordinates_before_rotate = [u_minus, v_minus]
                minus_coordinates_after_rotate = rotate(minus_coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])
                minus_coordinates_after_rotate_and_translation = minus_coordinates_after_rotate + np.array([x, y])
                minus_x_coor = minus_coordinates_after_rotate_and_translation[0]
                minus_y_coor = minus_coordinates_after_rotate_and_translation[1]

                if str(id) not in self.minus_coordiantes:
                    self.minus_coordiantes[str(id)] = []
                self.minus_coordiantes[str(id)].append([minus_x_coor, minus_y_coor, z_coor])

            if self.min_lane_id < min(keys_negativ):
                for i in range(self.min_lane_id, min(keys_negativ)):
                    if str(i) not in self.u_lane:
                        self.u_lane[str(i)] = []
                        self.v_lane[str(i)] = []
                    self.u_lane[str(i)].append(float('NaN'))
                    self.v_lane[str(i)].append(float('NaN'))
                    if str(i) not in self.minus_coordiantes:
                        self.minus_coordiantes[str(i)] = []
                    self.minus_coordiantes[str(i)].append([float('NaN'), float('NaN'), float('NaN')])


            for id in keys_positive:
                s_offset, width_a, width_b, width_c, width_d = choose_width_parameters(self.road, s_coordinate, index_lanesection, str(id))
                width = width_a + width_b * s_offset + width_c * s_offset ** 2 +\
                    width_d * s_offset ** 3

                u_minus = self.u_lane[str(id - 1)][-1] - width * math.sin(hdg_this_s)
                v_minus = self.v_lane[str(id - 1)][-1] + width * math.cos(hdg_this_s)
                if str(id) not in self.u_lane:
                    self.u_lane[str(id)] = []
                    self.v_lane[str(id)] = []
                self.u_lane[str(id)].append(u_minus)
                self.v_lane[str(id)].append(v_minus)
                minus_coordinates_before_rotate = [u_minus, v_minus]
                minus_coordinates_after_rotate = rotate(minus_coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])
                minus_coordinates_after_rotate_and_translation = minus_coordinates_after_rotate + np.array([x, y])
                minus_x_coor = minus_coordinates_after_rotate_and_translation[0]
                minus_y_coor = minus_coordinates_after_rotate_and_translation[1]

                if str(id) not in self.minus_coordiantes:
                    self.minus_coordiantes[str(id)] = []
                self.minus_coordiantes[str(id)].append([minus_x_coor, minus_y_coor, z_coor])

            if self.max_lane_id > max(keys_positive):
                for i in range(max(keys_positive)+1, self.max_lane_id+1):
                    if str(i) not in self.u_lane:
                        self.u_lane[str(i)] = []
                        self.v_lane[str(i)] = []
                    self.u_lane[str(i)].append(float('NaN'))
                    self.v_lane[str(i)].append(float('NaN'))
                    if str(i) not in self.minus_coordiantes:
                        self.minus_coordiantes[str(i)] = []
                    self.minus_coordiantes[str(i)].append([float('NaN'), float('NaN'), float('NaN')])

    def plot_2D(self):
        time1 = time.time()
        if (len(self.s_coordinates) == 0):
            self.parse_geometry_parameters()

        if (len(self.ref_line_coordinates) == 0):
            self.calculate_3d_coordinate()

        plt.figure()
        plt.plot(np.array(self.ref_line_coordinates)[:, 0], np.array(self.ref_line_coordinates)[:, 1], 'r-.',
                 linewidth=1, label="Reference line")


        plt.plot(np.array(self.center_line_coordinates)[:, 0], np.array(self.center_line_coordinates)[:, 1],
                 linewidth=0.8, label="Center line")
        for i in self.minus_coordiantes.keys():

            plt.plot(np.array(self.minus_coordiantes[i])[:, 0], np.array(self.minus_coordiantes[i])[:, 1],
                     linewidth=0.8, label= "Lane " + i + " boundary")

        plt.axis('equal')
        plt.legend()
        plt.title("Road lanes visualization")
        time2 = time.time()
        print("Plot finished: Time used --- %s seconds ---" % (time2 - time1))
        plt.show()
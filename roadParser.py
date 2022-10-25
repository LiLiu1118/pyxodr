import numpy as np
from transformation import rotate
import matplotlib.pyplot as plt
from plot_adjustment import axisEqual3D
from planView import PlanView, Geometry, ParamPoly3
from elevationProfile import Elevation, ElevationProfile
from road import Road
from lanes import *
import math
import time
from choose_parameters import *

class RoadParser:

    def __init__(self, road_node):
        self.road_node = road_node
        self.geometry_nodes = []
        self.id = road_node.get('id')
        self.road = None
        self.already_gathered = False
        self.road_length = self.road_node.get("length")
        self.s_coordinates = np.linspace(0, float(self.road_length), num=int(float(self.road_length) * 100),
                                         endpoint=True,retstep=False, dtype=None, axis=0)
        self.ref_line_coordinates = []
        self.center_line_coordinates = []
        self.minus1_coordiantes = []



    def parse_geometry_parameters(self):
        if (self.already_gathered == True):
            return
        if (self.road_node == None):
            print("Road node is None!")
            raise TypeError

        # save information about plan_view, elevation, id and length
        plan_view = PlanView()
        geometry_nodes = self.road_node.findall('.//geometry')
        self.geometry_nodes = geometry_nodes
        for geometry_node in geometry_nodes:
            s = geometry_node.get('s')
            x = geometry_node.get('x')
            y = geometry_node.get('y')
            hdg = geometry_node.get('hdg')
            length = geometry_node.get('length')
            geometry = Geometry(s, x, y, hdg, length)

            if (geometry_node[0].tag == "paramPoly3"):
                poly_params = geometry_node.findall('.//paramPoly3')
                for poly_param in poly_params:
                    aU = poly_param.get('aU')
                    bU = poly_param.get('bU')
                    cU = poly_param.get('cU')
                    dU = poly_param.get('dU')
                    aV = poly_param.get('aV')
                    bV = poly_param.get('bV')
                    cV = poly_param.get('cV')
                    dV = poly_param.get('dV')
                    paramPoly3 = ParamPoly3(aU, bU, cU, dU, aV, bV, cV, dV)
                    geometry.model = paramPoly3
                plan_view.geometrys.append(geometry)
        road = Road()
        road.length = self.road_length
        road.id = self.road_node.get("id")
        road.planView = plan_view
        elevationProfile = ElevationProfile()
        elevation_nodes = self.road_node.findall('.//elevation')

        for elevation_node in elevation_nodes:
            s = elevation_node.get('s')
            a = elevation_node.get('a')
            b = elevation_node.get('b')
            c = elevation_node.get('c')
            d = elevation_node.get('d')
            elevation = Elevation(s, a, b, c, d)
            elevationProfile.elevations.append(elevation)
        road.elevationProfile = elevationProfile
        self.road = road


        # save information about lanes
        laneOffset_nodes = self.road_node.findall('.//laneOffset')
        lanes = Lanes()
        for laneOffset_node in laneOffset_nodes:
            s = laneOffset_node.get('s')
            a = laneOffset_node.get('a')
            b = laneOffset_node.get('b')
            c = laneOffset_node.get('c')
            d = laneOffset_node.get('d')
            lane_offset = LaneOffset(s, a, b, c, d)
            lanes.laneOffsets.append(lane_offset)

        laneSection_nodes = self.road_node.findall('.//laneSection')
        for laneSection_node in laneSection_nodes:
            s = laneSection_node.get('s')
            lane_section = LaneSection(s)
            lanes_all_num_node = laneSection_node.findall('.//lane')
            for lane_all_num_node in lanes_all_num_node:
                if lane_all_num_node.get("id") == '-1':
                    lane_minus1 = Lane_minus1()
                    width_items_node = lane_all_num_node.findall('.//width')
                    for width_item_node in width_items_node:
                        sOffset = width_item_node.get('sOffset')
                        a = width_item_node.get('a')
                        b = width_item_node.get('b')
                        c = width_item_node.get('c')
                        d = width_item_node.get('d')
                        width_item = Width_item(sOffset, a, b, c, d)
                        lane_minus1.width_items.append(width_item)
                    lane_section.lane_minus1 = lane_minus1

            lanes.laneSections.append(lane_section)
        self.road.lanes = lanes
        self.already_gathered = True


    def get_road_length(self):

        return self.road_node.get("length")


    def calculate_3d_coordinate(self):
        for s_coordinate in self.s_coordinates:
            s_xy = x = y = hdg = length = model = None
            s_z = a = b = c = d = None
            s_laneOffset = a_laneOffset = lanesections = b_laneOffset = c_laneOffset = d_laneOffset = None
            geometry_index = 0
            lane_section_s = s_Offset_1 = s_Offset = width_a = width_b = width_c = width_d = None

            s_Offset_1, width_a, width_b, width_c, width_d = choose_width_parameters(self.road, s_coordinate)

            s_laneOffset, a_laneOffset, b_laneOffset, c_laneOffset, d_laneOffset = choose_lane_offset_parameters(self.road, s_coordinate)

            s_xy, x, y, hdg, length, geometry_index = choose_ref_line_parameters(self.road, s_coordinate)

            s_z, a, b, c, d = choose_elevation_parameters(self.road, s_coordinate)

            if (self.geometry_nodes[geometry_index][0].tag == "paramPoly3"):
                gradient_u = float(self.road.planView.geometrys[geometry_index].model.bU) + 2*float(self.road.planView.geometrys[geometry_index].model.cU) * (s_coordinate - s_xy) + 3*float(self.road.planView.geometrys[geometry_index].model.dU) * (s_coordinate - s_xy) ** 2
                gradient_v = float(self.road.planView.geometrys[geometry_index].model.bV) + 2*float(self.road.planView.geometrys[geometry_index].model.cV) * (s_coordinate - s_xy) + 3*float(self.road.planView.geometrys[geometry_index].model.dV) * (s_coordinate - s_xy) ** 2

                hdg_this_s = math.atan2(gradient_v, gradient_u) #angle in radian
                # math.tan(3.1415926 / 2)
                u = float(self.road.planView.geometrys[geometry_index].model.aU) + float(self.road.planView.geometrys[geometry_index].model.bU) * (s_coordinate - s_xy) + float(self.road.planView.geometrys[geometry_index].model.cU) * (s_coordinate - s_xy) ** 2 + float(self.road.planView.geometrys[geometry_index].model.dU) * (s_coordinate - s_xy) ** 3
                v = float(self.road.planView.geometrys[geometry_index].model.aV) + float(self.road.planView.geometrys[geometry_index].model.bV) * (s_coordinate - s_xy) + float(self.road.planView.geometrys[geometry_index].model.cV) * (s_coordinate - s_xy) ** 2 + float(self.road.planView.geometrys[geometry_index].model.dV) * (s_coordinate - s_xy) ** 3
                offset = float(a_laneOffset) + float(b_laneOffset) * (s_coordinate - s_laneOffset) + float(c_laneOffset) * (s_coordinate - s_laneOffset)**2 + float(d_laneOffset) * (s_coordinate - s_laneOffset)**3
                u_center = u - offset * math.sin(hdg_this_s)
                v_center = v + offset * math.cos(hdg_this_s)
                width = float(width_a) + float(width_b) * s_Offset_1 + float(width_c) * s_Offset_1**2 + float(width_d) * s_Offset_1**3
                u_minus1 = u_center + (offset+width) * math.sin(hdg_this_s)
                v_minus1 = v_center - (offset+width) * math.cos(hdg_this_s)

                coordinates_before_rotate = np.vstack((u, v)).T
                middle_coordinates_before_rotate = np.vstack((u_center, v_center)).T
                minus1_coordinates_before_rotate = np.vstack((u_minus1, v_minus1)).T

                coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[
                    float(self.road.planView.geometrys[geometry_index].model.aU),
                    float(self.road.planView.geometrys[geometry_index].model.aV)])
                middle_coordinates_after_rotate = rotate(middle_coordinates_before_rotate, hdg, rotation_around=[
                    float(self.road.planView.geometrys[geometry_index].model.aU),
                    float(self.road.planView.geometrys[geometry_index].model.aV)])
                minus1_coordinates_after_rotate = rotate(minus1_coordinates_before_rotate, hdg, rotation_around=[
                    float(self.road.planView.geometrys[geometry_index].model.aU),
                    float(self.road.planView.geometrys[geometry_index].model.aV)])

                coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
                middle_coordinates_after_rotate_and_translation = middle_coordinates_after_rotate + np.array([x, y])
                minus1_coordinates_after_rotate_and_translation = minus1_coordinates_after_rotate + np.array([x, y])

                x_coor = coordinates_after_rotate_and_translation[0]
                y_coor = coordinates_after_rotate_and_translation[1]

                middle_x_coor = middle_coordinates_after_rotate_and_translation[0]
                middle_y_coor = middle_coordinates_after_rotate_and_translation[1]

                minus1_x_coor = minus1_coordinates_after_rotate_and_translation[0]
                minus1_y_coor = minus1_coordinates_after_rotate_and_translation[1]
            z_coor = a + b * (s_coordinate - s_z) + c * (s_coordinate - s_z) ** 2 + d * (s_coordinate - s_z) ** 3
            self.ref_line_coordinates.append([x_coor, y_coor, z_coor])
            self.center_line_coordinates.append([middle_x_coor, middle_y_coor, z_coor])
            self.minus1_coordiantes.append([minus1_x_coor, minus1_y_coor, z_coor])



    def plot_2D(self):
        time1 = time.time()
        if (len(self.s_coordinates) == 0):
            self.parse_geometry_parameters()

        if (len(self.ref_line_coordinates) == 0):
            self.calculate_3d_coordinate()

        plt.figure()
        plt.plot(np.array(self.ref_line_coordinates)[:,0], np.array(self.ref_line_coordinates)[:,1], 'r',
                 label="Reference line")
        plt.plot(np.array(self.center_line_coordinates)[:, 0], np.array(self.center_line_coordinates)[:, 1], 'b')
        plt.plot(np.array(self.minus1_coordiantes)[:, 0], np.array(self.minus1_coordiantes)[:, 1], 'b',
                 label="Lane boundary")

        plt.axis('equal')
        plt.legend()
        plt.title("Driving lane coordinates")
        time2 = time.time()
        print("Plot finished: Time used --- %s seconds ---" % (time2 - time1))
        plt.show()


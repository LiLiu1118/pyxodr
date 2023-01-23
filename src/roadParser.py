import math
import time
import numpy as np
from road import Road
from lanes import *
from object import Object
from choose_parameters import *
from transformation import rotate
import matplotlib.pyplot as plt
from plot_adjustment import axisEqual3D
from planView import PlanView, Geometry, ParamPoly3, Line
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
        self.lanes_coordinates = {}
        self.lane_offset_s = []
        self.lane_section_s = []
        self.geometry_s = []
        self.elevation_s = []
        self.u_lane = {"0": []}
        self.v_lane = {"0": []}
        self.max_lane_id = 0
        self.min_lane_id = 0
        self.keys_negative = None
        self.keys_positive = None
        self.obj_coordinates = []
        self.delineator = []

    def get_driving_lane_id(self):
        lane_section_nodes = self.road_node.findall('.//laneSection')
        for lane_section_node in lane_section_nodes:
            s = float(lane_section_node.get('s'))
            lane_section = LaneSection(s)
            self.lane_section_s.append(s)
            lanes_all_num_node = lane_section_node.findall('.//lane')
            for lane_all_num_node in lanes_all_num_node:
                lane_id = lane_all_num_node.get("id")

    def get_road_length(self):
        return float(self.road_node.get("length"))

    # parse road lane geometry
    def parse_lanes_parameters(self):
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

            if geometry_node[0].tag == "paramPoly3":
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
            elif geometry_node[0].tag == "line":
                line_params = geometry_node.findall('.//line')
                for line_param in line_params:
                    line = Line()
                    geometry.model = line
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
            self.lane_offset_s.append(s)
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

    def parse_object_parameters(self):
        if self.road_node is None:
            print("Road node is None!")
            raise TypeError

        object_nodes = self.road_node.findall('.//object')
        for object_node in object_nodes:
            object_type = object_node.get('type')
            object_subtype = object_node.get('subtype')
            object_id = float(object_node.get('id'))
            s = float(object_node.get('s'))
            t = float(object_node.get('t'))
            object_new = Object(object_type, object_subtype, object_id, s, t)
            self.road.objects.append(object_new)

    def calculate_lanes_coordinate(self):
        for s_coordinate in self.s_coordinates:
            s_xy, x, y, hdg, length, geometry_index = choose_ref_line_parameters(self.road, s_coordinate,
                                                                                 self.geometry_s)

            s_z, a, b, c, d = choose_elevation_parameters(self.road, s_coordinate, self.elevation_s)

            s_lane_offset, a_lane_offset, b_lane_offset, c_lane_offset, d_lane_offset = choose_lane_offset_parameters(
                self.road, s_coordinate, self.lane_offset_s)

            if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                gradient_u = self.road.planView.geometrys[geometry_index].model.bU + 2 * \
                             self.road.planView.geometrys[geometry_index].model.cU * (s_coordinate - s_xy) + 3 * \
                             self.road.planView.geometrys[geometry_index].model.dU * (s_coordinate - s_xy) ** 2
                gradient_v = self.road.planView.geometrys[geometry_index].model.bV + 2 * \
                             self.road.planView.geometrys[geometry_index].model.cV * (s_coordinate - s_xy) + 3 * \
                             self.road.planView.geometrys[geometry_index].model.dV * (s_coordinate - s_xy) ** 2

                hdg_this_s = math.atan2(gradient_v, gradient_u)
                u = self.road.planView.geometrys[geometry_index].model.aU + \
                    self.road.planView.geometrys[geometry_index].model.bU * (s_coordinate - s_xy) + \
                    self.road.planView.geometrys[geometry_index].model.cU * (s_coordinate - s_xy) ** 2 + \
                    self.road.planView.geometrys[geometry_index].model.dU * (s_coordinate - s_xy) ** 3
                v = self.road.planView.geometrys[geometry_index].model.aV + \
                    self.road.planView.geometrys[geometry_index].model.bV * (s_coordinate - s_xy) + \
                    self.road.planView.geometrys[geometry_index].model.cV * (s_coordinate - s_xy) ** 2 + \
                    self.road.planView.geometrys[geometry_index].model.dV * (s_coordinate - s_xy) ** 3
                offset = a_lane_offset + b_lane_offset * (s_coordinate - s_lane_offset) + \
                    c_lane_offset * (s_coordinate - s_lane_offset) ** 2 + d_lane_offset * (
                    s_coordinate - s_lane_offset) ** 3
                u_center = u - offset * math.sin(hdg_this_s)
                v_center = v + offset * math.cos(hdg_this_s)
                self.u_lane['0'].append(u_center)
                self.v_lane['0'].append(v_center)

                coordinates_before_rotate = [u, v]
                middle_coordinates_before_rotate = [u_center, v_center]

                coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])
                middle_coordinates_after_rotate = rotate(middle_coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])

                coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
                middle_coordinates_after_rotate_and_translation = middle_coordinates_after_rotate + np.array([x, y])

                x_coordinate = coordinates_after_rotate_and_translation[0]
                y_coordinate = coordinates_after_rotate_and_translation[1]

                middle_x_coordinate = middle_coordinates_after_rotate_and_translation[0]
                middle_y_coordinate = middle_coordinates_after_rotate_and_translation[1]

                z_coordinate = a + b * (s_coordinate - s_z) + c * (s_coordinate - s_z) ** 2 + d * (s_coordinate - s_z) ** 3
                self.ref_line_coordinates.append([x_coordinate, y_coordinate, z_coordinate])
                self.center_line_coordinates.append([middle_x_coordinate, middle_y_coordinate, z_coordinate])

            if self.geometry_nodes[geometry_index][0].tag == "line":
                u = s_coordinate - s_xy
                v = 0
                offset = a_lane_offset + b_lane_offset * (s_coordinate - s_lane_offset) + \
                         c_lane_offset * (s_coordinate - s_lane_offset) ** 2 + d_lane_offset * (
                                 s_coordinate - s_lane_offset) ** 3

                u_center = u
                v_center = v + offset
                self.u_lane['0'].append(u_center)
                self.v_lane['0'].append(v_center)

                coordinates_before_rotate = [u, v]
                middle_coordinates_before_rotate = [u_center, v_center]
                coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[0, 0])
                middle_coordinates_after_rotate = rotate(middle_coordinates_before_rotate, hdg, rotation_around=[0, 0])
                coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
                middle_coordinates_after_rotate_and_translation = middle_coordinates_after_rotate + np.array([x, y])

                x_coordinate = coordinates_after_rotate_and_translation[0]
                y_coordinate = coordinates_after_rotate_and_translation[1]
                middle_x_coordinate = middle_coordinates_after_rotate_and_translation[0]
                middle_y_coordinate = middle_coordinates_after_rotate_and_translation[1]

                z_coordinate = a + b * (s_coordinate - s_z) + c * (s_coordinate - s_z) ** 2 + d * (s_coordinate - s_z) ** 3
                self.ref_line_coordinates.append([x_coordinate, y_coordinate, z_coordinate])
                self.center_line_coordinates.append([middle_x_coordinate, middle_y_coordinate, z_coordinate])

            f1 = list(filter(lambda i: i <= s_coordinate, self.lane_section_s))
            index_lane_section = len(f1) - 1
            keys = list(self.road.lanes.laneSections[index_lane_section].lane_with_id.keys())
            keys = list(map(int, keys))
            keys_negative = [i for i in keys if i < 0]
            keys_positive = [i for i in keys if i > 0]
            keys_positive = list(reversed(keys_positive))
            self.keys_negative = keys_negative
            self.keys_positive = keys_positive
            for lane_id in keys_negative:
                s_offset, width_a, width_b, width_c, width_d = choose_width_parameters(self.road, s_coordinate,
                                                                                       index_lane_section, str(lane_id))
                width = width_a + width_b * s_offset + width_c * s_offset ** 2 + \
                        width_d * s_offset ** 3
                if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                    u_minus = self.u_lane[str(lane_id + 1)][-1] + width * math.sin(hdg_this_s)
                    v_minus = self.v_lane[str(lane_id + 1)][-1] - width * math.cos(hdg_this_s)
                elif self.geometry_nodes[geometry_index][0].tag == "line":
                    u_minus = self.u_lane[str(lane_id + 1)][-1]
                    v_minus = self.v_lane[str(lane_id + 1)][-1] - width

                if str(lane_id) not in self.u_lane:
                    self.u_lane[str(lane_id)] = []
                    self.v_lane[str(lane_id)] = []
                self.u_lane[str(lane_id)].append(u_minus)
                self.v_lane[str(lane_id)].append(v_minus)
                minus_coordinates_before_rotate = [u_minus, v_minus]

                if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                    minus_coordinates_after_rotate = rotate(minus_coordinates_before_rotate, hdg, rotation_around=[
                        self.road.planView.geometrys[geometry_index].model.aU,
                        self.road.planView.geometrys[geometry_index].model.aV])
                elif self.geometry_nodes[geometry_index][0].tag == "line":
                    minus_coordinates_after_rotate = rotate(minus_coordinates_before_rotate, hdg,
                                                            rotation_around=[0, 0])
                minus_coordinates_after_rotate_and_translation = minus_coordinates_after_rotate + np.array([x, y])
                minus_x = minus_coordinates_after_rotate_and_translation[0]
                minus_y = minus_coordinates_after_rotate_and_translation[1]

                if str(lane_id) not in self.lanes_coordinates:
                    self.lanes_coordinates[str(lane_id)] = []
                self.lanes_coordinates[str(lane_id)].append([minus_x, minus_y, z_coordinate])

            if len(keys_negative) == 0:
                negative_threshold = 0
            else:
                negative_threshold = min(keys_negative)

            if self.min_lane_id < negative_threshold:
                for i in range(self.min_lane_id, negative_threshold):
                    if str(i) not in self.u_lane:
                        self.u_lane[str(i)] = []
                        self.v_lane[str(i)] = []
                    self.u_lane[str(i)].append(float('NaN'))
                    self.v_lane[str(i)].append(float('NaN'))
                    if str(i) not in self.lanes_coordinates:
                        self.lanes_coordinates[str(i)] = []
                    self.lanes_coordinates[str(i)].append([float('NaN'), float('NaN'), float('NaN')])

            for lane_id in keys_positive:
                s_offset, width_a, width_b, width_c, width_d = choose_width_parameters(self.road, s_coordinate,
                                                                                       index_lane_section, str(lane_id))
                width = width_a + width_b * s_offset + width_c * s_offset ** 2 + \
                        width_d * s_offset ** 3

                if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                    u_minus = self.u_lane[str(lane_id - 1)][-1] - width * math.sin(hdg_this_s)
                    v_minus = self.v_lane[str(lane_id - 1)][-1] + width * math.cos(hdg_this_s)
                elif self.geometry_nodes[geometry_index][0].tag == "line":
                    u_minus = self.u_lane[str(lane_id - 1)][-1]
                    v_minus = self.v_lane[str(lane_id - 1)][-1] + width
                if str(lane_id) not in self.u_lane:
                    self.u_lane[str(lane_id)] = []
                    self.v_lane[str(lane_id)] = []
                self.u_lane[str(lane_id)].append(u_minus)
                self.v_lane[str(lane_id)].append(v_minus)
                minus_coordinates_before_rotate = [u_minus, v_minus]

                if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                    minus_coordinates_after_rotate = rotate(minus_coordinates_before_rotate, hdg, rotation_around=[
                        self.road.planView.geometrys[geometry_index].model.aU,
                        self.road.planView.geometrys[geometry_index].model.aV])
                elif self.geometry_nodes[geometry_index][0].tag == "line":
                    minus_coordinates_after_rotate = rotate(minus_coordinates_before_rotate, hdg,
                                                            rotation_around=[0, 0])

                minus_coordinates_after_rotate_and_translation = minus_coordinates_after_rotate + np.array([x, y])
                minus_x = minus_coordinates_after_rotate_and_translation[0]
                minus_y = minus_coordinates_after_rotate_and_translation[1]

                if str(lane_id) not in self.lanes_coordinates:
                    self.lanes_coordinates[str(lane_id)] = []
                self.lanes_coordinates[str(lane_id)].append([minus_x, minus_y, z_coordinate])

            if len(keys_positive) == 0:
                positive_threshold = 0
            else:
                positive_threshold = max(keys_positive)

            if self.max_lane_id > positive_threshold:
                for i in range(positive_threshold + 1, self.max_lane_id + 1):
                    if str(i) not in self.u_lane:
                        self.u_lane[str(i)] = []
                        self.v_lane[str(i)] = []
                    self.u_lane[str(i)].append(float('NaN'))
                    self.v_lane[str(i)].append(float('NaN'))
                    if str(i) not in self.lanes_coordinates:
                        self.lanes_coordinates[str(i)] = []
                    self.lanes_coordinates[str(i)].append([float('NaN'), float('NaN'), float('NaN')])

    # this function only can be used after the parse_object_parameters
    def calculate_objects_coordinate(self):
        for item in self.road.objects:
            s_xy, x, y, hdg, length, geometry_index = choose_ref_line_parameters(self.road, item.s, self.geometry_s)
            if self.geometry_nodes[geometry_index][0].tag == "paramPoly3":
                gradient_u = self.road.planView.geometrys[geometry_index].model.bU + 2 * \
                             self.road.planView.geometrys[geometry_index].model.cU * (item.s - s_xy) + 3 * \
                             self.road.planView.geometrys[geometry_index].model.dU * (item.s - s_xy) ** 2

                gradient_v = self.road.planView.geometrys[geometry_index].model.bV + 2 * \
                             self.road.planView.geometrys[geometry_index].model.cV * (item.s - s_xy) + 3 * \
                             self.road.planView.geometrys[geometry_index].model.dV * (item.s - s_xy) ** 2

                hdg_this_s = math.atan2(gradient_v, gradient_u)
                u = self.road.planView.geometrys[geometry_index].model.aU + \
                    self.road.planView.geometrys[geometry_index].model.bU * (item.s - s_xy) + \
                    self.road.planView.geometrys[geometry_index].model.cU * (item.s - s_xy) ** 2 + \
                    self.road.planView.geometrys[geometry_index].model.dU * (item.s - s_xy) ** 3
                v = self.road.planView.geometrys[geometry_index].model.aV + \
                    self.road.planView.geometrys[geometry_index].model.bV * (item.s - s_xy) + \
                    self.road.planView.geometrys[geometry_index].model.cV * (item.s - s_xy) ** 2 + \
                    self.road.planView.geometrys[geometry_index].model.dV * (item.s - s_xy) ** 3

                t = item.t
                u_obj = u - t * math.sin(hdg_this_s)
                v_obj = v + t * math.cos(hdg_this_s)

                obj_coordinates_before_rotate = [u_obj, v_obj]

                obj_coordinates_after_rotate = rotate(obj_coordinates_before_rotate, hdg, rotation_around=[
                    self.road.planView.geometrys[geometry_index].model.aU,
                    self.road.planView.geometrys[geometry_index].model.aV])
                obj_coordinates_after_rotate_and_translation = obj_coordinates_after_rotate + np.array([x, y])
                obj_coordinates_after_rotate_and_translation = list(obj_coordinates_after_rotate_and_translation)
                if item.subtype == 'permanentDelineator':
                    a = obj_coordinates_after_rotate_and_translation.copy()
                    a.append(item.t < 0)
                    self.delineator.append(a)
                self.obj_coordinates.append(obj_coordinates_after_rotate_and_translation)

            elif self.geometry_nodes[geometry_index][0].tag == "line":
                u = item.s - s_xy
                v = 0
                t = item.t
                u_obj = u
                v_obj = v + t
                obj_coordinates_before_rotate = [u_obj, v_obj]
                obj_coordinates_after_rotate = rotate(obj_coordinates_before_rotate, hdg, rotation_around=[0, 0])
                obj_coordinates_after_rotate_and_translation = obj_coordinates_after_rotate + np.array([x, y])
                obj_coordinates_after_rotate_and_translation = list(obj_coordinates_after_rotate_and_translation)
                if item.subtype == 'permanentDelineator':
                    a = obj_coordinates_after_rotate_and_translation.copy()
                    a.append(item.t < 0)
                    self.delineator.append(a)
                self.obj_coordinates.append(obj_coordinates_after_rotate_and_translation)

    def plot_lanes_2d(self):
        time1 = time.time()
        if len(self.s_coordinates) == 0:
            self.parse_lanes_parameters()

        if len(self.ref_line_coordinates) == 0:
            self.calculate_lanes_coordinate()

        plt.figure()
        plt.plot(np.array(self.ref_line_coordinates)[:, 0], np.array(self.ref_line_coordinates)[:, 1], 'r-.',
                 linewidth=1, label="Reference line")

        plt.plot(np.array(self.center_line_coordinates)[:, 0], np.array(self.center_line_coordinates)[:, 1],
                 linewidth=0.8, label="Center line")
        for i in self.lanes_coordinates.keys():
            plt.plot(np.array(self.lanes_coordinates[i])[:, 0], np.array(self.lanes_coordinates[i])[:, 1],
                     linewidth=0.8, label="Lane " + i + " boundary")

        plt.title("Road and objects visualization")
        time2 = time.time()
        print("Plot finished: Time used --- %s seconds ---" % (time2 - time1))
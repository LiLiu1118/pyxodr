import numpy as np
from transformation import rotate
import matplotlib.pyplot as plt
from plot_adjustment import axisEqual3D
from planView import PlanView, Geometry, ParamPoly3
from elevationProfile import Elevation, ElevationProfile
from road import Road
from lanes import Lanes, Lane, LaneOffset, LaneSection
import math

class RoadParser:

    def __init__(self, road_node):
        self.road_node = road_node
        self.geometry_nodes = []
        self.id = road_node.get('id')
        self.road = None
        self.already_gathered = False
        self.road_length = self.road_node.get("length")
        self.s_coordinates = np.linspace(0, float(self.road_length), num=int(float(self.road_length) * 100), endpoint=True,
                                         retstep=False,
                                         dtype=None, axis=0)
        self.center_line_coordinates = None


    def setup_road_information(self):
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
            lanes.laneSections.append(lane_section)
        self.road.lanes = lanes
        self.already_gathered = True



    def get_road_length(self):

        return self.road_node.get("length")

    def get_3d_coordinate(self, s_coordinate):

        s_xy = x = y = hdg = length = model = None
        s_z = a = b = c = d = None
        s_laneOffset = a_laneOffset = b_laneOffset = c_laneOffset = d_laneOffset = None
        geometry_index = 0

        for i in range(len(self.road.lanes.laneOffsets) - 1):
            if (s_coordinate >= float(self.road.lanes.laneOffsets[i].s) and s_coordinate < float(self.road.lanes.laneOffsets[i+1].s)):
                s_laneOffset = float(self.road.lanes.laneOffsets[i].s)
                a_laneOffset = float(self.road.lanes.laneOffsets[i].a)
                b_laneOffset = float(self.road.lanes.laneOffsets[i].b)
                c_laneOffset = float(self.road.lanes.laneOffsets[i].c)
                d_laneOffset = float(self.road.lanes.laneOffsets[i].d)
                break
            if (i == len(self.road.lanes.laneOffsets) - 2):
                s_laneOffset = float(self.road.lanes.laneOffsets[-1].s)
                a_laneOffset = float(self.road.lanes.laneOffsets[-1].a)
                b_laneOffset = float(self.road.lanes.laneOffsets[-1].b)
                c_laneOffset = float(self.road.lanes.laneOffsets[-1].c)
                d_laneOffset = float(self.road.lanes.laneOffsets[-1].d)
                break

        for i in range(len(self.road.planView.geometrys) - 1):
            if (s_coordinate >= float(self.road.planView.geometrys[i].s) and s_coordinate < float(self.road.planView.geometrys[i+1].s)):
                s_xy = float(self.road.planView.geometrys[i].s)
                x = float(self.road.planView.geometrys[i].x)
                y = float(self.road.planView.geometrys[i].y)
                hdg = float(self.road.planView.geometrys[i].hdg)
                length = float(self.road.planView.geometrys[i].length)
                geometry_index = i
                break
            if (i == len(self.road.planView.geometrys) - 2):
                s_xy = float(self.road.planView.geometrys[-1].s)
                x = float(self.road.planView.geometrys[-1].x)
                y = float(self.road.planView.geometrys[-1].y)
                hdg = float(self.road.planView.geometrys[-1].hdg)
                length = float(self.road.planView.geometrys[-1].length)
                geometry_index = -1
                break

        for i in range(len(self.road.elevationProfile.elevations) - 1):
            if (s_coordinate >= float(self.road.elevationProfile.elevations[i].s) and s_coordinate < float(self.road.elevationProfile.elevations[i + 1].s)):
                s_z = float(self.road.elevationProfile.elevations[i].s)
                a = float(self.road.elevationProfile.elevations[i].a)
                b = float(self.road.elevationProfile.elevations[i].b)
                c = float(self.road.elevationProfile.elevations[i].c)
                d = float(self.road.elevationProfile.elevations[i].d)
                break
            if (i == len(self.road.elevationProfile.elevations) - 2):
                s_z = float(self.road.elevationProfile.elevations[-1].s)
                a = float(self.road.elevationProfile.elevations[-1].a)
                b = float(self.road.elevationProfile.elevations[-1].b)
                c = float(self.road.elevationProfile.elevations[-1].c)
                d = float(self.road.elevationProfile.elevations[-1].d)
                break
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

            coordinates_before_rotate = np.vstack((u, v)).T
            middle_coordinates_before_rotate = np.vstack((u_center, v_center)).T
            coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[float(self.road.planView.geometrys[geometry_index].model.aU), float(self.road.planView.geometrys[geometry_index].model.aV)])
            middle_coordinates_after_rotate = rotate(middle_coordinates_before_rotate, hdg, rotation_around=[
                float(self.road.planView.geometrys[geometry_index].model.aU),
                float(self.road.planView.geometrys[geometry_index].model.aV)])

            coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
            middle_coordinates_after_rotate_and_translation = middle_coordinates_after_rotate + np.array([x, y])

            x_coor = coordinates_after_rotate_and_translation[0]
            y_coor = coordinates_after_rotate_and_translation[1]

            middle_x_coor = middle_coordinates_after_rotate_and_translation[0]
            middle_y_coor = middle_coordinates_after_rotate_and_translation[1]

        z_coor = a + b * (s_coordinate - s_z) + c * (s_coordinate - s_z) ** 2 + d * (s_coordinate - s_z) ** 3

        return x_coor, y_coor, z_coor, middle_x_coor, middle_y_coor


    def plot_ref_line(self):
        if (len(self.s_coordinates) == 0):
            self.setup_road_information()

        plt.figure()
        ax = plt.axes(projection="3d")
        xyz = []
        xyz_middle = []
        for s_item in self.s_coordinates:
            x, y, z, middle_x, middle_y = self.get_3d_coordinate(s_item)
            xyz.append([x, y, z])
            xyz_middle.append([middle_x, middle_y, z])
        xyz = np.array(xyz)
        xyz_middle = np.array(xyz_middle)
        ax.plot3D(xyz[:, 0], xyz[:, 1], xyz[:, 2], 'green')
        ax.plot3D(xyz_middle[:, 0], xyz_middle[:, 1], xyz_middle[:, 2], 'blue')
        axisEqual3D(ax)
        plt.title("Reference line of Road Section {0} \nTotal number of plan view geometries: {1}".format(self.id,
                                                                                                   len(self.road.planView.geometrys)))
        plt.xlabel("X", fontsize=15)
        plt.ylabel("Y", fontsize=15)
        plt.show()
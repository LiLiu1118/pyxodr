import numpy as np
from transformation import rotate
import matplotlib.pyplot as plt
from plot_adjustment import axisEqual3D

class Road:

    def __init__(self, road_node):
        self.road_node = road_node
        self.id = road_node.get('id')
        self.length = None
        self.ref_line_params_xy = None
        self.ref_line_params_z = None
        self.s_coordinates = []

    # if want to do something with road object, should first calculat params
    def calculate_ref_line_params(self):
        if(self.ref_line_params_xy != None and self.ref_line_params_z != None):
            return self.ref_line_params_xy, self.ref_line_params_z

        if (self.road_node == None):
            print("Road node is None!")
            raise TypeError

        geometries_params_xy = []
        geometries_xy_tags = self.road_node.findall('.//geometry')
        for geometry in geometries_xy_tags:
            item = []
            s = geometry.get('s')
            x = geometry.get('x')
            y = geometry.get('y')
            hdg = geometry.get('hdg')
            length = geometry.get('length')
            item.append(float(s))
            item.append(float(x))
            item.append(float(y))
            item.append(float(hdg))
            item.append(float(length))
            poly_params = geometry.findall('.//paramPoly3')
            for poly_param in poly_params:
                aU = poly_param.get('aU')
                bU = poly_param.get('bU')
                cU = poly_param.get('cU')
                dU = poly_param.get('dU')
                aV = poly_param.get('aV')
                bV = poly_param.get('bV')
                cV = poly_param.get('cV')
                dV = poly_param.get('dV')
                item.append(float(aU))
                item.append(float(bU))
                item.append(float(cU))
                item.append(float(dU))
                item.append(float(aV))
                item.append(float(bV))
                item.append(float(cV))
                item.append(float(dV))
            geometries_params_xy.append(item)

        geometries_params_z = []
        geometries_z_tags = self.road_node.findall('.//elevation')
        road_length = np.array(geometries_params_xy)[-1, 0] + np.array(geometries_params_xy)[-1, 4]

        for geometry in geometries_z_tags:
            item = []
            s = geometry.get('s')
            a = geometry.get('a')
            b = geometry.get('b')
            c = geometry.get('c')
            d = geometry.get('d')

            item.append(float(s))
            item.append(float(a))
            item.append(float(b))
            item.append(float(c))
            item.append(float(d))
            geometries_params_z.append(item)
        self.ref_line_params_xy = geometries_params_xy
        self.ref_line_params_z = geometries_params_z
        self.length = np.array(self.ref_line_params_xy)[-1, 0] + np.array(self.ref_line_params_xy)[-1, 4]
        self.s_coordinates = np.linspace(0, self.length, num=int(self.length * 100), endpoint=True, retstep=False,
                    dtype=None, axis=0)


    def get_road_length(self):
        if (self.road_node == None):
            print("Road node is None!")
            raise TypeError

        if (self.length == None):
            self.set_get_ref_line_params()
        return self.length

    def get_3d_coordinate(self, s_coordinate):
        s_xy = x = y = hdg = length = aU = bU = cU = dU = aV = bV = cV = dV = None
        s_z = a = b = c = d = None

        for i in range(len(self.ref_line_params_xy) - 1):
            if (s_coordinate >= self.ref_line_params_xy[i][0] and s_coordinate < self.ref_line_params_xy[i + 1][0]):
                s_xy, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV = self.ref_line_params_xy[i]
                break
            if (i == len(self.ref_line_params_xy) - 2):
                s_xy, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV = self.ref_line_params_xy[-1]
                break

        for i in range(len(self.ref_line_params_z) - 1):
            if (s_coordinate >= self.ref_line_params_z[i][0] and s_coordinate < self.ref_line_params_z[i + 1][0]):
                s_z, a, b, c, d = self.ref_line_params_z[i]
                break
            if (i == len(self.ref_line_params_z) - 2):
                s_z, a, b, c, d = self.ref_line_params_z[-1]
                break

        u = aU + bU * (s_coordinate - s_xy) + cU * (s_coordinate - s_xy) ** 2 + dU * (s_coordinate - s_xy) ** 3
        v = aV + bV * (s_coordinate - s_xy) + cV * (s_coordinate - s_xy) ** 2 + dV * (s_coordinate - s_xy) ** 3
        coordinates_before_rotate = np.vstack((u, v)).T
        coordinates_before_rotate_and_translation = coordinates_before_rotate + np.array([x, y])
        coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[aU, aV])
        coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
        x_coor = coordinates_after_rotate_and_translation[0]
        y_coor = coordinates_after_rotate_and_translation[1]
        z_coor = a + b * (s_coordinate - s_z) + c * (s_coordinate - s_z) ** 2 + d * (s_coordinate - s_z) ** 3

        return x_coor, y_coor, z_coor


    def plot_ref_line(self):
        if (len(self.s_coordinates) == 0):
            self.set_get_ref_line_params()

        fig = plt.figure()
        ax = plt.axes(projection="3d")

        xyz = []
        for s_item in self.s_coordinates:
            x, y, z = self.get_3d_coordinate(s_item)
            xyz.append([x, y, z])
        xyz = np.array(xyz)
        ax.plot3D(xyz[:, 0], xyz[:, 1], xyz[:, 2], 'green')
        axisEqual3D(ax)
        plt.title("Reference line of Road Section {0} \nTotal number of line segments: {1}".format(id,
                                                                                                   len(self.ref_line_params_xy)))
        plt.xlabel("X", fontsize=15)
        plt.ylabel("Y", fontsize=15)
        plt.show()
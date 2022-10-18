import numpy as np
import xml.etree.ElementTree as ET


class XODRParser:

    def __init__(self, file_path):
        self.id = None
        self.road = None
        self.ref_line_params_xy = None
        self.ref_line_params_z = None
        self.file_path = file_path
        self.tree = ET.parse(self.file_path)
        self.root = self.tree.getroot()

    def set_id(self, id):
        self.id = id

    def find_road(self, ):
        if (self.road):
            return self.road
        else:
            roads = self.root.findall('.//road')
            for road in roads:
                id_temp = road.get('id')
                if (id_temp == self.id):
                    self.road = road
                    return self.road

    def get_ref_line_params(self):
        if (self.road == None):
            self.find_road()

        geometries_params_xy = []
        geometries_xy_tags = self.road.findall('.//geometry')
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
        geometries_z_tags = self.road.findall('.//elevation')
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
        return geometries_params_xy, geometries_params_z
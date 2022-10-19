import cv2
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET


def rotate(vector, theta, rotation_around=None) -> np.ndarray:
    """
    :param vector: list of length 2 OR
                   list of list where inner list has size 2 OR
                   1D numpy array of length 2 OR
                   2D numpy array of size (number of points, 2)
    :param theta: rotation angle in degree (+ve value of anti-clockwise rotation)
    :param rotation_around: "vector" will be rotated around this point,
                    otherwise [0, 0] will be considered as rotation axis
    :return: rotated "vector" about "theta" degree around rotation
             axis "rotation_around" numpy array
    """
    vector = np.array(vector)

    if vector.ndim == 1:
        vector = vector[np.newaxis, :]

    if rotation_around is not None:
        vector = vector - rotation_around

    vector = vector.T

    # theta = np.radians(theta)

    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    output: np.ndarray = (rotation_matrix @ vector).T

    if rotation_around is not None:
        output = output + rotation_around

    return output.squeeze()


def plot_reference_line(s, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV):
    p = np.linspace(0, length, num=10, endpoint=True, retstep=False, dtype=None, axis=0)

    def get_u():
        u = aU + bU * p + cU * p**2 + dU * p**3
        return u

    def get_v():
        v = aV + bV * p + cV * p**2 + dV * p**3
        return v

    u = get_u()
    v = get_v()
    coordinates_before_rotate = np.vstack((u, v)).T
    coordinates_before_rotate_and_translation = coordinates_before_rotate + np.array([x,y])
    coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[aU, aV])
    coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x,y])
    return u, v, coordinates_after_rotate_and_translation, coordinates_after_rotate, coordinates_before_rotate_and_translation

def get_3d_coordinate(s_item, geometries_params_xy, geometries_params_z):
    s_xy = x = y = hdg = length = aU = bU = cU = dU = aV = bV = cV = dV = None
    s_z = a = b = c = d = None

    for i in range(len(geometries_params_xy)-1):
        if (s_item >= geometries_params_xy[i][0] and s_item < geometries_params_xy[i+1][0]):
            s_xy, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV = geometries_params_xy[i]
            break
        if (i == len(geometries_params_xy)-2):
            s_xy, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV = geometries_params_xy[-1]
            break

    for i in range(len(geometries_params_z)-1):
        if (s_item >= geometries_params_z[i][0] and s_item < geometries_params_z[i+1][0]):
            s_z, a, b, c, d = geometries_params_z[i]
            break
        if (i == len(geometries_params_z)-2):
            s_z, a, b, c, d = geometries_params_z[-1]
            break

    u = aU + bU * (s_item-s_xy) + cU * (s_item-s_xy) ** 2 + dU * (s_item-s_xy) ** 3
    v = aV + bV * (s_item-s_xy) + cV * (s_item-s_xy) ** 2 + dV * (s_item-s_xy) ** 3
    coordinates_before_rotate = np.vstack((u, v)).T
    coordinates_before_rotate_and_translation = coordinates_before_rotate + np.array([x, y])
    coordinates_after_rotate = rotate(coordinates_before_rotate, hdg, rotation_around=[aU, aV])
    coordinates_after_rotate_and_translation = coordinates_after_rotate + np.array([x, y])
    x_coor = coordinates_after_rotate_and_translation[0]
    y_coor = coordinates_after_rotate_and_translation[1]
    z_coor = a + b * (s_item - s_z) + c * (s_item - s_z) ** 2 + d * (s_item - s_z) ** 3

    return x_coor, y_coor, z_coor







if __name__ == "__main__":

    tree = ET.parse("./example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr")
    root = tree.getroot()
    orderIds = ["1000611004", "1000611067"]
    roads = root.findall('.//road')
    road_we_need = None
    id = None
    for road in roads:
        id = road.get('id')
        if (id == '1002000'):
            road_we_need = road
            break

    # parse the params of xy plane, divide by different s into sublist
    geometries_params_xy = []
    geometries_xy_tags = road_we_need.findall('.//geometry')
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

    # parse the params of z coordinates, divide by different s into sublist
    geometries_params_z = []
    geometries_z_tags = road_we_need.findall('.//elevation')
    print(len(geometries_z_tags))
    road_length = np.array(geometries_params_xy)[-1,0] + np.array(geometries_params_xy)[-1,4]
    print("road length: ", road_length)
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

    fig = plt.figure()
    ax = plt.axes(projection="3d")

    s_coordinates = np.linspace(0, road_length, num=int(road_length*100), endpoint=True, retstep=False, dtype=None, axis=0)
    xyz = []
    for s_item in s_coordinates:
        x, y, z = get_3d_coordinate(s_item, geometries_params_xy, geometries_params_z)
        xyz.append([x, y, z])
    xyz = np.array(xyz)
    print("xyz: ", xyz)
    ax.plot3D(xyz[:,0], xyz[:,1], xyz[:,2], 'green')


    def axisEqual3D(ax):
        extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        sz = extents[:, 1] - extents[:, 0]
        centers = np.mean(extents, axis=1)
        maxsize = max(abs(sz))
        r = maxsize / 2
        for ctr, dim in zip(centers, 'xyz'):
            getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


    axisEqual3D(ax)
    plt.title("Reference line of Road Section {0} \nTotal number of line segments: {1}".format(id, len(geometries_params_xy)))
    # plt.text(-5, 60, 'Parabola $Y = x^2$', fontsize=22)
    plt.xlabel("X", fontsize=15)
    plt.ylabel("Y", fontsize=15)
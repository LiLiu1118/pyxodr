from xodr_parser import XODRParser
import matplotlib.pyplot as plt
import numpy as np
import time
from roadParser import RoadParser
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
import math

# time1 = time.time()
# path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
# parser1 = XODRParser(path)
# gps = [455262.18, 5724680.20]
# exist, id, lane, road_parser = parser1.find_road_from_gps(gps)
# projection = parser1.calculate_projection_geometry(road_parser, gps, lane, 10)
#
# road_parser.plot_2d()
# print(id)
# plt.scatter(gps[0], gps[1], s=6, c='k')
# plt.plot(projection[:,0], projection[:,1], c='k', label='Vehicle')
# print(projection[1,0], projection[1,1])
# print(projection[0,0], projection[0,1])
# plt.arrow(projection[:,0][-2], projection[:,1][-2], projection[:,0][-1]-projection[:,0][-2], projection[:,1][-1]-projection[:,1][-2], width=0.1,fc='k', ec='k', label="Projection created")
# plt.axis('equal')
# plt.legend()
# plt.show()
# time2 = time.time()
# print("time: ", time2-time1)

path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)
road = parser1.find_id('1002000')
road.parse_lanes_parameters()
road.calculate_lanes_coordinate()
road.plot_lanes_2d()
# road.parse_object_parameters()
# road.calculate_objects_coordinate()
# obj_coordinates = np.array(road.obj_coordinates)

# deli = road.delineator.copy()
# deli.reverse()
# print(deli)
# delic = np.array(deli)
# plt.figure()
# plt.scatter(delic[:,0].astype('float'), delic[:,1].astype('float'))
# plt.show()
#
# with open(r'C:/Users/liuli110/Desktop/delineator.txt', 'w') as fp:
#     fp.write("x;\ty;\tright hand side;\n")
#     for item in deli:
#         # write each item on a new line
#         fp.write("%s;\t" % item[0])
#         fp.write("%s;\t" % item[1])
#         fp.write("%s;\n" % item[2])
#     print('Done')

# road.plot_lanes_2d()
# plt.scatter(obj_coordinates[:, 0], obj_coordinates[:, 1], s=2, marker='*', label="Objects")
plt.axis('equal')
plt.legend()
plt.show()





from xodr_parser import XODRParser
import matplotlib.pyplot as plt
import numpy as np
import time

path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)

time1 = time.time()
road_parser = parser1.find_road("1002000")
time2 = time.time()
print("Road found: Time used --- %s seconds ---" % (time2 - time1))

time3 = time.time()
road_parser.parse_geometry_parameters()
time4 = time.time()
print("Parameter parsed: Time used --- %s seconds ---" % (time4 - time3))

time5 = time.time()
road_parser.calculate_3d_coordinate()
time6 = time.time()
print("All coordinates calculated: Time used --- %s seconds ---" % (time6 - time5))

road_parser.plot_2D()




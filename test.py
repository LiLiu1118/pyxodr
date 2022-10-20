from xodr_parser import XODRParser

path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)

road_parser= parser1.find_road("1002000")

road_parser.gather_ref_line_params()
road_parser.gather_ref_line_params()
print(road_parser.s_coordinates)
print(road_parser.get_road_length())
print(road_parser.road.length)
print(road_parser.road.id)
print(road_parser.already_gathered)
road_parser.plot_ref_line()

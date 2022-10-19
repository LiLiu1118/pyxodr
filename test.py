from xodr_parser import XODRParser

path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)

road = parser1.find_road("1002000")

road.calculate_ref_line_params()
road.plot_ref_line()
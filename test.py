from xodr_parser import XODRParser

path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)

road_parser = parser1.find_road("1002000")

road_parser.setup_road_information()
road_parser.plot_ref_line()


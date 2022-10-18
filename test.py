from xodr_parser import XODRParser

path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)
parser1.set_id("1002000")
parser1.set_get_ref_line_params()
parser1.plot_ref_line()
from xodr_parser import XODRParser

path = "C:/Users/liuli110/Downloads/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser1 = XODRParser(path)
parser1.set_id("1002000")
xy, z = parser1.get_ref_line_params()
print(len(xy))
print(len(z))

def get_width_parameters(road, s_coordinate):
    for i in range(len(road.lanes.laneSections) - 1):
        if (s_coordinate >= float(road.lanes.laneSections[i].s) and s_coordinate < float(
                road.lanes.laneSections[i + 1].s)):
            if (len(road.lanes.laneSections[i].lane_minus1.width_items) == 1):
                s_Offset = s_coordinate - float(road.lanes.laneSections[i].s)
                s_Offset_1 = s_coordinate - float(road.lanes.laneSections[i].s) - float(
                    road.lanes.laneSections[i].lane_minus1.width_items[0].sOffset)
                width_a = float(road.lanes.laneSections[i].lane_minus1.width_items[0].a)
                width_b = float(road.lanes.laneSections[i].lane_minus1.width_items[0].b)
                width_c = float(road.lanes.laneSections[i].lane_minus1.width_items[0].c)
                width_d = float(road.lanes.laneSections[i].lane_minus1.width_items[0].d)
                break
            for j in range(len(road.lanes.laneSections[i].lane_minus1.width_items) - 1):
                if (s_coordinate >= float(road.lanes.laneSections[i].s) + float(
                        road.lanes.laneSections[i].lane_minus1.width_items[j].sOffset)
                        and s_coordinate < float(road.lanes.laneSections[i].s) + float(
                            road.lanes.laneSections[i].lane_minus1.width_items[j + 1].sOffset)):
                    lanesections = float(road.lanes.laneSections[i].s)

                    s_Offset = s_coordinate - float(road.lanes.laneSections[i].s)
                    s_Offset_1 = s_coordinate - float(road.lanes.laneSections[i].s) - float(
                        road.lanes.laneSections[i].lane_minus1.width_items[j].sOffset)
                    width_a = float(road.lanes.laneSections[i].lane_minus1.width_items[j].a)
                    width_b = float(road.lanes.laneSections[i].lane_minus1.width_items[j].b)
                    width_c = float(road.lanes.laneSections[i].lane_minus1.width_items[j].c)
                    width_d = float(road.lanes.laneSections[i].lane_minus1.width_items[j].d)
                    break

                if (j == len(road.lanes.laneSections[i].lane_minus1.width_items) - 2):
                    s_Offset = s_coordinate - float(road.lanes.laneSections[i].s)
                    s_Offset_1 = s_coordinate - float(road.lanes.laneSections[i].s) - float(
                        road.lanes.laneSections[i].lane_minus1.width_items[-1].sOffset)
                    lanesections = float(road.lanes.laneSections[i].s)
                    width_a = float(road.lanes.laneSections[i].lane_minus1.width_items[-1].a)
                    width_b = float(road.lanes.laneSections[i].lane_minus1.width_items[-1].b)
                    width_c = float(road.lanes.laneSections[i].lane_minus1.width_items[-1].c)
                    width_d = float(road.lanes.laneSections[i].lane_minus1.width_items[-1].d)
                    break
            break
        if (i == len(road.lanes.laneSections) - 2):
            if (len(road.lanes.laneSections[-1].lane_minus1.width_items) == 1):
                # lane_setion_s = float(self.road.lanes.laneSections[i].s)
                s_Offset = s_coordinate - float(road.lanes.laneSections[-1].s)
                s_Offset_1 = s_coordinate - float(road.lanes.laneSections[-1].s) - float(
                    road.lanes.laneSections[-1].lane_minus1.width_items[0].sOffset)
                lanesections = float(road.lanes.laneSections[i].s)
                width_a = float(road.lanes.laneSections[-1].lane_minus1.width_items[0].a)
                width_b = float(road.lanes.laneSections[-1].lane_minus1.width_items[0].b)
                width_c = float(road.lanes.laneSections[-1].lane_minus1.width_items[0].c)
                width_d = float(road.lanes.laneSections[-1].lane_minus1.width_items[0].d)
                break
            for j in range(len(road.lanes.laneSections[-1].lane_minus1.width_items) - 1):
                if (s_coordinate >= float(road.lanes.laneSections[-1].s) + float(
                        road.lanes.laneSections[-1].lane_minus1.width_items[j].sOffset)
                        and s_coordinate < float(road.lanes.laneSections[-1].s) + float(
                            road.lanes.laneSections[-1].lane_minus1.width_items[j + 1].sOffset)):
                    # lane_setion_s = float(self.road.lanes.laneSections[i].s)
                    s_Offset = s_coordinate - float(road.lanes.laneSections[-1].s)
                    s_Offset_1 = s_coordinate - float(road.lanes.laneSections[-1].s) - float(
                        road.lanes.laneSections[-1].lane_minus1.width_items[j].sOffset)
                    lanesections = float(road.lanes.laneSections[i].s)
                    width_a = float(road.lanes.laneSections[-1].lane_minus1.width_items[j].a)
                    width_b = float(road.lanes.laneSections[-1].lane_minus1.width_items[j].b)
                    width_c = float(road.lanes.laneSections[-1].lane_minus1.width_items[j].c)
                    width_d = float(road.lanes.laneSections[-1].lane_minus1.width_items[j].d)
                    break
                if (j == len(road.lanes.laneSections[-1].lane_minus1.width_items) - 2):
                    s_Offset = s_coordinate - float(road.lanes.laneSections[-1].s)
                    s_Offset_1 = s_coordinate - float(road.lanes.laneSections[-1].s) - float(
                        road.lanes.laneSections[-1].lane_minus1.width_items[-1].sOffset)
                    lanesections = float(road.lanes.laneSections[i].s)

                    width_a = float(road.lanes.laneSections[-1].lane_minus1.width_items[-1].a)
                    width_b = float(road.lanes.laneSections[-1].lane_minus1.width_items[-1].b)
                    width_c = float(road.lanes.laneSections[-1].lane_minus1.width_items[-1].c)
                    width_d = float(road.lanes.laneSections[-1].lane_minus1.width_items[-1].d)
                    break
            break
    return s_Offset_1, width_a, width_b, width_c, width_d
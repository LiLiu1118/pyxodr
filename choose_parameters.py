
def choose_width_parameters(road, s_coordinate):
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

def choose_lane_offset_parameters(road, s_coordinate):
    for i in range(len(road.lanes.laneOffsets) - 1):
        if (s_coordinate >= float(road.lanes.laneOffsets[i].s) and s_coordinate < float(
                road.lanes.laneOffsets[i + 1].s)):
            s_laneOffset = float(road.lanes.laneOffsets[i].s)
            a_laneOffset = float(road.lanes.laneOffsets[i].a)
            b_laneOffset = float(road.lanes.laneOffsets[i].b)
            c_laneOffset = float(road.lanes.laneOffsets[i].c)
            d_laneOffset = float(road.lanes.laneOffsets[i].d)
            break
        if (i == len(road.lanes.laneOffsets) - 2):
            s_laneOffset = float(road.lanes.laneOffsets[-1].s)
            a_laneOffset = float(road.lanes.laneOffsets[-1].a)
            b_laneOffset = float(road.lanes.laneOffsets[-1].b)
            c_laneOffset = float(road.lanes.laneOffsets[-1].c)
            d_laneOffset = float(road.lanes.laneOffsets[-1].d)
            break
    return s_laneOffset, a_laneOffset, b_laneOffset, c_laneOffset, d_laneOffset

def choose_ref_line_parameters(road, s_coordinate):
    for i in range(len(road.planView.geometrys) - 1):
        if (s_coordinate >= float(road.planView.geometrys[i].s) and s_coordinate < float(
                road.planView.geometrys[i + 1].s)):
            s_xy = float(road.planView.geometrys[i].s)
            x = float(road.planView.geometrys[i].x)
            y = float(road.planView.geometrys[i].y)
            hdg = float(road.planView.geometrys[i].hdg)
            length = float(road.planView.geometrys[i].length)
            geometry_index = i
            break
        if (i == len(road.planView.geometrys) - 2):
            s_xy = float(road.planView.geometrys[-1].s)
            x = float(road.planView.geometrys[-1].x)
            y = float(road.planView.geometrys[-1].y)
            hdg = float(road.planView.geometrys[-1].hdg)
            length = float(road.planView.geometrys[-1].length)
            geometry_index = -1
            break
    return s_xy, x, y, hdg, length, geometry_index

def choose_elevation_parameters(road, s_coordinate):
    for i in range(len(road.elevationProfile.elevations) - 1):
        if (s_coordinate >= float(road.elevationProfile.elevations[i].s) and s_coordinate < float(
                road.elevationProfile.elevations[i + 1].s)):
            s_z = float(road.elevationProfile.elevations[i].s)
            a = float(road.elevationProfile.elevations[i].a)
            b = float(road.elevationProfile.elevations[i].b)
            c = float(road.elevationProfile.elevations[i].c)
            d = float(road.elevationProfile.elevations[i].d)
            break
        if (i == len(road.elevationProfile.elevations) - 2):
            s_z = float(road.elevationProfile.elevations[-1].s)
            a = float(road.elevationProfile.elevations[-1].a)
            b = float(road.elevationProfile.elevations[-1].b)
            c = float(road.elevationProfile.elevations[-1].c)
            d = float(road.elevationProfile.elevations[-1].d)
            break
    return s_z, a, b, c, d
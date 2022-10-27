
def choose_width_parameters(road, s_coordinate, index1, id):
    # f1 = list(filter(lambda x: x <= s_coordinate, lane_section_s))
    # index1 = len(f1) - 1
    # print("how many id exist: ", len(list(road.lanes.laneSections[index1].lane_with_id.keys())))
    s_offset_list = [width_items.sOffset for width_items in road.lanes.laneSections[index1].lane_with_id[id].width_items]
    f2 = list(filter(lambda x: road.lanes.laneSections[index1].s + x <= s_coordinate, s_offset_list))
    index2 = len(f2) - 1
    s_offset = s_coordinate - road.lanes.laneSections[index1].s - \
        road.lanes.laneSections[index1].lane_with_id[id].width_items[index2].sOffset
    width_a = road.lanes.laneSections[index1].lane_with_id[id].width_items[index2].a
    width_b = road.lanes.laneSections[index1].lane_with_id[id].width_items[index2].b
    width_c = road.lanes.laneSections[index1].lane_with_id[id].width_items[index2].c
    width_d = road.lanes.laneSections[index1].lane_with_id[id].width_items[index2].d
    return s_offset, width_a, width_b, width_c, width_d

def choose_lane_offset_parameters(road, s_coordinate, laneoffset_s):
    f = list(filter(lambda x: x <= s_coordinate, laneoffset_s))
    index = len(f) - 1
    s_lane_offset = road.lanes.laneOffsets[index].s
    a_lane_offset = road.lanes.laneOffsets[index].a
    b_lane_offset = road.lanes.laneOffsets[index].b
    c_lane_offset = road.lanes.laneOffsets[index].c
    d_lane_offset = road.lanes.laneOffsets[index].d

    return s_lane_offset, a_lane_offset, b_lane_offset, c_lane_offset, d_lane_offset

def choose_ref_line_parameters(road, s_coordinate, geometry_s):
    f = list(filter(lambda x: x <= s_coordinate, geometry_s))
    index = len(f) - 1
    s_xy = road.planView.geometrys[index].s
    x = road.planView.geometrys[index].x
    y = road.planView.geometrys[index].y
    hdg = road.planView.geometrys[index].hdg
    length = road.planView.geometrys[index].length
    geometry_index = index

    return s_xy, x, y, hdg, length, geometry_index

def choose_elevation_parameters(road, s_coordinate, elevation_s):
    f = list(filter(lambda x: x <= s_coordinate, elevation_s))
    index = len(f) - 1
    s_z = road.elevationProfile.elevations[index].s
    a = road.elevationProfile.elevations[index].a
    b = road.elevationProfile.elevations[index].b
    c = road.elevationProfile.elevations[index].c
    d = road.elevationProfile.elevations[index].d

    return s_z, a, b, c, d
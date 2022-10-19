import numpy as np
import xml.etree.ElementTree as ET


from road import Road

class XODRParser:

    def __init__(self, file_path):
        self.file_path = file_path
        self.tree = ET.parse(self.file_path)
        self.root = self.tree.getroot()
        self.roads = {}

    def find_road(self, id):
        if (id in self.roads):
            return self.roads[id]
        else:
            roads = self.root.findall('.//road')
            for road in roads:
                id_temp = road.get('id')
                if (id_temp == id):
                    road_founded = Road(road)
                    self.roads[id] = road_founded
                    return road_founded

        return None

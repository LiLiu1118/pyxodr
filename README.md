# pyxodr

pyxodr is a Python library for dealing with OpenDRIVE files, including the parsing function for road surface geometric parameters stored in the OpenDRIVE format, as well as data calculation and visualization functions.

## The OpenDRIVE format

![ASAM logo](https://github.com/LiLiu1118/pyxodr/blob/develop/ASAM_official_files/association-for-standardization-of-automation-and-measuring-systems-asam-logo-vector.png)

The ASAM OpenDRIVE format provides a common base for describing road networks with extensible markup language (XML) syntax, using the file extension xodr. The data that is stored in an ASAM OpenDRIVE file describes the geometry of roads, lanes and objects, such as roadmarks on the road, as well as features along the roads, like signals. The road networks that are described in the ASAM OpenDRIVE file can either be synthetic or based on real data. (Intercepted from ASAM official website). For more information: [Click here go to the ASAM OpenDRIVE official documentation](https://releases.asam.net/OpenDRIVE/1.6.0/ASAM_OpenDRIVE_BS_V1-6-0.html#_roads)

For example, the OpenDRIVE format defines a crosswalk using the following XML fragment

![ASAM crosswalk](https://github.com/LiLiu1118/pyxodr/blob/develop/ASAM_official_files/crosswalk.png)

```xml
 <objects>
            <object type="crosswalk" id="10" s="10.0" t="0.0" zOffset="0.0"
            orientation="none" length="10.0" width="7.0" hdg="0.0" pitch="0.0" roll="0.0">
            <outlines>
                  <outline id="0">
                  <cornerRoad s="5.0" t="3.5" dz="0.0" height="4.0" id="0"/>
                  <cornerRoad s="8.0" t="-3.5" dz="0.0" height="4.0" id="1"/>
                  <cornerRoad s="12.0" t="-3.5" dz="0.0" height="4.0" id="2"/>
                  <cornerRoad s="15.0" t="3.5" dz="0.0" height="4.0" id="3"/>
               </outline>
            </outlines>
            <markings>
              <marking width="0.1" color="white" zOffset="0.005" spaceLength ="0.05"
              lineLength ="0.2" startOffset="0.0" stopOffset="0.0">
                  <cornerReference id="0"/>
                  <cornerReference id="1"/>
              </marking>
              <marking width="0.1" color="white" zOffset="0.005" spaceLength ="0.05"
              lineLength ="0.2" startOffset="0.0" stopOffset="0.0">
                  <cornerReference id="2"/>
                  <cornerReference id="3"/>
              </marking>
            </markings>
            </object>
        </objects>
```

## Installation

As a work during the author's internship, the installation as a python library is currently not supported. It may become possible after obtaining permission, and the possible installation command in the future is as follows:

```bash
pip install pyxodr
```

## Usage

This parser provides a variety of functions, the most basic methods of use are as follows

```python
from xodr_parser import XODRParser

# define OpenDRIVE file path
path = "example_files/2022-09-13_1579_AHEAD_Lippstadt_Innenstadt_HeWeg_ODR.xodr"
parser = XODRParser(path)

# find road by id
road = parser.find_road("1002000")

# get all params for reference line calculated
road.calculate_ref_line_params()

# visualize the reference line
road.plot_ref_line()
```

For further functions: [Click here to read the documentation](https://github.com/LiLiu1118/pyxodr/blob/master/documentation/XODRParser_Documentation.pdf)

## Example results

### road visulization

![road visulization](https://github.com/LiLiu1118/pyxodr/blob/develop/example_results/road_visulization.png)

### intersection detail

![intersection detail](https://github.com/LiLiu1118/pyxodr/blob/develop/example_results/intersection_detail.png)

### object visulization

![object visulization](https://github.com/LiLiu1118/pyxodr/blob/develop/example_results/object_visulization.png)


## License
[MIT](https://choosealicense.com/licenses/mit/)

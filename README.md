# pyxodr

Pyxodr is a Python library for dealing with OpenDRIVE files.

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


## License
[MIT](https://choosealicense.com/licenses/mit/)

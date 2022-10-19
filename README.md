# pyxodr

Pyxodr is a Python library for dealing with OpenDRIVE files.

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install foobar.

```bash
pip install pyxodr
```

## Usage

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

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
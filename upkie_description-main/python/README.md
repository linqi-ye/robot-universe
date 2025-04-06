# Upkie wheeled biped robot

<img src="https://user-images.githubusercontent.com/1189580/169594012-2d685579-2b66-4470-9def-57bd0656b420.png" align="right" width="300">

[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie_description.svg)](https://anaconda.org/conda-forge/upkie_description)
[![PyPI version](https://img.shields.io/pypi/v/upkie_description)](https://pypi.org/project/upkie_description/)

URDF descriptions for the [Upkie](https://hackaday.io/project/185729-upkie-wheeled-biped-robot) wheeled biped.

## Installation

### From conda-forge

```
conda install -c conda-forge upkie_description
```

### From PyPI

```
pip install upkie_description
```

## Usage

This module helps retrieve Upkie's model from a Python program. Import it by:

```python
import upkie_description
```

You can then load the description directly in a robotics framework, for instance in [Pinocchio](https://github.com/stack-of-tasks/pinocchio):

```python
robot = upkie_description.load_in_pinocchio()
```

The module also provides the following paths:

<dl>
    <dt>
        <code>upkie_description.PATH</code>
    </dt>
    <dd>
        Path to the "upkie_description" folder itself.
    </dd>
    <dt>
        <code>upkie_description.MESHES_PATH</code>
    </dt>
    <dd>
        Path to the "meshes" folder.
    </dd>
    <dt>
        <code>upkie_description.URDF_PATH</code>
    </dt>
    <dd>
        Path to the URDF file of the model.
    </dd>
</dl>

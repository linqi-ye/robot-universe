# Upkie wheeled biped description

<img src="https://user-images.githubusercontent.com/1189580/169594012-2d685579-2b66-4470-9def-57bd0656b420.png" align="right" width="300">

[![Packaging](https://img.shields.io/github/actions/workflow/status/upkie/upkie_description/packaging.yml?branch=main)](https://github.com/upkie/upkie_description/actions)
[![Documentation](https://img.shields.io/badge/docs-wiki-brightgreen?style=flat)](https://upkie.github.io/upkie/index.html)
[![Conda version](https://img.shields.io/conda/vn/conda-forge/upkie_description.svg)](https://anaconda.org/conda-forge/upkie_description)
[![PyPI version](https://img.shields.io/pypi/v/upkie_description)](https://pypi.org/project/upkie_description/)

URDF descriptions for [Upkie](https://github.com/upkie/upkie) wheeled bipeds.

The Upkie documentation also details properties of this model and how it was constructed:

- [Joint limits](https://upkie.github.io/upkie/kinematics.html#joint-limits)
- [Location of IMU and anchor frames](https://upkie.github.io/upkie/kinematics.html#frames)
- [Model conventions](https://upkie.github.io/upkie/kinematics.html#model-conventions)

## Python module

The description can be loaded directly in various Python robotics frameworks, for instance:

```python
import upkie_description

robot = upkie_description.load_in_pinocchio()
```

Check out the [Python readme](python/README.md) for more details.

## Citation

If you use this description in your works, please cite the Upkie project and its contributors:

```bibtex
@software{upkie,
  title = {{Upkie open source wheeled biped robot}},
  author = {Caron, St\'{e}phane and Perrin-Gilbert, Nicolas and Ledoux, Viviane and G\"{o}kbakan, \"{Umit} Bora and Raverdy, Pierre-Guillaume and Raffin, Antonin and Tordjman--Levavasseur, Valentin},
  url = {https://github.com/upkie/upkie},
  license = {Apache-2.0},
  version = {6.0.0},
  year = {2024}
}
```

## License

The Apache 2.0 license applies to all files in this repository, to the exception of the [wheel\_tire](meshes/wheel_tire) mesh which is under the CC BY 4.0 license. Meshes from [mjbots](meshes/mjbots), as well as the torso meshes derived from the [mjbots quad](https://github.com/mjbots/quad), are also Apache 2.0.

## See also

- [Upkie](https://github.com/upkie/upkie): main repository for the robot's hardware and software
- [3D printed parts](https://github.com/upkie/upkie_parts): CAD files and add-ons for the robot
- [xacrodoc](https://github.com/adamheins/xacrodoc): compile xacro files without ROS, helped this project a lot!

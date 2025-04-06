# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- Bazel: Export all URDF variants in `upkie_description` filegroup
- examples: Show robot in a custom configuration
- examples: Visualize wheel frames while both legs move around

### Changed

- docs: Update documentation links
- examples: Rename example to "show in neutral configuration"

### Fixed

- Frame visualization example

## [2.1.0] - 2024-07-31

### Added

- Add ``--variant`` keyword argument to all examples
- Add ``camera_eye`` frame to the camera variant
- Add an empty visual box to each virtual link
- CICD: Add Python workflow

### Changed

- Replace the `virtual_link_inertia` Xacro macro by `virtual_link`

### Fixed

- Rendering of the IMU frame orientation in MeshCat
- Rendering of virtual-link frames in MeshCat

## [2.0.0] - 2024-07-25

### Added

- CICD: Add changelog workflow
- CICD: Add packaging workflow
- Camera variant of the URDF
- python: Add "variant" keyword argument to `load_in_pinocchio`
- Added camera support mass and camera to the "camera" variant
- Added camera eye to the "camera" variant
### Changed

- **Breaking:** Uppercase paths in Python for consistency with `robot_descriptions.py`
- Refactor the URDF generation process using Xacro and `xacrodoc`

## [1.6.0] - 2024-07-05

### Added

- Example to display the center of mass
- Filter frame names in frame display example

### Changed

- Change parent of "imu" frame from "base" to "torso"
- Increase velocity and torque limits to motor specs
- Make "base" a virtual link
- Use the same syntax everywhere for self-closing markups
- Zero out inertia matrices of virtual links

### Removed

- Remove dummy "orange" color from materials
- Remove collision meshes of virtual links
- Remove visualization meshes of virtual links

## [1.5.0] - 2024-02-20

### Changed

- Shorten source headers using SPDX license identifiers
- Update project links

### Fixed

- Make Pinocchio an optional dependency

## [1.4.0] - 2023-07-19

### Added

- Summary of joint limits in the README

### Changed

- Hip position and velocity limits
- IMU orientation to be consistent with assembly instructions
- Knee position and velocity limits
- Wheel velocity limits

## [1.3.0] - 2023-04-05

### Added

- Example: display joint frames from the URDF description
- Example: load Upkie in Pinocchio

### Changed

- Update velocity limits to more reasonable values

## [1.2.0] - 2023-03-14

### Added

- Example: display frames from the URDF description
- Function to load the description directly in Pinocchio

### Fixed

- Orientation of the IMU frame (thanks to @G-Levine)

## [1.1.1] - 2023-02-13

### Changed

- Rename "chassis" to "torso"

## [1.1.0] - 2022-05-23

### Added

- Build file to use the model in Bazel projects
- Illustration to the README
- Link to the mjbots quad chassis
- Python distribution of the model

### Changed

- Relicense the project to Apache 2.0
- Turn IMU 180 degrees around the base yaw axis

### Fixed

- Cleaned dangling Bazel reference to ``//tools/lint``
- Links to Printables.com
- Project name for CMake

## [1.0.0] - 2022-03-17

Initial robot description.

### Added

- Mesh files for the legs, wheel tire and mjbots parts
- ROS build and package files
- Robot description (kinematics, inertias, collisions) in URDF
- Scripts to compute box and cylinder inertias

[unreleased]: https://github.com/upkie/upkie_description/compare/v2.1.0...HEAD
[2.1.0]: https://github.com/upkie/upkie_description/compare/v2.0.0...v2.1.0
[2.0.0]: https://github.com/upkie/upkie_description/compare/v1.6.0...v2.0.0
[1.6.0]: https://github.com/upkie/upkie_description/compare/v1.5.0...v1.6.0
[1.5.0]: https://github.com/upkie/upkie_description/compare/v1.4.0...v1.5.0
[1.4.0]: https://github.com/upkie/upkie_description/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/upkie/upkie_description/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/upkie/upkie_description/compare/v1.1.1...v1.2.0
[1.1.1]: https://github.com/upkie/upkie_description/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/upkie/upkie_description/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/upkie/upkie_description/releases/tag/v1.0.0

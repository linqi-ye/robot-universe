#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

"""Load Upkie's description in Pinocchio."""

import os
from typing import Optional, Union

import pinocchio as pin

from .paths import PATH

PinocchioJoint = Union[
    pin.JointModelRX,
    pin.JointModelRY,
    pin.JointModelRZ,
    pin.JointModelPX,
    pin.JointModelPY,
    pin.JointModelPZ,
    pin.JointModelFreeFlyer,
    pin.JointModelSpherical,
    pin.JointModelSphericalZYX,
    pin.JointModelPlanar,
    pin.JointModelTranslation,
]


def load_in_pinocchio(
    root_joint: Optional[PinocchioJoint] = None,
    variant: str = "",
) -> pin.RobotWrapper:
    """Load Upkie's description in Pinocchio.

    Args:
        root_joint (optional): First joint of the kinematic chain, for example
            a free flyer between the floating base of a mobile robot and an
            inertial frame. Defaults to no joint, i.e., a fixed base.
        variant (optional): Variant of the robot description to load,
            corresponding to the suffix of the URDF file. For example, set to
            "camera" to load ``upkie_camera.urdf``.

    Returns:
        Robot model for Pinocchio.
    """
    filename = f"upkie_{variant}.urdf" if variant else "upkie.urdf"
    urdf_path = os.path.join(PATH, "urdf", filename)
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=[PATH, os.path.dirname(PATH)],
        root_joint=root_joint,
    )
    return robot

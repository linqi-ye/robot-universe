#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2022 Stéphane Caron

"""
Paths to the URDF package and main description for Upkie wheeled bipeds.
"""

import os

# Path to upkie_description
PATH = os.path.dirname(os.path.realpath(__file__))

# Path to the meshes folder
MESHES_PATH = os.path.join(PATH, "meshes")

# Path to the robot's URDF
URDF_PATH = os.path.join(PATH, "urdf", "upkie.urdf")

__all__ = [
    "MESHES_PATH",
    "PATH",
    "URDF_PATH",
]

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""
Display the center of mass and visual model of the robot.
"""

import argparse
import time

import numpy as np
import upkie_description
from pinocchio.visualize import MeshcatVisualizer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left_hip", help="Left hip joint angle in [rad]")
    parser.add_argument("left_knee", help="Left knee joint angle in [rad]")
    parser.add_argument("left_wheel", help="Left wheel joint angle in [rad]")
    parser.add_argument("right_hip", help="Right hip joint angle in [rad]")
    parser.add_argument("right_knee", help="Right knee joint angle in [rad]")
    parser.add_argument("right_wheel", help="Right wheel joint angle in [rad]")
    args = parser.parse_args()

    robot = upkie_description.load_in_pinocchio(root_joint=None)
    q = np.array([float(pair[1]) for pair in args._get_kwargs()])

    robot.setVisualizer(MeshcatVisualizer())
    robot.initViewer(open=True)
    robot.loadViewerModel()
    robot.display(q)

    time.sleep(10.0)  # avoid terminating too fast

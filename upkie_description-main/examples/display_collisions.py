#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""
Display the robot description along and its collision meshes.
"""

import argparse
import time

import upkie_description
from pinocchio.visualize import MeshcatVisualizer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant",
        help="variant of the robot description to load",
    )
    args = parser.parse_args()

    robot = upkie_description.load_in_pinocchio(variant=args.variant)
    robot.setVisualizer(MeshcatVisualizer())
    robot.initViewer(open=True)
    robot.loadViewerModel(collision_color=(1.0, 0.0, 0.0, 0.3))
    robot.viz.displayCollisions(True)
    robot.display(robot.q0)

    time.sleep(1.0)  # avoid terminating too fast

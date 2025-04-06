#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Load Upkie in Pinocchio and display its visual model in MeshCat."""

import argparse
import time

import pinocchio as pin
import upkie_description
from pinocchio.visualize import MeshcatVisualizer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant",
        help="variant of the robot description to load",
    )
    args = parser.parse_args()

    label = "robot"
    if args.variant:
        label = f"'{args.variant}' variant of the robot"
    robot = upkie_description.load_in_pinocchio(
        # NB: we add a free-flyer so that the torso mass is included
        root_joint=pin.JointModelFreeFlyer(),
        variant=args.variant,
    )
    robot.setVisualizer(MeshcatVisualizer())
    robot.initViewer(open=True)
    robot.loadViewerModel()
    robot.display(robot.q0)

    time.sleep(1.0)  # wait long enough for MeshCat to fire up

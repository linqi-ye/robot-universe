#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""
Display all frames from the robot description.

This example relies on the following dependencies:

- `MeshCat <https://github.com/rdeits/meshcat-python>`_
- `meshcat-shapes <https://github.com/stephane-caron/meshcat-shapes>`_
- `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`_
"""

import argparse
import time

import meshcat_shapes
import numpy as np
import pinocchio as pin
from meshcat import transformations
from pinocchio.visualize import MeshcatVisualizer

import upkie_description

FRAME_SCALE = 1.0

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
    robot.loadViewerModel(visual_color=[1.0, 1.0, 1.0, 0.3])
    viewer = robot.viz.viewer
    q = robot.q0.copy()

    display_frames = set(
        [
            "left_anchor",
            "left_contact",
            "right_anchor",
            "right_contact",
        ]
    )
    frame_ids = [
        frame_id
        for frame_id, frame in enumerate(robot.model.frames)
        if frame.name in display_frames
    ]
    robot.viz.displayFrames(True, frame_ids=frame_ids or None)
    robot.viz.updateFrames()
    for frame in robot.model.frames:
        if frame.name in display_frames:
            handle = viewer["pinocchio"]["frames"][frame.name]
            meshcat_shapes.frame(
                handle["frame"],
                axis_length=0.05 * FRAME_SCALE,
                axis_thickness=0.002 * FRAME_SCALE,
                opacity=0.8,
                origin_radius=0.005,
            )
            meshcat_shapes.textarea(
                handle["text"],
                text=frame.name,
                width=0.05,
                height=0.05,
                font_size=100,
            )

    left_wheel_jid = robot.model.getJointId("left_wheel")
    left_wheel_joint = robot.model.joints[left_wheel_jid]
    right_wheel_jid = robot.model.getJointId("right_wheel")
    right_wheel_joint = robot.model.joints[right_wheel_jid]
    names = [
        f"{side}_{kind}"
        for side in ["left", "right"]
        for kind in ["hip", "knee"]
    ]
    upper_leg_joints = [
        robot.model.joints[robot.model.getJointId(joint_name)]
        for joint_name in names
    ]

    dt = 0.01  # [s]
    wheel_velocity = 1.0  # [rad] / [s]
    for i in range(1000):
        robot.display(q)
        pin.updateFramePlacements(robot.model, robot.data)
        q[left_wheel_joint.idx_q] += wheel_velocity * dt
        q[right_wheel_joint.idx_q] += wheel_velocity * dt
        for joint in upper_leg_joints:
            sign = +1 if i % 500 < 500 else -1
            q[joint.idx_q] = sign * 0.5 * np.sin(i * dt)
        for frame in robot.model.frames:
            if frame.name not in display_frames:
                continue
            handle = viewer["pinocchio"]["frames"][frame.name]
            Rx = transformations.rotation_matrix(0.5 * np.pi, [1.0, 0.0, 0.0])
            Rz = transformations.rotation_matrix(0.5 * np.pi, [0.0, 0.0, 1.0])
            trans = transformations.translation_matrix(
                [0.0, 0.0, 0.005 * FRAME_SCALE]
            )
            handle["text"].set_transform(trans @ Rz @ Rx)
        time.sleep(dt)

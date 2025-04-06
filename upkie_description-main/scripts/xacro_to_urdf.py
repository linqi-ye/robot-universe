#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""
Convert a Xacro description to URDF.
"""

import argparse

from xacrodoc import XacroDoc


def parse_command_line_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Namespace resulting from parsing command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input_file",
        help="path to the input Xacro file",
    )
    parser.add_argument(
        "output_file",
        help="path to the output URDF file",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_command_line_arguments()
    xacro_doc = XacroDoc.from_file(args.input_file, resolve_packages=False)
    xacro_doc.to_urdf_file(args.output_file)

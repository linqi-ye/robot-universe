# @file test_icub_models.py
# @authors Giulio Romualdi
# @copyright 2022 Istituto Italiano di Tecnologia Released under the terms of the Creative Commons Attribution Share Alike 4.0 International


import pytest

import icub_models


def test_models_path():
    assert icub_models.get_models_path().is_dir()


def test_model_files():
    for robot_name in icub_models.get_robot_names():
        assert icub_models.get_model_file(robot_name).is_file()

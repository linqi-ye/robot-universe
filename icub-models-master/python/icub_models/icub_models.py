# @file icub_models.py
# @authors Giulio Romualdi
# @copyright 2022 Istituto Italiano di Tecnologia Released under the terms of the Creative Commons Attribution Share Alike 4.0 International

import os
import pathlib
from typing import List


def get_models_path() -> pathlib.Path:
    relative_path = "@ICUB_MODELS_CXX_PYTHON_INSTALL_RELATIVE_DIR@"
    # If relative_path starts with @, it is a not escaped string
    # from CMake, i.e. that means that the file was installed via setup.py
    # let's substitute relative_path with its default value
    if relative_path.startswith("@"):
        # This is required to handle the difference between the unix and
        # windows folder structure
        if os.name == "nt":
            root = pathlib.Path(__file__).parent.parent.parent.parent
        else:
            root = pathlib.Path(__file__).parent.parent.parent.parent.parent
    else:
        root = pathlib.Path(__file__).parent / pathlib.Path(relative_path)

    return (root / "share" / "iCub" / "robots").resolve()


def get_robot_names() -> List[str]:
    return [d.name for d in get_models_path().iterdir() if d.is_dir()]


def get_model_file(robot_name: str) -> pathlib.Path:
    model_file = get_models_path() / robot_name / "model.urdf"

    if model_file.is_file():
        return model_file
    else:
        raise FileNotFoundError(f"Model file for robot {robot_name} not found.")

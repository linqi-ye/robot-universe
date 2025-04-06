# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# Released under the terms of the BSD 3-Clause License

from setuptools import setup
from pathlib import Path


if __name__ == "__main__":

    data_files = [
        (str("share" / path.parent), [str(path)])
        for path in Path("iCub").glob("**/*.*")
    ]

    setup(data_files=data_files)

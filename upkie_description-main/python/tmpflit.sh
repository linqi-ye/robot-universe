#!/bin/sh
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

PROJECT_NAME=upkie_description

if [ "$#" -lt 1 ]; then
    echo "usage: same as flit"
    echo ""
    echo "examples:"
    echo "  $0 build"
    echo "  $0 publish --repository testpypi"
    echo ""
    flit --help
    exit 1
fi

SCRIPT=$(realpath "$0")
SCRIPTDIR=$(dirname "${SCRIPT}")
COMMAND=$@
SRCDIR=${SCRIPTDIR}/..
TMPDIR=$(mktemp -d)

echo "[debug] COMMAND=${COMMAND}"
echo "[debug] SRCDIR=${SRCDIR}"
echo "[debug] TMPDIR=${TMPDIR}"

for folder in meshes urdf; do
    mkdir -p ${TMPDIR}/${PROJECT_NAME}/$(dirname ${folder})
    cp -r ${SRCDIR}/${folder} ${TMPDIR}/${PROJECT_NAME}/${folder}
done

cp ${SCRIPTDIR}/${PROJECT_NAME}/*.py ${TMPDIR}/${PROJECT_NAME}/
cp ${SCRIPTDIR}/README.md ${TMPDIR}/README.md
cp ${SCRIPTDIR}/pyproject.toml ${TMPDIR}/pyproject.toml

cd ${TMPDIR} && flit ${COMMAND}

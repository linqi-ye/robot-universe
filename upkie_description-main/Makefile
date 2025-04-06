# SPDX-License-Identifier: Apache-2.0

OUTPUTS = urdf/upkie.urdf urdf/upkie_camera.urdf

.PHONY: all
all: $(OUTPUTS)

.PHONY: clean
clean:
	rm -f $(OUTPUTS)

.PHONY: rebuild
rebuild: clean all

urdf/%.urdf: xacro/%.xacro
	python scripts/xacro_to_urdf.py $< $@

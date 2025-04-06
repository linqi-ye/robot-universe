# JVRC humanoid robot model

<img src="https://scaron.info/images/jvrc1-model.png" width="250" align="right" />

This package contains the robot description for the JVRC-1 model, a virtual humanoid robot released for education and research.

## History

The model files in this repository come from [mc\_rtc\_data](https://github.com/jrl-umi3218/mc_rtc_data), a collection of robot and environment models distributed with the [mc\_rtc](https://jrl-umi3218.github.io/mc_rtc/) framework. The JVRC-1 model was initially created in VRML format at [jvrc/model](https://github.com/jvrc/model) and converted to URDF using [simtrans](https://github.com/fkanehiro/simtrans).

## Show and tell

Feel free to [open a PR](https://github.com/stephane-caron/jvrc_description/pulls) to share what you have done with this model.

| Date | Topic | Screenshot |
|------|-------|------------|
| 2019 | [Docker image of a walking controller in simulation](https://hub.docker.com/r/stephanecaron/lipm_walking_controller) — Snapshot of a whole-body controller used on the HRP-4 humanoid to walk and climb stairs. This image includes a fully functional build and simulation environment where you can try the controller on the JVRC model.  | <img src="https://user-images.githubusercontent.com/1189580/69481155-04de3500-0e52-11ea-91cc-02d05d504ffa.png" width="250"> |
| 2016 | [Task-based inverse kinematics in Python](https://scaron.info/robot-locomotion/inverse-kinematics.html) — A whole-body inverse kinematics based on the weight-prioritized multi-task framework. Tasks include foot-surface contacts, center of mass or centroidal momentum control. Depends on OpenRAVE. | <img src="https://scaron.info/images/weighted-issue.png" width="250"> |

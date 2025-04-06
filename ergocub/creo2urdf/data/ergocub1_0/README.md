

> [!warning]
>
> The urdf of `ergoCubGazeboV1/SN000` has never been generated with [`creo2urdf`](https://github.com/icub-tech-iit/creo2urdf), it has been created in the transition period when for using [`simmechanics_to_urdf`](https://github.com/robotology/simmechanics_to_urdf) was needed to port the CAD [from CREO9 to CREO7](https://github.com/robotology/simmechanics-to-urdf/issues/55) thus we minimized the fixes on the CAD, patching it via software(https://github.com/icub-tech-iit/ergocub-software/issues/106)
> 
> The simulation model CAD has not been updated for a while, we are keeping the yaml up-to-date but if changes are needed in the robot model it is better to apply them directly to the urdf instead of generating with `creo2urdf`.

This is the mapping between the CAD files, the files contained in this folder and the files contained in `ergocub-software/urdf/ergoCub/robots`:

| CAD model in `cad-mechanics` | yaml file | csv file | YARP_ROBOT_NAME |
|:----------:|:---------:|:----------:|:--------------:|
| `cad-mechanics/projects/simulation_model/ergocub_1-0/sim_ergocub_1-0.asm.1` | `ergocub-software/urdf/creo2urdf/data/ergocub1_0/ERGOCUB_all_options.yaml` | `ergocub-software/urdf/creo2urdf/data/common/ERGOCUB_joint_all_parameters.csv` | `ergoCubSN000`  |
| `cad-mechanics/projects/simulation_model/ergocub_1-0/sim_ergocub_1-0.asm.1` | `ergocub-software/urdf/creo2urdf/data/ergocub1_0/ERGOCUB_all_options_gazebo.yaml` | `ergocub-software/urdf/creo2urdf/data/common/ERGOCUB_joint_all_parameters.csv` | `ergoCubGazeboV1`  |
| `cad-mechanics/projects/simulation_model/ergocub_1-0/sim_ergocub_1-0.asm.1` | `ergocub-software/urdf/creo2urdf/data/ergocub1_0/ERGOCUB_all_options_minContacts.yaml` | `ergocub-software/urdf/creo2urdf/data/common/ERGOCUB_joint_all_parameters.csv` | `ergoCubGazeboV1_minContacts`  |

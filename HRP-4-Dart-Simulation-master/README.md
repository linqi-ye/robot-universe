# HRP-4-Dart-Simulation

Dynamic simulation of the position controlled HRP-4 robot performing a dynamic walk. The gait generation algorithm is designed to perform automatic footstep placement and timing adaptation to allow a strong degree of robustness. The code is offered for  benchmarking, review, education or as a simple example of simulation on Dart.
Please contact the authors for any question.

DART REMARKS

Please install Dart (Dynamic Animation and Robotics Toolkit) https://dartsim.github.io/ 
It is strongly suggested to use Ubuntu 16.04/18.04 (in the sense that the simulation has been developed using these OS).
ADVICE: install Dart from source by following the described procedure on Dart Website. Please type 
 
             git checkout tags/v6.6.0       
                  or
             git checkout tags/v6.6.1 
                  
to not checkout the latest tag of DART 6, otherwise you may have issues in running the code. The problem is only related to Dart, so if you are already practiced with it, feel free to use any version.  
ATTENTION: the mesh paths in HRP-4 urdf file must be modified and the Eigen directory should be decompressed if you need it.
The folder in which I have all the code and where I compile it is also added in the repository as an example, named "HRP4_folder.zip".

SIMULATION REMARKS

The core component of the code are the Controller and the MPC solver. The MPC solver provides a CoM trajectory, expressed w.r.t. the support foot, to be kinematically tracked by the Controller, which is also responsible of the swing foot trajectory generation, the measurement collection and some further tools (data storage, external pushes management, etc.).
Inverse Kinematic is addressed with a simple weighted pseudo inverse method. Jacobians are kindly provided by Dart.  

The MPC gait generation is the peculiar part of the simulation. The simulation implements the IS-MPC (Intrinsically Stable MPC for humanoid gait generation developed at Diag Robotics Lab). IS-MPC is characterized by a wide set of interesting properties such as a mathematically demonstrable internal stability (no CoM divergence w.r.t. the ZMP).Moreover, the so called "feasibility driven timing adaptation" is also part of the code and can be activated. The following papers are suggested for further details

1) N. Scianca, M. Cognetti, D. De Simone, L. Lanari and G. Oriolo, "Intrinsically stable MPC for humanoid gait generation," 2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids), Cancun, 2016, pp. 601-606;

2) N. Scianca, D. De Simone, L. Lanari and G. Oriolo, MPC for Humanoid Gait Generation: Stability and Feasibility.



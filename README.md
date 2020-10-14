# 🤖 Motion Planning for the KUKA LBR iiwa arm
The project implements Motion Planning for the KUKA LBR iiwa robotic arm.
![](https://github.com/ariannagavioli/Motion-Planning-for-KUKA-LBR/blob/main/resources/KUKA_presentation.gif)

Given a set of points the end effector must go through and an obstacle, the goal of the project is to implement a Motion Planning, performing null-space reconfiguration, using MATLAB and the CoppeliaSim simulator.
### Prerequisites and how to run it
In order to run the project, MATLAB 2020 and [CoppeliaSim robot simulator](https://www.coppeliarobotics.com/downloads) must be installed. Add the libraries necessary for the communication with CoppeliaSim, copying them from the CoppeliaSim installation folder:
* `remApi.m`
* `remoteApi.so`
* `remoteApiProto.m`

in a subfolder `vrep_lib` in the current workspace.

Open the scene `vrep_model.ttt` defined in the `vrep_sim` sub-folder in the CoppeliaSim simulator and execute `main.m` from the MATLAB environment.
### Results 
Two possible motion controllers were used, one being a Cartesian Motion controller and the second being a Joint Motion controller. It's possible to configure which controller to use during the simulation by setting the `controller` variabe in `main.m` (line 17) either to `'cartesian'` or `'joint'`.

The following videos show two versions of motion planning for the arm: on the left, the standard implementation without obstacle-avoidance, on the right, the obstacle-free motion implemented by exploiting 5 control points placed on the arm itself. The shown results are obtained using cartesian motion control, however the simulation can be tested using joint motion control as well.

![](https://github.com/ariannagavioli/Motion-Planning-for-KUKA-LBR/blob/main/resources/cartesian_std_side.gif)
![](https://github.com/ariannagavioli/Motion-Planning-for-KUKA-LBR/blob/main/resources/cartesian_augmented_side.gif)

For further information, please consult the [project presentation](https://docs.google.com/presentation/d/1LdzxDmdqFgt6EMwRkEascuR0Q9dO_xD2QAOD-iXdemc/edit?usp=sharing).

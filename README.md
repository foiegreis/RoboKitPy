# RoboKitPy
**Robotics toolbox in Python**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">
	<img src="docs/imgs/robokitpy_logo1.png" width="400">
</p>


## Contents

- [RoboKitPy](#1)
- [Installation](#2)
- [Tutorials](#3)
- [Code Examples](#4)


<a id='1'></a>

## RoboKitPy

**RoboKitPy** is an open-source Python library designed to bridge the gap between theoretical robotics concepts and practical application. The repository serves as a comprehensive toolkit for both students and professionals, facilitating an easier understanding of complex robotics topics through well-documented and executable Python code.

### Current Features
RoboKitPy currently supports a wide range of functionalities centered around the core areas of robotics:

- **Forward Kinematics**: Compute the position and orientation of the robot's end-effector based on given joint parameters.
- **Inverse Kinematics**: Determine the joint parameters necessary to achieve a desired end-effector position and orientation.
- **Differential Forward Kinematics**: Analyze the velocity relationships between joint velocities and end-effector velocities, featuring the calculation of manipulability and force ellipsoids to assess the performance and capability of robotic manipulators.
- **Statics**: Handle the static balance and force transmission within robotic structures.

### Supported Robotic Models
The library includes implementations for various robotic configurations, both planar and spatial, making it versatile for different educational and research needs:

- Planar Robots: 
  - RR 
  - RRR 
- Spatial Robots:
  - RRR and RRR Coplanar configurations
  - Puma560
  - PRRRRP
  - UR5e

### Upcoming Features
RoboKitPy is actively being developed with future updates aimed at expanding its capabilities:

- **Dynamics**: In-depth analysis of motion dynamics for better control and efficiency.
- **Trajectory Generation**: Algorithms for smooth path planning for robot trajectories.
- **Motion Planning and Obstacle Avoidance**: Enhanced algorithms for navigating through complex environments.
- **Behavioral Planning**: Integration of decision-making processes in robotic tasks.
- **SLAM**: Tools for robotic mapping and navigation in unknown environments.
- **Mobile Robots**: Expanding the library to include autonomous ground vehicles.
- **Navigation, Sensor Fusion, and Perception**: Advanced modules for robust robotic perception and interaction with the environment. 

### Educational Companion
RoboKitPy runs in parallel with theoretical posts and tutorials available at my website https://www.roboticsunveiled.com.

<p align="center">
	<img src="docs/imgs/roboticsunveiled.png" width="500">
</p>

### Contribution and Community
We welcome contributions from the community to help grow RoboKitPy and make robotics more accessible to a broader audience. Whether you're interested in adding new models, improving existing algorithms, or providing educational content, your input is valuable.


<a id='2'></a>
## Installation

Requirements: Python >= 3.6

### Using pip

**Coming Soon**
```shell script
pip3 install robokitpy
```

### From GitHub

```shell script
git clone https://github.com/foiegreis/RoboKitPy.git
cd robokitpy
pip3 install -e .
```


<a id='3'></a>
## Tutorials
The [`examples`](https://github.com/foiegreis/RoboKitPy/tree/main/robokitpy/examples) folder contains some examples on the functionalities of the package. 


<a id='4'></a>
## Code Examples

We will load a model of the Franka-Emika Panda robot defined by a URDF file

```python
from robokitpy.core.fk import *
from robokitpy.models.spatial.ur5e import UR5e

# UR5e
# Known joint configuration θ1-θ6
theta = [0.7854, -0.7854, 0.7854, -1.5708, -1.5708, 0.7854]

# Model
model = UR5e()

# FK DH
dh_table = model.DH(theta)

# FORWARD KINEMATICS applying DH
fk_dh = fk_dh(dh_table)
print(f"\nForward Kinematics T0{dh_table.shape[0]} applying DH for the configuration {theta}: \n{fk_dh}")

# FK POE
M = model.M()
s_list = model.S()
b_list = model.B()

# FORWARD KINEMATICS applying PoE SPACE FORM
fk_s = fk_space(M, s_list, theta)
print(f"\nForward Kinematics T0{s_list.shape[0]} applying PoE Space Form for the configuration {theta}: \n{fk_s}")

# FORWARD KINEMATICS applying PoE BODY FORM
fk_b = fk_body(M, b_list, theta)
print(f"\nForward Kinematics T0{b_list.shape[0]} applying PoE Space Form for the configuration {theta}: \n{fk_b}")
```

This code will return the forward kinematics of the UR5e robot, given the joint configuration:

```
Forward Kinematics T06 applying DH for the configuration [0.7854, -0.7854, 0.7854, -1.5708, -1.5708, 0.7854]: 
[[-0.     -1.      0.      0.4798]
 [-1.      0.      0.      0.6339]
 [ 0.     -0.     -1.      0.3075]
 [ 0.      0.      0.      1.    ]]

Forward Kinematics T06 applying PoE Space Form for the configuration [0.7854, -0.7854, 0.7854, -1.5708, -1.5708, 0.7854]: 
[[ 0.     -1.      0.      0.4798]
 [-1.      0.      0.      0.6339]
 [ 0.     -0.     -1.      0.3075]
 [ 0.      0.      0.      1.    ]]

Forward Kinematics T06 applying PoE Space Form for the configuration [0.7854, -0.7854, 0.7854, -1.5708, -1.5708, 0.7854]: 
[[ 0.     -1.      0.      0.4798]
 [-1.      0.      0.      0.6339]
 [ 0.     -0.     -1.      0.3075]
 [ 0.      0.      0.      1.    ]]

```

For the inverse kinematics, the code will have a similar fashion:

```python
from robokitpy.core.ik import *
from robokitpy.models.spatial.ur5e import UR5e

# Desired end-effector pose
Tsd = np.array([[ 0, -1, 0, 0.4798],
                 [-1, 0, 0, 0.6339],
                 [ 0, 0, -1, 0.3075],
                 [ 0, 0, 0, 1]])
# Initial guess
thetalist0 = np.array([np.pi/4, -np.pi/4, np.pi/4, -np.pi/4, -np.pi/4, np.pi/4])

# Thresholds
eps_w = 0.001
eps_v = 0.0001

# model
model = UR5e()
M = model.M()
s_list = model.S()
b_list = model.B()

max_iterations = 20

# INVERSE KINEMATICS applying PoE SPACE FORM
ik_s, success = ik_space(M, s_list, Tsd, thetalist0, eps_w, eps_v, max_iterations)
print(f"\nInverse kinematics in Space form: \n{ik_s}")
print("success: ", success)

# INVERSE KINEMATICS applying PoE BODY FORM
ik_b, success = ik_body(M, b_list, Tsd, thetalist0, eps_w, eps_v, max_iterations)
print(f"\nInverse kinematics in Body form: \n{ik_b}")
print("success: ", success)

print("\nExpected result:\n",  np.round([np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, np.pi/4], 8))
```

That will result in:

```shell
Inverse kinematics in Space form: 
[ 0.78546815 -0.78537123  0.78525326 -1.57096848 -1.57078516  0.78542622]
success:  True

Inverse kinematics in Body form: 
[ 0.78536466 -0.78552649  0.78560546 -1.57116855 -1.5708196   0.78532415]
success:  True

Expected result:
 [ 0.78539816 -0.78539816  0.78539816 -1.57079633 -1.57079633  0.78539816]
```

We can also plot the robot, showing the Velocity (Manipulability) and Force Ellipsoids

```python
from robokitpy.models.spatial.ur5e import UR5e
from robokitpy.plot.plot_3d import *

thetalist = np.array([np.pi/4, -np.pi/4, np.pi/4, np.pi/4, -np.pi/4, np.pi/4])

model = UR5e()

plot_robot_3d(model, thetalist, velocity_ellipsoid=False, force_ellipsoid=True, linear=False, scale=0.1)
```
That will generate this 3D plot


<p align="center">
	<img src="docs/imgs/ur5e.png" with="300">
</p>



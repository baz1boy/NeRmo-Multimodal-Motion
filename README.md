# NeRmo Mouse Robot

![Robot Simulation](images/NeRmo_Overview.jpg)

The **video demonstration** shows here:

[![Video Demonstration](https://img.shields.io/youtube/channel/views/:UClva9bOC5x7I6EH2yGsfpeg)](https://www.youtube.com/watch?v=iS-gbdyKS5s&list=PLG0yEiqorTkgIp97KAagTFfdkPwRGbhUh&index=63)

A 12-DOF legged small-sized quadruped robot capable of multi-modal motion modes. Different motion modes are tested in both simulation and real-world environments.

All files are configured based on the MuJoCo environment in Python, starting with the modeling and motion testing of a 2D planar leg and gradually progressing to the simulation of the entire robot, followed by real-world robotic testing.

## Features
- **12-DOF leg structure:** based on the tri-segemented linkage-based leg.
- **Multi-modal motion control:** including **basic motion**, **turning in place**, **self-recovery from falling**, **squeezing thourgh small openings**, and **surmounting obstacles**. The jump motion is also included but has not achieved effective performance.

## Setup
### Prerequisites
- Ubuntu 20.04/22.04
- Python 3.8+
- MuJoCo

### Simulation Configuraiton
The XML files are stored in the `models` folder, with each leg individually modeled. In the main body XML file, all components—including the legs, head, body, tail, and environment are assembled. The model structures created in MuJoCo are replaced with CAD STL files located in the `meshes` folder.

### Test Setup
Each motion test has a dedicated test script `sim_test_xxx.py`. In the scripts, it is important to ensure that the appropriate MuJoCo modeling environment files are imported. 

For certain tests, such as squeezing, which includes tests for **low-height openings** and **narrow-width openings**, make sure to modify the corresponding environment configuration in `robot_body_squeezing.xml` in `models` accordingly.

## Test Program
`sim_test_turning.py`: The turning test marks the beginning of the turning in place testing. The preceding commented-out sections correspond to tests of twisting motion, including roll, pitch, and yaw rotations.

`sim_test_recover.py`: It includes the simulation of side-lying and back-lying postures, and unnecessary test cases need to be commented out.

`sim_test_squeezing.py`: It is used to test the robot's ability to pass through low-height and narrow-width openings. The testing target can be selected by commenting or uncommenting **"Through a low hole"** and **"Through a narrow hole"** accordingly. Depending on the selected target, the corresponding environment modeling—low test or narrow test—needs to be specified in `models/test environment/squeezing_scene.xml`.

`sim_test_lower_leaning.py`: In the **"Trajectory Points"** part, select the start and end positions of the foot end. Positions **1** to **4** correspond to lower posture testing, while **point 5** is used for **leaning posture testing**. The parameter **s** is used to adjust the **gait stride length** in the lower posture. Modify the height of passage in `models/test environment/move_scene.xml` to test the **minimum passable height** based on the body height data collected by the sensors when the robot operates in **lower posture** or **leaning posture**.

`sim_test_surmounting.py`: Adjust the obstacle dimension in `models/test environment/surmounting_scene.xml` to test the maximum surmountable height.

## Kinematics
All kinematics calculations follow the unit system used in MuJoCo, meaning that all lengths are measured in meters. The leg dimension data is read from `leg_Data.py`.

`leg_Joint.py` computes the **forward kinematics**, while `leg_IK.py` is used for **inverse kinematics** calculations. If errors occur during testing, it may be necessary to slightly adjust the value of `q_guess` to improve convergence.

## Control Architecture
In `sim_test_xxx.py`, the process always begins with initialization, transitioning the robot from the modeled position to the initial running posture `init_pleg`, and converting it to **servo control values** `q_init`.

### ControL Speed
The motion speed is adjusted by dividing a planned trajectory into `n_step` sets of control values. 

For example, for a motion defined by a start point and an end point, the value of `n_step` determines the motion speed along the trajectory between these two points, given that the step duration is fixed.

### Control Variables
Some control variables need to be modified in `sim_controller.py`, such as the step angle in the turning motion, rather than directly specifying the desired values in the test script.

The required leg control data for a motion is obtained from the **motion pattern library** in `body_posture.py`. The final optimization is then performed in **trajectory optimization** within `sim_controller.py`, where additional control for the head joints is also incorporated.

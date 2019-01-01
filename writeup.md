## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png``````
[robot-joint-space-1]: ./misc_images/robot-joint-space-1.png
[robot-joint-space-2]: ./misc_images/robot-joint-space-2.png
[inverse-calculation]: ./misc_images/inverse-calculation.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Following is a snapshot from the pick-and-place simulator running in demo mode with the **Kuka KR210** robotic arm. 
We analyze the kinematics of the robotic arm using this demo.

![alt text][image1]


The XML file named `kr210.urdf.xacro` contains the URDF (Unified Robot Description Format) definition for the Kuka KR210. 
Here we inspect the position ``xyz`` (x, y and z coordinates in cartesian system measured in meters) and 
orientation ``rpy`` (roll, pitch and yaw angles measured in radians) of each arm as mentioned in the `origin` tag 
inside each of the `joint` elements in the xml. 

```xml
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
  </joint>
  <joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
  </joint>
  <joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
  </joint>
  <joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
  </joint>
  <joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
  </joint>
```

We simplify the above details in the following table showing relative location of joint _`{i-1}`_ to joint _`{i}`_ 
as extracted from the URDF. Note that ``{i=7}`` represents the gripper (G), also called the end-effector (EE).

`{i}` | Joint Name | Parent Link | Child Link | x(m) | y(m) | z(m) | roll | pitch | yaw |
--- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
`1` | joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
`2` | joint_2 | link_1 | link_2 | 0 .35| 0 | 0.42 | 0 | 0 | 0 |
`3` | joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
`4` | joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
`5` | joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
`6` | joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
`7` | gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0 |

We use the above table to visualize the measurements in the robot joint space as follows: 

![alt_text][robot-joint-space-1]

Since joints 4,5 and 6 are revolute joints and their axes intersect at `joint_5`, 
we combine these 3 joints as if it is a single spherical wrist centered on `joint_5`. This further simplifies our calculations as follows.

![alt_text][robot-joint-space-2]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The DH table uses the following 4 parameters:
- **α<sub>i-1</sub>**  - twist angle - angle between the **Z<sub>i-1</sub>** and **Z<sub>i</sub>** axes.
- **a<sub>i-1</sub>** - link length - distance between the **Z<sub>i-1</sub>** and **Z<sub>i</sub>** axes.
- **d<sub>i</sub>** - link offset - distance between the **X<sub>i-1</sub>** and **X<sub>i</sub>** axes.
- **θ<sub>i</sub>** - joint angle - angle between the **X<sub>i-1</sub>** and **X<sub>i</sub>** axes.

The modified DH parameters obtained is tabulated below:

`{i}`| Links | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
--- | --- | --- | --- | --- | ---
`1` |  0->1 | 0 | 0 | 0.75 | θ<sub>1</sub>
`2` | 1->2 | -π/2 | 0.35 | 0 | θ<sub>2</sub> - π/2
`3` | 2->3 | 0 | 1.25 | 0 | θ<sub>3</sub>
`4` | 3->4 | -π/2 | -0.054 | 1.5 | θ<sub>4</sub>
`5` | 4->5 | π/2 | 0 | 0 | θ<sub>5</sub>
`6` | 5->6 | -π/2 | 0 | 0 | θ<sub>6</sub>
`7` | 6->G | 0 | 0 | 0.303 | 0

We use `python` to implement the forward kinematics with `sympy` package to code the transformations.

```python
# Create symbols
#
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles α
a0,     a1,     a2,     a3,     a4,     a5,     a6     = symbols('a0:7')     # link lengths a
d1,     d2,     d3,     d4,     d5,     d6,     d7     = symbols('d1:8')     # link offsets d
q1,     q2,     q3,     q4,     q5,     q6,     q7     = symbols('q1:8')     # joint angles θ



# Create Modified DH parameters
#
dh_dict = {
     alpha0:      0, a0:      0, d1:  0.75, q1:         q1,
     alpha1: -pi/2., a1:   0.35, d2:     0, q2: q2 - pi/2.,
     alpha2:      0, a2:   1.25, d3:     0, q3:         q3,
     alpha3: -pi/2., a3: -0.054, d4:  1.50, q4:         q4,
     alpha4:  pi/2., a4:      0, d5:     0, q5:         q5,
     alpha5: -pi/2., a5:      0, d6:     0, q6:         q6,
     alpha6:      0, a6:      0, d7: 0.303, q7:          0
}

```

Next, we create the following python function to obtain the individual frame transformation matrix,

```python
# Define Modified DH Transformation matrix
#
def transform(alpha, a, d, q):
    return Matrix([[             cos(q),             -sin(q),           0,               a],
                   [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                   [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                   [                  0,                   0,           0,               1]])
```

and compute each individual transformation matrix substituting the appropriate dh parameters in the above function as follows:

```python
# Create individual transformation matrices
#
T0_1 = transform(alpha0, a0, d1, q1).subs(dh_dict)
T1_2 = transform(alpha1, a1, d2, q2).subs(dh_dict)
T2_3 = transform(alpha2, a2, d3, q3).subs(dh_dict)
T3_4 = transform(alpha3, a3, d4, q4).subs(dh_dict)
T4_5 = transform(alpha4, a4, d5, q5).subs(dh_dict)
T5_6 = transform(alpha5, a5, d6, q6).subs(dh_dict)
T6_EE = transform(alpha6, a6, d7, q7).subs(dh_dict)
```
Using the individual transformation matrices obtained above, we compute the base-to-end-effector transformation matrix as a composite transformation as follows:

```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since the last three joints in our robot are revolute and their joint axes intersect at a single point, 
we have a case of spherical wrist with `joint_5` being the common intersection point and hence the wrist center. 
The IK problem can be decoupled into two sub problems viz. **Inverse Position Kinematics** and **Inverse Orientation Kinematics**. 
We address each of them as follows.  

#### Inverse Position Kinematics
Here we solve for the first 3 joint angles viz. θ<sub>1</sub>, θ<sub>2</sub> and θ<sub>3</sub> such that the wrist center reaches 
the target position correctly. For this, we need to derive the position of the wrist center from the end effector position. The end-effector position
is first rotated and translated to arrive at the to the wrist center position.

```python
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])


            # End-effector(gripper) position w.r.t. base (compensated with correction)
            R_EE = z_rotate(yaw) * y_rotate(pitch) * x_rotate(roll) * R_COR
            EE = Matrix([[px], [py], [pz]])
            # Wrist Center
            WC = EE - (0.303) * R_EE[:, 2]
```

Now calculating θ<sub>1</sub> is pretty straightforward, simply by projecting the Z of the wrist center onto the ground and 
taking the angle it makes with the X axis.


```python
theta_1 = atan2(WC[1], WC[0])
```
With the help of the following diagram and simple trigonometry we can also calculate θ<sub>2</sub> and θ<sub>3</sub>. Notice the
triangle with sides A,B and C where two sides are known, A = d<sub>4</sub> = 1.5 and C = a<sub>2</sub> = 1.25.
![alt text][inverse-calculation]

```python
            # the sides 
            SIDE_A = 1.501
            SIDE_B_xy = sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35
            SIDE_B_z = WC[2] - 0.75
            SIDE_B = sqrt(pow((SIDE_B_xy), 2) + pow((SIDE_B_z), 2))
            SIDE_C = 1.25
            
            # the angles
            ANG_A = acos(( SIDE_B * SIDE_B + SIDE_C * SIDE_C - SIDE_A * SIDE_A) / (2 * SIDE_B * SIDE_C))
            ANG_B = acos((-SIDE_B * SIDE_B + SIDE_C * SIDE_C + SIDE_A * SIDE_A) / (2 * SIDE_A * SIDE_C))
            
            theta_2 = pi / 2 - ANG_A - atan2(SIDE_B_z, SIDE_B_xy)
            theta_3 = pi / 2 - ANG_B + 0.036
```

#### Inverse Orientation Kinematics
Here we solve for the last 3 joint angles viz. θ<sub>4</sub>, θ<sub>5</sub> and θ<sub>6</sub>.  
Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

```R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6```

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of 
individual rotations between respective links, following holds true:

```R0_6 = R_EE```

where.
`R_EE` = Homogeneous RPY rotation between `base_link` and `gripper_link` as calculated above.


We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and 
pre-multiply both sides of the above equation by inv(R0_3) which leads to:

```R3_6 = inv(R0_3) * R_EE```

The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the 
joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for 
joint 4, 5, and 6. Following is the implementation in python.

```python
            # Extract rotation matrices from the transformation matrices
            #
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R3_6 = R0_3.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3}).T * R_EE

            theta_4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta_5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
            theta_6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            #
            ###
```

Note that we have used transpose operation instead of inverse operation in the python implementation. 
This is because R0_3 matrix is an orthogonal matrix, and hence transpose is equivalant to the inverse operation
and is faster in execution.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The final working code for the project can be found in the [IK_server.py](kuka_arm/scripts/IK_server.py) file. 
The output video can be found [here](misc_images/result.mp4).




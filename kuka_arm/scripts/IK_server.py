#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Useful utilities
#
def to_degrees(rad):
    return rad * 180 / pi


def to_radians(deg):
    return deg * pi / 180

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

# Define Modified DH Transformation matrix
#
def transform(alpha, a, d, q):
    return Matrix([[             cos(q),             -sin(q),           0,               a],
                   [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                   [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                   [                  0,                   0,           0,               1]])

# Create individual transformation matrices
#
T0_1 = transform(alpha0, a0, d1, q1).subs(dh_dict)
T1_2 = transform(alpha1, a1, d2, q2).subs(dh_dict)
T2_3 = transform(alpha2, a2, d3, q3).subs(dh_dict)
T3_4 = transform(alpha3, a3, d4, q4).subs(dh_dict)
T4_5 = transform(alpha4, a4, d5, q5).subs(dh_dict)
T5_6 = transform(alpha5, a5, d6, q6).subs(dh_dict)
T6_EE = transform(alpha6, a6, d7, q7).subs(dh_dict)
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE


#Define Rotation Matrices
#
def x_rotate(q):
    return Matrix([[1, 0, 0],
                   [0, cos(q), -sin(q)],
                   [0, sin(q), cos(q)]])


def y_rotate(q):
    return Matrix([[cos(q), 0, sin(q)],
                   [0, 1, 0],
                   [-sin(q), 0, cos(q)]])


def z_rotate(q):
    return Matrix([[cos(q), -sin(q), 0],
                   [sin(q), cos(q), 0],
                   [0, 0, 1]])

# Rotation discrepancy correction factor for orientation difference between URDF and DH table
R_COR = z_rotate(to_radians(180)) * y_rotate(to_radians(-90))


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        ### Your FK code here
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            # End-effector(gripper) position w.r.t. base (compensated with correction)
            R_EE = z_rotate(yaw) * y_rotate(pitch) * x_rotate(roll) * R_COR
            EE = Matrix([[px], [py], [pz]])
            # Wrist Center
            WC = EE - (0.303) * R_EE[:, 2]
            #
            # Calculate joint angles using Geometric IK method
            #
            SIDE_A = 1.501
            SIDE_B_xy = sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35
            SIDE_B_z = WC[2] - 0.75
            SIDE_B = sqrt(pow((SIDE_B_xy), 2) + pow((SIDE_B_z), 2))
            SIDE_C = 1.25

            ANG_A = acos((SIDE_B * SIDE_B + SIDE_C * SIDE_C - SIDE_A * SIDE_A) / (2 * SIDE_B * SIDE_C))
            ANG_B = acos((- SIDE_B * SIDE_B + SIDE_C * SIDE_C + SIDE_A * SIDE_A) / (2 * SIDE_A * SIDE_C))

            theta_1 = atan2(WC[1], WC[0])
            theta_2 = pi / 2 - ANG_A - atan2(SIDE_B_z, SIDE_B_xy)
            theta_3 = pi / 2 - ANG_B + 0.036

            # Extract rotation matrices from the transformation matrices
            #
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R3_6 = R0_3.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3}).T * R_EE

            theta_4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta_5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
            theta_6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()

#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def to_degrees(rad):
    return rad * 180 / pi


def to_radians(deg):
    return deg * pi / 180


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


def transform(α, a, d, q):
    return Matrix([[cos(q), -sin(q), 0, a],
                   [sin(q) * cos(α), cos(q) * cos(α), -sin(α), -sin(α) * d],
                   [sin(q) * sin(α), cos(q) * sin(α), cos(α), cos(α) * d],
                   [0, 0, 0, 1]])


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:

        ### Your FK code here
        # Create symbols
        #
        α0, α1, α2, α3, α4, α5, α6 = symbols('alpha0:7')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        #
        # Create Modified DH parameters
        #
        mod_dh = {
            α0: 0, a0: 0, d1: 0.75, q1: q1,
            α1: -pi / 2., a1: 0.35, d2: 0, q2: -pi / 2. + q2,
            α2: 0, a2: 1.25, d3: 0, q3: q3,
            α3: -pi / 2., a3: -0.054, d4: 1.5, q4: q4,
            α4: pi / 2, a4: 0, d5: 0, q5: q5,
            α5: -pi / 2., a5: 0, d6: 0, q6: q6,
            α6: 0, a6: 0, d7: 0.303, q7: 0
        }
        #
        # Define Modified DH Transformation matrix
        #
        #
        # Create individual transformation matrices
        #
        τ0_1 = transform(α0, a0, d1, q1).subs(mod_dh)
        τ1_2 = transform(α1, a1, d2, q2).subs(mod_dh)
        τ2_3 = transform(α2, a2, d3, q3).subs(mod_dh)
        τ3_4 = transform(α3, a3, d4, q4).subs(mod_dh)
        τ4_5 = transform(α4, a4, d5, q5).subs(mod_dh)
        τ5_6 = transform(α5, a5, d6, q6).subs(mod_dh)
        τ6_E = transform(α6, a6, d7, q7).subs(mod_dh)

        τ0_E = τ0_1 * τ1_2 * τ2_3 * τ3_4 * τ4_5 * τ5_6 * τ6_E
        r0_3 = τ0_1[0:3, 0:3] * τ1_2[0:3, 0:3] * τ2_3[0:3, 0:3]
        #
        # Extract rotation matrices from the transformation matrices
        #
        #
        ###

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

            (roll, pitch, yaw) = τ.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            corr_rotate = z_rotate(to_radians(180)) * y_rotate(to_radians(-90))
            r_ee = z_rotate(yaw) * y_rotate(pitch) * x_rotate(roll) * corr_rotate
            ee = Matrix([[px], [py], [pz]])

            c_wrist = ee - (0.303) * r_ee[:, 2]
            #
            # Calculate joint angles using Geometric IK method
            #
            t_1 = atan2(c_wrist[1], c_wrist[0])
            s_a = 1.50
            s_b_xy = sqrt(c_wrist[0] * c_wrist[0] + c_wrist[1] * c_wrist[1]) - 0.35
            s_b_z = c_wrist[2] - 0.75
            s_b = sqrt(pow((s_b_xy), 2) + pow((s_b_z), 2))
            s_c = 1.25
            ang_a = acos((s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c))
            ang_b = acos((- s_b * s_b + s_c * s_c + s_a * s_a) / (2 * s_a * s_c))
            t_2 = pi / 2 - ang_a - atan2(s_b_z, s_b_xy)
            t_3 = pi / 2 - ang_b + 0.036
            r0_3 = r0_3.evalf(subs={q1: t_1, q2: t_2, q3: t_3})
            r3_6 = r0_3.T * r_ee
            t_4 = atan2(r3_6[2, 2], -r3_6[0, 2])
            t_5 = atan2(sqrt(r3_6[0, 2] ** 2 + r3_6[2, 2] ** 2), r3_6[1, 2])
            t_6 = atan2(-r3_6[1, 1], r3_6[1, 0])
            #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [t_1, t_2, t_3, t_4, t_5, t_6]
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

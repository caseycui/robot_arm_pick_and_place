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


              
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        r, p, y = symbols('r p y')

        R_x = Matrix([[ 1,              0,        0],
                      [ 0,         cos(r),  -sin(r)],
                      [ 0,         sin(r),  cos(r)]])
        
        R_y = Matrix([[ cos(p),        0,  sin(p)],
                      [      0,        1,       0],
                      [-sin(p),        0, cos(p)]])
        
        R_z = Matrix([[ cos(y),  -sin(y),       0],
                      [ sin(y),   cos(y),       0],
                      [      0,        0,       1]])
        
        ### Your FK code here
        # Create symbols
	q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8') # theta_i
	d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
	a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
       	
	# Create Modified DH parameters
 	s = {alpha0: 0,		 a0: 0,		d1:0.75,
 	     alpha1: -pi/2,	 a1: 0.35, 	d2:0,		q2:q2-pi/2,
 	     alpha2: 0,		 a2: 1.25, 	d3:0,
 	     alpha3: -pi/2,	 a3: -0.054, 	d4:1.5,
 	     alpha4:  pi/2,	 a4: 0,		d5:0,		
 	     alpha5: -pi/2,	 a5: 0,		d6:0,		
 	     alpha6: 0,		 a6: 0, 	d7:0.303,    	q7:0}
	
	# Define Modified DH Transformation matrix

	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],  
        	       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               	       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               	       [                   0,                   0,            0,               1]])
	T0_1 = T0_1.subs(s)

	T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
	T1_2 = T1_2.subs(s)

	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
	T2_3 = T2_3.subs(s)

	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
	T3_4 = T3_4.subs(s)

	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
	T4_5 = T4_5.subs(s)

	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
	T5_6 = T5_6.subs(s)

	T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
	T6_G = T6_G.subs(s)

	# Create individual transformation matrices
        T0_3 = T0_1 * T1_2 * T2_3
 	#T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 *T4_5 * T5_6 * T6_G)	
 	T0_G = T0_3 * T3_4 *T4_5 * T5_6 * T6_G	
	
	# Extract rotation matrices from the transformation matrices
	    
        R0_G = T0_G[0:3,0:3]
        R0_3 = T0_3[0:3,0:3]

        ## Improve performance by moving out calculations outside loop ##
	# Compensate for rotation discrepancy between DH parameters and Gazebo

	#R_xcorr = rot_x(pi/2)
	#R_ycorr = rot_y(pi/2)
	#R_zcorr = rot_z(pi/2)

        # intrinsic rotation from DH frame to gazebo frame at 0 angles
        R_corr = R_z.subs(y,radians(180)) * R_y.subs(p,radians(-90))
	

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
      
            # rotation from gazebo frame to RPY frame as given by simulation 
            #Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
            Rrpy = R_z * R_y * R_x * R_corr
            Rrpy = Rrpy.subs({'r': roll, 'p':pitch, 'y': yaw})
		
	    # Calculate joint angles using Geometric IK method
	    
            # First, solve for the Wrist Center position (relative to the base frame)
	    # Ar(P/A0) = Ra_b * Br(P/B0) + Ar(B0/A0) # see lesson 12.10 Homogeneous Transforms
            #  Ar(P/A0)       Ra_b   |  Ar(B0/A0)     Br(P/B0)
            #[ --------] = [-----------------------][----------]   this is the matrix form of the above equation   
            #     1          0  0  0 |      1             1
            # The above matrix form is also expressed as Ar(P/A0) = Ta_b * Br(P/B0)
            # Ar(P/A0) = Ta_b * Tb_c * Cr(P/C0) # see lesson12.11 Composition of Homogeneous Transforms
	    # therefore, we know the WC pos in gripper frame, we know the T_total from base frame to gripper frame
        
            # 0. compute theta1  	    
            #T_0gripper = Rrpy.row_join(Matrix([[px],[py],[pz]]))
            #print(T_0gripper)
            #T_0gripper = T_0gripper.col_join(Matrix([[0,0,0,1]]))
            #T_0gripper.subs(s)
            #Pg_wc = Matrix([[0],[0],[-d7],[1]]).subs(s)           
            #P0_wc = T_0gripper *  Pg_wc
            #(x_wc,y_wc,z_wc,dont_care) = P0_wc
            
            EE = Matrix([[px],[py],[pz]])
            x_wc, y_wc, z_wc =  EE - (0.303) * Rrpy[:,2] 
            theta1 = atan2(y_wc,x_wc)
    
          
            # 1. compute theta2, theta3 based on the position of WC 
            # A = distance between joint 3 and wrist center. 
            #z_disp = a3.evalf(subs=s)
            #z_wc_prime = z_wc -z_disp
             
            # A,B,C 
            
            side_a = 1.501
            side_b = sqrt(pow((sqrt(x_wc * x_wc + y_wc * y_wc) - 0.35), 2) + pow((z_wc - 0.75),2))
            side_c = 1.25  
            #r = sqrt(x_wc * x_wc + y_wc * y_wc) -a1.evalf(subs=s)
            #l = z_wc - d1.evalf(subs=s)
            #B_2 = pow(r,2) + pow(l,2)
            
            #A_2 = d4.evalf(subs=s) * d4.evalf(subs=s) + a3.evalf(subs=s) * a3.evalf(subs=s)
            
            #C = a2.evalf(subs=s)
            #C_2 = pow(C,2)
            
            # Law of cosine	 		
            # B^2 = A^2 + C^2 - 2ACcosb		
            #b = acos((A_2 + C_2 - B_2)/(2*sqrt(A_2)*C))		
            #a = acos((B_2 + C_2 - A_2)/(2*sqrt(B_2)*C))
            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))
            
            # gamma angle
            #gamma = atan2(l,r)
            
            #theta2 = pi/2 - a - gamma
            theta2 = pi/2 - angle_a - atan2(z_wc - 0.75, sqrt(x_wc * x_wc + y_wc * y_wc) - 0.35) 
            # phi
            # Note that wrist center postion when joint3 is at 0 angle has a drop in z axis
            #phi = atan2(-a3.evalf(subs=s),d4.evalf(subs=s))
            #theta3 = pi/2 -b -phi 
            theta3 = pi/2 - angle_b - 0.036 # 0.036 accounts for sag in link4 of -0.054m
    
           
            # 2. compute q4, q5, q6 based on the orientation of WC 
                    
            # R0_G = Rrpy		
            E0_3 = R0_3.evalf(subs = {q1:theta1, q2:theta2, q3:theta3})
            E3_G = E0_3.inv(method = "LU") * Rrpy

            #print("E0_3 = ",E0_3)
            #print("Rrpy = ",Rrpy)
            #theta4, theta5, theta6 = solve_linear_system_LU(E3_6,[q4,q5,q6])
            #theta4, theta5, theta6 = 0,0,0

            theta4 = atan2(E3_G[2,2],-E3_G[0,2]) 
            theta5 = atan2(sqrt(E3_G[0,2] * E3_G[0,2] + E3_G[2,2] * E3_G[2,2]),E3_G[1,2]) 
            theta6 = atan2(-E3_G[1,1],E3_G[1,0]) 

            #if (abs(theta1) > pi):
            #    theta1 = (theta1 + pi/2)%(2*pi)- pi/2
            #if (abs(theta2) > pi):
            #    theta2 = (theta2 + pi/2)%(2*pi)- pi/2
            #if (abs(theta3) > pi):
            #    theta3 = (theta3 + pi/2)%(2*pi)- pi/2
            #if (abs(theta4) > pi):
            #    theta4 = (theta4 + pi/2)%(2*pi)- pi/2
            #if (abs(theta5) > pi):
            #    theta5 = (theta5 + pi/2)%(2*pi)- pi/2
            #if (abs(theta6) > pi):
            #    theta6 = (theta6 + pi/2)%(2*pi)- pi/2
               
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

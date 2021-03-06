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

[image1]: ./imgs/DH_frames.PNG
[image2]: ./imgs/kr210_urdf.PNG
[image3]: ./imgs/homotransform.jpg
[image4]: ./imgs/Ta_b.PNG
[image5]: ./imgs/T0_g.PNG
[image6]: ./imgs/threeD_view_theta.jpg
[image7]: ./imgs/Euler_angles.PNG
[image8]: ./imgs/pick_n_place.PNG



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The demo ran very smoothly and relatively fast in terms of robot arm speed. It seems to be picking up all the locations fine except for location 6. I will talk about this again in the result section. 

Now, we will define the DH frames. To simplify the calculations, we choose the DH frames as suggested in the course, with links4-6 share the same origin, aka, the wrist center.

![alt text][image1]

From the kr210.urdf.xacro file, we can find the link parameters according to our chosen DH frames

![alt text][image2]

And we will fill in the DH table according to the URDF file.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 |  pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7=0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

We can create the following transformation matrices about each joint. T0_1 means from joint 0 to joint 1

	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],  
        	       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               	       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               	       [                   0,                   0,            0,               1]])

	T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])

	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])

	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])

	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])

	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])

	T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])

Next, we can derive the generalized homogeneous transform between base_link and gripper_link. The base_link frame can be seen as derived from gripper_link frame at zero position following a series of intrinsic transforms as shown below:

![alt text][image3]

Therefore, we have derived the transform between base_link and gripper_link at its zero position:

Rcorr = R_x(pi/2) * R_y(pi/2) * R_z(pi/2)

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

To derive the theta angles, we can divide the problem into inverse position, and inverse orientation subproblems.

1. First, we can solve the inverse position subproblem by calculating the wrist center position.
We have previously calculated the homogeneous transform between the base_link and the gripper_link at its zero position. And since the gripper_link rotates in the simulation, we can obtain the rotation frame from the roll,pitch,yaw angles given by simulation.
Therefore, the total transform between the gripper_link frame and base_link frame is given as:

Rrpy = R_z(yaw) * R_y(pitch) * R_x(roll) * R_corr

According to lecture note, if we know a point (P)'s position in frame B (relative to B0), B0's position relative to frame A (A0), and the transfrom from B to A (Ra_b), we can derive that point's position in frame A (relative to A0), as shown in the following formula:

Ar(P/A0) = Ra_b * Br(P/B0) + Ar(B0/A0)   (see lesson 12.10 Homogeneous Transforms)

It can also be expressed as a matrix:

![alt text][image4]
  
The above matrix form is also expressed as Ar(P/A0) = Ta_b * Br(P/B0)

Now, frame A is our base_link frame, frame  B is our gripper frame, and P is the wrist center, A0 is the origin of the base_link frame, B0 is the position of the gripper origin.
We know from the simulation of the gripper position [px,py,pz], this is relative to the base_link frame, and Ra_b is Rrpy.
Therefore, we can construct the following 

![alt text][image5]
  
We also know the wrist center position relative to the gripper frame :  (0, 0, -d7)
Therefore, we can obtain the wrist center position relative to the base_link frame.
	    
        T_0gripper = Rrpy.row_join(Matrix([[px],[py],[pz]]))
        T_0gripper = T_0gripper.col_join(Matrix([[0,0,0,1]]))
        Pg_wc = Matrix([[0],[0],[-d7],[1]]).subs(s)           
        P0_wc = T_0gripper *  Pg_wc
        (x_wc,y_wc,z_wc,dont_care) = P0_wc
	
Since only -d7 is non-zero in the matrix, we can simplify the calculations to the following:	

        EE = Matrix([[px],[py],[pz]])
        x_wc, y_wc, z_wc =  EE - (0.303) * Rrpy[:,2] 
	
Now we have the wrist center position in base_link frame, we can calculate theta1 (q1), theta2 (q2), theta3 (q3).	
Below is a 3D view of the wrist center position determined by theta1, theta2 and theta3.

![alt text][image6]

theta1 is simply arctan of wrist center x, and y position cast onto the x-y plane.
theta2 = pi/2 - angle_a - angle_gamma (gamma), and gamma can be calculated with r and l. 
r = length of wc cast on x-y plane minus a1.
l = height of wc minus d1.
angle a, b, c can be calculated with consine law.
theta3 = pi/2 - b - angle_phi (angle of wrist center relative to link3 at zero position, this is caused by the drop of -0.054 from O3 to O4).

2. Second, we will solve the inverse orientation subproblem.

By far we know the total transform from base_link to gripper link,  and have calculated theta1-3.
Therefore, we can extract the rotation matrix of R0_G and R0_3, and derive R3_G which contains theta4-6.

	R0_G = Rrpy
	R0_3 = T0_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs = {q1:theta1, q2:theta2, q3:theta3})
        R3_G = R0_3.inv(method = "LU") * Rrpy
	
According to the euler angles trasformation matrix, we can calculate theta4-6.

![alt text][image7]

        theta4 = atan2(E3_G[2,2],-E3_G[0,2]) 
        theta5 = atan2(sqrt(E3_G[0,2] * E3_G[0,2] + E3_G[2,2] * E3_G[2,2]),E3_G[1,2]) 
        theta6 = atan2(-E3_G[1,1],E3_G[1,0]) 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

Techniques applied that improved performance:
First, I used simpified math for calculating theta1-3, instead of calculating internal variables such and r and l. The internal variables can be found in comment, as angle gamma and angle phi, etc. This has a slight saving in compute time and slight reduction in floating point rounding errors.
Second, I used Sympy matrices to define T0_1, T1_2... and R_x, R_y, R_z, instead of using functions. This has a slight saving in compute time. 
Third, I moved the calculations irrelavant to the for-loop outside. This has a big improvement on robot response time in Gazebo.
Fourth, I have applied tricks to keep the theta angles within joint limits according to the URDF file as much as possible. This improves path planning and execution in a few ways: it makes the robot arm movement steadier and faster; it saves extra path planning due to truncated angles; it saves loss from constrained movements. From the message stream, I see very few 'joint constraint limit' warnings compared to a lot before this technique is applied. Specifically, for theta2 and theta3, I keep the angles within joint limits by keeping the angles they need to substract within certain limits utilizing trignomotry. And for theta1, theta5, I keep the angles in (-pi,pi) range.
Fifth, I used extra decimal places for certain pre-calculated floating point values, such as angle phi (the WC drop angle from O3), this has improved precision in end effector pose.

Techniques tried and failed to improve performance:
I tried to use np.array to do matrix calculation, but couldn't get it to work for two reasons: 1, the precision seems to be awlful even when I specify the data type to be float32, the WC alone is off by 0.2 on average, and I get NaN values for theta4-6; 2, since I need to substitute the thetas in the Sympy matrix, I need to convert the sympy matrix to np array, or vice versa. I tried multiple ways but none seem to work and give me a rather large error.
I tried to set a tolerance value so that if newly received gripper position and euler angles are within tolerance from last state, then skip the IK calculation. I noticed that with this I get a larger list of poses (from ~20 tp ~100) from the path planner, therefore, I crossed this off. 

Results: 
Robot arm is able to move relatively steady, but slowy(compared to the demo), to perform the tasks.
I've individually set the target to spawn from 1 to 9, and all locations passed with the arm picking up and dropping off as expected.
However, when I set the target spawn parameter to 0, and give it a run, some locations fails. I couldn't figure out whether this is due to that the robot moved slightly from time to time, or some other covariance that is affecting the end-effector position from one run to another. This may also be the cause to the demo's failure to pick up position 6 after some moves.

Other Observations: 
Sometimes the robot arm couldn't adjust to the best position to grasp object, and there seems to be a timeout, so the robot arm goes on to execute the grasp command in whatever position the gripper is in thus hitting down the object.


Here's an image showing the robot arm in motion:
![alt text][image8]

Future improvement: 
1. improve the speed of execution
2. figure out the correlation problem between consecutive pickup and drop moves
3. figure out the 'deadlock' situation where the robot arm couldn't adjust to the best position and keeps going through the same set of angles over and over.


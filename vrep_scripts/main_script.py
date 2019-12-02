import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm


# Get distances measurements from each joint center to base frame (useful for forward kinematics)
def get_joint():
	X = []
	Y = []
	Z = []
	result,vector=vrep.simxGetObjectPosition(clientID, joint_one_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_two_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_three_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_four_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_five_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_six_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	X = np.round(X, decimals = 3)
	Y = np.round(Y, decimals = 3)
	Z = np.round(Z, decimals = 3)
	return X,Y,Z

# Function that used to move joints
def SetJointPosition(theta):
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	#time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	#time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	#time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	#time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	#time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	#time.sleep(0.5)

# Function that used to move joints
def SetJointVelocity():
    vel = 0.5
    o = vrep.simxSetJointTargetVelocity(clientID, joint_one_handle, vel, vrep.simx_opmode_blocking)
	#time.sleep(0.5)
    print(o)
    o = vrep.simxSetJointTargetVelocity(clientID, joint_two_handle, vel, vrep.simx_opmode_blocking)
	#time.sleep(0.5)
    print(o)
    o=vrep.simxSetJointTargetVelocity(clientID, joint_three_handle, vel, vrep.simx_opmode_blocking)
	#time.sleep(0.5)
    print(o)
    o=vrep.simxSetJointTargetVelocity(clientID, joint_four_handle, vel, vrep.simx_opmode_blocking)
	#time.sleep(0.5)
    print(o)
    o=vrep.simxSetJointTargetVelocity(clientID, joint_five_handle, vel, vrep.simx_opmode_blocking)
	#time.sleep(0.5)
    print(o)
    o=vrep.simxSetJointTargetVelocity(clientID, joint_six_handle, vel, vrep.simx_opmode_blocking)
	#time.sleep(0.5)
    print(o)

# Function that used to read joint angles
def GetJointAngle():
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 1 joint variable')
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 2 joint variable')
	result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 3 joint variable')
	result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 4 joint variable')
	result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 5 joint variable')
	result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 6 joint variable')
	theta = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	return theta


# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

# Print object name list
result,joint_name,intData,floatData,stringData = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking)
#print(stringData)


# Get "handle" to the base of robot
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR5_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')
    
    

# Get "handle" to the all joints of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')
result, vacuum_cup_handle = vrep.simxGetObjectHandle(clientID, 'BaxterVacuumCup', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for vacuum cup')
# Get "handle" to the end-effector of robot
result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR5_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')
#==================================================================================================== #

result, basket_handle = vrep.simxGetObjectHandle(clientID, 'Cup', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for Cup')

result, sphere_handle = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sphere')

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #
time.sleep(1)

#Calculate Rotation of base joint
a = vrep.simxGetObjectPosition(clientID, basket_handle, joint_one_handle, vrep.simx_opmode_blocking )
vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active',1, vrep.simx_opmode_oneshot)
base_rot = np.arctan2(a[1][1], a[1][0]) + 0.5*np.pi
a2 = vrep.simxGetObjectPosition(clientID, basket_handle, joint_one_handle, vrep.simx_opmode_blocking )

v = 1.487
l1 = 0.425
l2 = 0.47
t = 0.49
d0 = (a2[1][1]**2+a2[1][0]**2)**0.5

table = []
for i in range(-50,51):
    theta = i*0.01*np.pi
    y = t + l1*np.cos(theta) + l2
    ti = np.sqrt((2.0*y)/9.81)
    dist = v*ti - l1*sin(theta)
    table.append((theta, dist))

#print(table)
#d = (a2[1][1]**2 + a2[1][0]**2)**0.5
#theta1 = 2*np.arctan2((-1.0*l1 - np.sqrt(-1*d**2 + l1**2)),d)
#print(theta1)
#SetJointVelocity()

#shot from the top
Goal_joint_angles = np.array([[base_rot,0,0,0,np.pi,0],[base_rot,0,0.5*np.pi,0,np.pi,0],[base_rot,0,-0.5*np.pi,0,np.pi,0]])
midPoint = 0

chosen = None
for t in table:
    if t[1] - d0 < 0.03:
        chosen = t[0]
        break
    
#Goal_joint_angles = np.array([[base_rot,0,0,0,np.pi,0],[base_rot,-0.5*np.pi,np.pi,0,np.pi,0],[base_rot,-0.5*np.pi,0,0,np.pi,0]])
#midPoint = 0.5*np.pi

#Goal_joint_angles = np.array([[base_rot,0,0,0,np.pi,0],[base_rot,0.5*np.pi,0,0,np.pi,0],[base_rot,0.5*np.pi,-1.0*np.pi,0,np.pi,0]])
#midPoint = -0.5*np.pi
thetaS = 0.5*np.pi-chosen
thetaE = thetaS - np.pi

Goal_joint_angles = np.array([[base_rot,0,0,0,np.pi,0],[base_rot,chosen,thetaS,0,np.pi,0],[base_rot,chosen,thetaE,0,np.pi,0]])
midPoint = thetaS - 0.5*np.pi


for i in range(len(Goal_joint_angles)):
        time.sleep(1.5)
        SetJointPosition(Goal_joint_angles[i])
''' 
print("#########")

r, l1 = vrep.simxGetObjectPosition(clientID, joint_three_handle, joint_two_handle, vrep.simx_opmode_blocking  ) 
if r==0:
    print(l1)
r, l2 = vrep.simxGetObjectPosition(clientID, sphere_handle, joint_three_handle, vrep.simx_opmode_blocking  ) 
if r==0:
    print(l2) 
print("#########")
'''
ret, lin, ang = vrep.simxGetObjectVelocity(clientID, sphere_handle, vrep.simx_opmode_streaming )
posret, pos = vrep.simxGetObjectPosition(clientID, sphere_handle, -1, vrep.simx_opmode_streaming ) 
ret, j2Pos = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_streaming)
hold = True

v=1.487

maxpos = 9999999
while(hold):
    posret, pos = vrep.simxGetObjectPosition(clientID, sphere_handle, -1, vrep.simx_opmode_buffer ) 
    ret2, j2Pos = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_buffer)

    if abs(j2Pos) < maxpos and ret2 == 0:
        maxpos = abs(j2Pos)
        

    if abs(j2Pos - midPoint) < 0.07 and ret2 == 0:
        vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active',0, vrep.simx_opmode_oneshot)
        print("Throw!")
        ret, lin, ang = vrep.simxGetObjectVelocity(clientID, sphere_handle, vrep.simx_opmode_buffer )
        print(lin)
        hold = False
        print(maxpos)
    
    
    # Wait two seconds
time.sleep(2)
# **************************************************************************************************** #

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
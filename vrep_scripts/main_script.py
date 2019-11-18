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

print("#########")
# Print object name list
result,joint_name,intData,floatData,stringData = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking)
#print(stringData)


# Get "handle" to the base of robot
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')
    
# Get "handle" to the all joints of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')
result, vacuum_cup_handle = vrep.simxGetObjectHandle(clientID, 'BaxterVacuumCup', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for vacuum cup')
# Get "handle" to the end-effector of robot
result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')
# ==================================================================================================== #

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #
time.sleep(1)
vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active',1, vrep.simx_opmode_oneshot)

    
Goal_joint_angles = np.array([[-0.5*np.pi,0,0,0,0,0],[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,0]])
#Alternate goal joint angles:
#np.array([[-0.5*np.pi,0,0,0,0,0],[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,0]]) #Acceleration Limit = 20
#np.array([[-0.5*np.pi,0,0,0,0,-0.5*np.pi],[-0.5*np.pi,-0.5*np.pi,0,0,0,-0.5*np.pi]])

#vrep.simxSetJointTargetVelocity(clientID, joint_one_handle, np.pi/2, vrep.simx_opmode_streaming)
#vrep.simxSetJointTargetVelocity(clientID, joint_two_handle, np.pi/2, vrep.simx_opmode_streaming)
#vrep.simxSetJointTargetVelocity(clientID, joint_three_handle, np.pi/2, vrep.simx_opmode_streaming)
#vrep.simxSetJointTargetVelocity(clientID, joint_four_handle, np.pi/2, vrep.simx_opmode_streaming)
#vrep.simxSetJointTargetVelocity(clientID, joint_five_handle, np.pi/2, vrep.simx_opmode_streaming)
#vrep.simxSetJointTargetVelocity(clientID, joint_six_handle, np.pi/2, vrep.simx_opmode_streaming)

for i in range(len(Goal_joint_angles)):
        time.sleep(0.5)

        SetJointPosition(Goal_joint_angles[i])
    
time.sleep(0.15)
        
xAccel=vrep.simxGetFloatSignal(clientID,'accelerometerX',vrep.simx_opmode_streaming)
yAccel=vrep.simxGetFloatSignal(clientID,'accelerometerY',vrep.simx_opmode_streaming)
zAccel=vrep.simxGetFloatSignal(clientID,'accelerometerZ',vrep.simx_opmode_streaming)

ballVelX=vrep.simxGetFloatSignal(clientID,'gyroX',vrep.simx_opmode_streaming)
ballVelY=vrep.simxGetFloatSignal(clientID,'gyroY',vrep.simx_opmode_streaming)
ballVelZ=vrep.simxGetFloatSignal(clientID,'gyroZ',vrep.simx_opmode_streaming)

hold = True
while(hold):

    xAccel=vrep.simxGetFloatSignal(clientID,'accelerometerX',vrep.simx_opmode_buffer)
    yAccel=vrep.simxGetFloatSignal(clientID,'accelerometerY',vrep.simx_opmode_buffer)
    zAccel=vrep.simxGetFloatSignal(clientID,'accelerometerZ',vrep.simx_opmode_buffer)
    ballVelX=vrep.simxGetFloatSignal(clientID,'gyroX',vrep.simx_opmode_streaming)
    ballVelY=vrep.simxGetFloatSignal(clientID,'gyroY',vrep.simx_opmode_streaming)
    ballVelZ=vrep.simxGetFloatSignal(clientID,'gyroZ',vrep.simx_opmode_streaming)
    if(xAccel[0] == 1 or yAccel[0] == 1 or zAccel[0] == 1):
        print("Error")
        continue
    print("Acceleration: ",xAccel, " ", yAccel, " ", zAccel)
    print("Ball Velocity: ",ballVelX, " ", ballVelY, " ", ballVelZ)
    if ((xAccel[1]**2+yAccel[1]**2+zAccel[1]**2)**0.5 < 21):
        vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active',0, vrep.simx_opmode_oneshot)
        print("Throw!")
        ballVel = (ballVelX[1]**2+ballVelY[1]**2+ballVelZ[1]**2)**0.5
        print("Ball velocity at Throw: ",ballVel)
        hold = False
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
# ECE_470_
Robotic 6 jointed arm basketball shooter repo
#Project Update 4
For this update, we started using the python client in VREP and implemented a minimal working example that allows us to work on improving the robot. Currently, we use a predefined goal configuration (except for the first joint) to throw the ball. This allows us to begin working on the various modules of our system.
1. Releasing the ball: Currently, we are using accelerometers and gyroscopes to determine when the ball should be released. In order for the ball to have an initial velocity, the ball has to be released while the robot is still moving. However, to limit the number of different ways in which this could happen, we have decided to limit the initial velocity of the ball relatively parallel to the gound.
2. Determining required goal configuration: Although we are currently using a predetermined goal configuration, we are already calculating the angle for the first joint such that the robot faces the basket. 
3. Using kinematics to determine necessary ball speed and height: Since we know the location of the basket, we should be able to determine the necessary height and velocity such that the ball lands in the basket. However, since there are countless different ways in which this could happen, we are working on simplified equations that work under the assumptions that there is a range of possible heights and velocities and that the initial velocity is parallel to the ground.


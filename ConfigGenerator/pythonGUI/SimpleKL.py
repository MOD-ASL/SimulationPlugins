## @package SimpleKL Provides some simple kinematic functions
import numpy as np
from Module import *
## Rotation matrix around x axis
# @param row Row 
# @return Numpy matrix object of rotation matrix
def rotx(row):
	return np.matrix([[1,0,0],[0,np.cos(row),-np.sin(row)],[0,np.sin(row),np.cos(row)]])
## Rotation matrix around y axis
# @param pitch Pitch 
# @return Numpy matrix object of rotation matrix
def roty(pitch):
	return np.matrix([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
## Rotation matrix around z axis
# @param yaw Yaw 
# @return Numpy matrix object of rotation matrix
def rotz(yaw):
	return np.matrix([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
## Checks whether two faces close enough
# @param pos1 Face norm vector of the first face
# @param pos2 Face norm vector of the second face
# @return If two faces are close, return True; otherwise, return False
def CloseEnough(pos1,pos2):
	referenceVec = np.matrix([[pos1[0]-pos2[0]],[pos1[1]-pos2[1]],[pos1[2]-pos2[2]]])
	dis = np.linalg.norm(referenceVec)
	if dis>=0.098 and dis<=0.102:
		return True
	else:
		return False
## Giving position of two modules to check which faces can be connected
# @param pos1 Position of the first module 1
# @param joint1 List of joint anlges of module 1
# @param pos1 Position of the first module 2
# @param joint1 List of joint anlges of a module 2
# @return If connectable, then return connectable node index number; otherwise False
def Connectable(module1,joint1,module2,joint2):
	threshold = 0.98
	referenceVec = np.matrix([[module1.Position[0]-module2.Position[0]],[module1.Position[1]-module2.Position[1]],[module1.Position[2]-module2.Position[2]]])
	referenceVec = referenceVec/np.linalg.norm(referenceVec)
	lftVec1 = np.matrix([[1],[0],[0]])
	rgtVec1 = np.matrix([[-1],[0],[0]])
	bckvec1 = np.matrix([[0],[1],[0]])
	frtVec1 = np.matrix([[0],[-1],[0]])
	bckvec1 = rotx(joint1[3])*bckvec1
	lftVec1 = module1.rotation_matrix*lftVec1
	rgtVec1 = module1.rotation_matrix*rgtVec1
	bckvec1 = module1.rotation_matrix*bckvec1
	frtVec1 = module1.rotation_matrix*frtVec1
	connect1 = 4
	if np.dot(referenceVec.transpose().tolist()[0],lftVec1.transpose().tolist()[0])<= -threshold:
		connect1 = 1
	if np.dot(referenceVec.transpose().tolist()[0],rgtVec1.transpose().tolist()[0])<= -threshold:
		connect1 = 2
	if np.dot(referenceVec.transpose().tolist()[0],bckvec1.transpose().tolist()[0])<= -threshold:
		connect1 = 3
	if np.dot(referenceVec.transpose().tolist()[0],frtVec1.transpose().tolist()[0])<= -threshold:
		connect1 = 0

	lftVec2 = np.matrix([[1],[0],[0]])
	rgtVec2 = np.matrix([[-1],[0],[0]])
	bckvec2 = np.matrix([[0],[1],[0]])
	frtVec2 = np.matrix([[0],[-1],[0]])
	bckvec2 = rotx(joint2[3])*bckvec2
	lftVec2 = module2.rotation_matrix*lftVec2
	rgtVec2 = module2.rotation_matrix*rgtVec2
	bckvec2 = module2.rotation_matrix*bckvec2
	frtVec2 = module2.rotation_matrix*frtVec2
	connect2 = 4
	if np.dot(referenceVec.transpose().tolist()[0],lftVec2.transpose().tolist()[0]) >= threshold:
		connect2 = 1
	if np.dot(referenceVec.transpose().tolist()[0],rgtVec2.transpose().tolist()[0]) >= threshold:
		connect2 = 2
	if np.dot(referenceVec.transpose().tolist()[0],bckvec2.transpose().tolist()[0]) >= threshold:
		connect2 = 3
	if np.dot(referenceVec.transpose().tolist()[0],frtVec2.transpose().tolist()[0]) >= threshold:
		connect2 = 0

	if connect1 != 4 and connect2 != 4:
		return (connect1,connect2)
	else:
		return False
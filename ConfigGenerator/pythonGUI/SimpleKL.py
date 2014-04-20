import numpy as np

def rotx(row):
	return np.matrix([[1,0,0],[0,np.cos(row),-np.sin(row)],[0,np.sin(row),np.cos(row)]])

def roty(pitch):
	return np.matrix([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])

def rotz(yaw):
	return np.matrix([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])

def CloseEnough(pos1,pos2):
	referenceVec = np.matrix([[pos1[0]-pos2[0]],[pos1[1]-pos2[1]],[pos1[2]-pos2[2]]])
	dis = np.linalg.norm(referenceVec)
	if dis>=0.098 and dis<=0.102:
		return True
	else:
		return False

def Connectable(pos1,joint1,pos2,joint2):
	threshold = 0.98
	referenceVec = np.matrix([[pos1[0]-pos2[0]],[pos1[1]-pos2[1]],[pos1[2]-pos2[2]]])
	referenceVec = referenceVec/np.linalg.norm(referenceVec)
	lftVec1 = np.matrix([[1],[0],[0]])
	rgtVec1 = np.matrix([[-1],[0],[0]])
	bckvec1 = np.matrix([[0],[1],[0]])
	frtVec1 = np.matrix([[0],[-1],[0]])
	bckvec1 = rotx(joint1[3])*bckvec1
	lftVec1 = rotz(pos1[5])*roty(pos1[4])*rotx(pos1[3])*lftVec1
	rgtVec1 = rotz(pos1[5])*roty(pos1[4])*rotx(pos1[3])*rgtVec1
	bckvec1 = rotz(pos1[5])*roty(pos1[4])*rotx(pos1[3])*bckvec1
	frtVec1 = rotz(pos1[5])*roty(pos1[4])*rotx(pos1[3])*frtVec1
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
	lftVec2 = rotz(pos2[5])*roty(pos2[4])*rotx(pos2[3])*lftVec2
	rgtVec2 = rotz(pos2[5])*roty(pos2[4])*rotx(pos2[3])*rgtVec2
	bckvec2 = rotz(pos2[5])*roty(pos2[4])*rotx(pos2[3])*bckvec2
	frtVec2 = rotz(pos2[5])*roty(pos2[4])*rotx(pos2[3])*frtVec2
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
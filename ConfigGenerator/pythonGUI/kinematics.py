import pdb
from numpy import matrix, deg2rad, rad2deg, cos, sin, hstack, vstack, eye, pi, arcsin, arccos, arctan2, sqrt
from math import copysign
c = cos
s = sin


def rotz(theta):
	''' Returns a rotation matrix about z-axis by angle theta, in radians. '''
	return matrix([[c(theta), -s(theta), 0],
			 	[s(theta),  c(theta), 0],
			 	[		 0, 	   0, 1],
			   ])

def rotx(theta):
	''' Returns a rotation matrix about x-axis by angle theta, in radians. '''
	return matrix([[1, 		  0, 		 0],
				   [0, c(theta), -s(theta)],
				   [0, s(theta),  c(theta)],
				  ])

def roty(theta):
	''' Returns a rotation matrix about y-axis by angle theta, in radians. '''
	return matrix([[c(theta),	0,	-s(theta)],
				   [	   0,	1,			0],
				   [s(theta),	0,	 c(theta)]
				  ])

def se3(r, t):
	''' Returns a 4x4 se3 matrix using r as rotation and t as translation. '''
	return vstack( [hstack([r, t]), matrix([0, 0, 0, 1])] )

def rotZYX2rpy(R):
	''' Returns a touple (r,p,y) which are the tait-bryan angles for R, which
	is a ZYX rotation matrix. '''
	#pitch = arcsin(-R[2,0])			# about y
	#yaw = arccos(R[0,0]/cos(pitch))	# about z
	#roll = arccos(R[2,2]/cos(pitch))	# about x
	pitch = arctan2( -R[2,0] , sqrt(R[2,1]**2 + R[2,2]**2) )
	yaw = arctan2( R[1,0], R[0,0] )
	roll = arctan2( R[2,1], R[2,2] )
	return (roll, pitch, yaw)

def rotToQuat(R):
	''' Adapted from function written by Daniel Mellinger
	from the following website, deals with the case when tr<0
	http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixT
	oQuaternion/index.htm '''

	#takes in W_R_B rotation matrix

	tr = R[0,0] + R[1,1] + R[2,2]

	if (tr > 0):
		S = sqrt(tr+1.0) * 2; # S=4*qw 
		qw = 0.25 * S
		qx = (R[2,1] - R[1,2]) / S
		qy = (R[0,2] - R[2,0]) / S 
		qz = (R[1,0] - R[0,1]) / S 
	elif ((R[0,0] > R[1,1]) and (R[0,0] > R[2,2])):
		S = sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2 # S=4*qx 
		qw = (R[2,1] - R[1,2]) / S
		qx = 0.25 * S
		qy = (R[0,1] + R[1,0]) / S 
		qz = (R[0,2] + R[2,0]) / S 
	elif (R[1,1] > R[2,2]):
		S = sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2 # S=4*qy
		qw = (R[0,2] - R[2,0]) / S
		qx = (R[0,1] + R[1,0]) / S 
		qy = 0.25 * S
		qz = (R[1,2] + R[2,1]) / S 
	else: 
		S = sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2 # S=4*qz
		qw = (R[1,0] - R[0,1]) / S
		qx = (R[0,2] + R[2,0]) / S
		qy = (R[1,2] + R[2,1]) / S
		qz = 0.25 * S

	sgn = copysign(1,qw)
	q = (qw*sgn, qx*sgn, qy*sgn, qz*sgn)
	#q = q*sign(qw);
	return q

#########
L = 0.05

def get_new_position(parent_module, new_module_angles, parent_face, new_face):
        """ Returns the position and orientation of the new module as (x,y,z,r,p,y) """
        new_wrt_old = get_xform(parent_module.JointAngle, new_module_angles,
        								   parent_face, new_face)
        p_pos = matrix(parent_module.Position[0:3]).T
        #p_roll = parent_module.Position[3]
        #p_pitch = parent_module.Position[4]
        #p_yaw = parent_module.Position[5]
        #p_rot = rotz(p_yaw)*roty(p_pitch)*rotx(p_roll)
        p_rot = parent_module.rotation_matrix
        new_wrt_world = se3(p_rot, p_pos)*new_wrt_old
        n_pos = tuple(new_wrt_world[0:3,3].ravel().tolist()[0])
        n_rpy = rotZYX2rpy( new_wrt_world[0:3,0:3] )
        print n_rpy
        if cos(n_rpy[1]) < 0.01:
        	print 'WARNING: Gymbal lock detected!'
        n_quat = rotToQuat( new_wrt_world[0:3,0:3] )
        return ( n_pos+n_rpy, new_wrt_world[0:3,0:3], n_quat )

def get_xform(angles1, angles2, face1, face2):
	''' Returns the se3 transform from the center of module1 to the center of module2, given their
	joint angles and the faces at which they are connected. '''
	T1 = get_xform1(angles1, face1)
	T2 = get_xform2(angles2, face2)
	return T1*T2

def get_xform1(a, face1):
	if face1 == 0:
		t0 = matrix([0, 0, 0]).T
		r0 = rotz( -pi/2 )
		t1 = matrix([L, 0, 0]).T
		r1 = rotx( a[0] )
		return se3(r0,t0)*se3(r1,t1)
	elif face1 == 1:
		t0 = matrix([0, 0, 0]).T
		r0 = rotx( a[3] )
		t1 = matrix([L, 0, 0]).T
		r1 = rotx( a[1] )
		return se3(r0,t0)*se3(r1,t1)
	elif face1 ==2:
		t0 = matrix([0, 0, 0]).T
		r0 = rotx( a[3] )
		t1 = matrix([0, 0, 0]).T
		r1 = rotz( pi )
		t2 = matrix([L, 0, 0]).T
		r2 = rotx( a[2] )
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)
	elif face1 == 3:
		t0 = matrix([0, 0, 0]).T
		r0 = rotx( a[3] )
		t1 = matrix([0, 0, 0]).T
		r1 = rotz( pi/2 )
		t2 = matrix([L, 0, 0]).T
		r2 = matrix(eye(3))
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)
	else:
		assert False, 'Unrecognized face number: ' + str(face1)

def get_xform2(a, face2):
	if face2 == 0:
		t0 = matrix([0, 0, 0]).T
		r0 = rotz( pi )
		t1 = matrix([0, 0, 0]).T
		r1 = rotx( a[0] )
		t2 = matrix([-L, 0, 0]).T
		r2 = rotz( pi/2 )
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)
	elif face2 == 1:
		t0 = matrix([0, 0, 0]).T
		r0 = rotz( pi )
		t1 = matrix([0, 0, 0]).T
		r1 = rotx( a[2] )
		t2 = matrix([-L, 0, 0]).T
		r2 = rotx( -a[3] ) 
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)
	elif face2 == 2:
		t0 = matrix([0, 0, 0]).T
		r0 = rotz( pi )
		t1 = matrix([0, 0, 0]).T
		r1 = rotx( a[2] ) # joint
		t2 = matrix([0, 0, 0]).T
		r2 = rotz( pi )
		t3 = matrix([L, 0, 0]).T
		r3 = rotx( -a[3] ) 
		return se3(r0,t0)*se3(r1,t1)*se3(r2,t2)*se3(r3,t3)
	elif face2 == 3:
		t0 = matrix([L, 0, 0]).T
		r0 = rotz( pi/2 )
		t1 = matrix([0, 0, 0]).T
		r1 = rotx( -a[3] )	# Note: this must be negated because positive rotations move the face-3 block relative the face-0 block.
		return se3(r0,t0)*se3(r1,t1)
	else:
		assert False, 'Unrecognized face number: ' + str(face2)

if __name__ == "__main__":
	angles1 = [0, 0, 0, pi/4]
	angles2 = [0, 0, 0, 0]
	face1 = 1
	face2 = 2
	print get_xform(angles1, angles2, face1, face2)
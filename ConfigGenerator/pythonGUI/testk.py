from numpy import matrix, deg2rad, rad2deg, cos, sin, hstack, vstack, eye, pi
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

def se3(r, t):
	''' Returns a 4x4 se3 matrix using r as rotation and t as translation. '''
	return vstack( [hstack([r, t]), matrix([0, 0, 0, 1])] )

if __name__=="__main__":
	l1 = 2
	l2 = 1
	theta1 = deg2rad(30)
	theta2 = deg2rad(10)
	#
	r1 = rotz(theta1)
	t1 = matrix([0, 0, 0]).T
	T1 = se3(r1, t1)
	#
	r2 = rotz(theta2)
	t2 = matrix([l1, 0, 0]).T
	T2 = se3(r2,t2)
	#
	r3 = matrix(eye(3))
	t3 = matrix([l2, 0, 0]).T
	T3 = se3(r3, t3)
	print T1*T2*T3
import numpy as np
from normalized import normalized_vec

__all__  = ["R_from_xyzw_quaternion","mul_xyzw_quaternion","xyzw_quaternion_from_rotvec_row"]

def mul_xyzw_quaternion(a,b):
	return np.array((
		a[0]*b[3]+a[3]*b[0]+a[1]*b[2]-a[2]*b[1],
		a[1]*b[3]+a[3]*b[1]+a[2]*b[0]-a[0]*b[2],
		a[2]*b[3]+a[3]*b[2]+a[0]*b[1]-a[1]*b[0],
		-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]+a[3]*b[3]))

# This mapping satisfies: R(q1*q2)=R(q1)*R(q2). This is probably the only mapping
# to matrices that satisfies this. 
def xyzw_quaternion_to_R(quat, matrix):
    quat = normalized_vec(quat)
    # Repetitive calculations.
    q4_2 = quat[3]**2
    q12 = quat[0] * quat[1]
    q13 = quat[0] * quat[2]
    q14 = quat[0] * quat[3]
    q23 = quat[1] * quat[2]
    q24 = quat[1] * quat[3]
    q34 = quat[2] * quat[3]

    # The diagonal.
    matrix[0, 0] = 2.0 * (quat[0]**2 + q4_2) - 1.0
    matrix[1, 1] = 2.0 * (quat[1]**2 + q4_2) - 1.0
    matrix[2, 2] = 2.0 * (quat[2]**2 + q4_2) - 1.0

    # Off-diagonal.
    matrix[0, 1] = 2.0 * (q12 - q34)
    matrix[0, 2] = 2.0 * (q13 + q24)
    matrix[1, 2] = 2.0 * (q23 - q14)

    matrix[1, 0] = 2.0 * (q12 + q34)
    matrix[2, 0] = 2.0 * (q13 - q24)
    matrix[2, 1] = 2.0 * (q23 + q14)

def R_from_xyzw_quaternion(q):
	R = np.zeros((3,3))
	xyzw_quaternion_to_R(q,R)
	return R

# returns a quaternion that corresponds to a rotation matrix
# acting on row vectors
def xyzw_quaternion_from_rotvec_row(r):
	r_norm_half = np.linalg.norm(r,axis=-1)*0.5
	return np.concatenate([-np.multiply(0.5*np.sinc(r_norm_half/np.pi),r),[np.cos(r_norm_half)]])

#convenience
def R_from_rotvec_row(rotvec):
	return R_from_xyzw_quaternion(xyzw_quaternion_from_rotvec_row(rotvec))

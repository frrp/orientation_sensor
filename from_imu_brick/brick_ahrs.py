import ctypes as ct

import os.path

dll_name = "libbrick_ahrs.so"

dllabspath = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name

m =ct.CDLL(dllabspath)

#__all__ = ["c_real_types","c_quaternion_types","update_filter"]

state_type = ct.c_float*9

def imu_update_filter_args():
	r=ct.c_float
	return [
		r,r,r,
		r,r,r,
		r,r,r,
		r,
		r,
		r,
		state_type
	]

m.imu_update_filter.restype = None
m.imu_update_filter.argtypes = imu_update_filter_args()


del imu_update_filter_args

def initial_state():
	state = state_type()
	state[0]=1.0
	state[4]=1.0
	return state

def update_filter((
			a_B_omega,
			beta,
			zeta,
			delta_t),
		q):
	((a0,a1,a2),(B0,B1,B2),(omega0,omega1,omega2)) = a_B_omega
	args = [omega0, omega1, omega2, a0, a1, a2, B0, B1, B2, beta, zeta, delta_t, q]
	m.imu_update_filter(*args)


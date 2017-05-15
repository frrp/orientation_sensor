from time import sleep
import numpy as np

def acquire_all(imu, cb_period, time):
	imu.set_all_data_period(cb_period)
	values = []
	imu.register_callback(imu.CALLBACK_ALL_DATA, lambda ax,ay,az,Bx,By,Bz,ox,oy,oz,T: values.append((ax,ay,az,Bx,By,Bz,ox,oy,oz,T)))
	sleep(time)
	imu.set_all_data_period(0)
	sleep(0.25)
	return values

def aggregate_and_group(m):
	m_agg = np.array([np.mean(m,axis=0),np.std(m,axis=0)])
	a = m_agg[...,0:3]
	B = m_agg[...,3:6]
	omega = m_agg[...,6:9]
	T = m_agg[...,9]
	return (a,B,omega,T)

def get_a(m):
	return m[0]

def get_B(m):
	return m[1]

def get_omega(m):
	return m[2]

def get_T(m):
	return m[3]

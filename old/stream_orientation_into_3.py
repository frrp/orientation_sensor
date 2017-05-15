import from_imu_brick.brick_ahrs as ba
import utils.transformations as tfms
import brick_calibration as bc
import corrections as cs
import numpy as np


def stop_realtime_filtering(imu):
	#consider clearing the registered callback
	imu.set_all_data_period(0)

# todo: make it possible to use several filters simultaneously


def f_get_corrected_imu_vecs(C_imu):
	gcab = bc.f_get_corrected_a_B(C_imu)
	def f((a,B,omega,T)):
		a_corr, B_corr = gcab((a,b))
		return a_corr,B_corr,omega

def f_filter_step(beta,zeta):
	def f((a,B,omega),delta_t,filter_state):
		ba.update_filter((a,B,omega,beta,zeta,delta_t),filter_state)		
	return f

(np.array(omega)*np.pi/(180*14.375)

#14.375 is a strange constant documented in imu brick documentation
def correct_filter_step(filter_step,correct_imu_vecs):
	def f(args,delta_t,filter_state):
		a=args[0:3]
		B=args[3:6]
		omega=np.array(args[6:9])*np.pi/(180*14.375)
		T=args[9]
		filter_step(correct_imu_vecs((args[0:3],args[3:6],np.array(args[6:9])*np.pi/(180*14.375)),args[9])
	return f

# ----------------------------------------------------------------------------------------
def float_array(a):
	return np.array(a,dtype=float)

# bare bones (je na uzivateli, aby pouzil spravny register_callback a spravny filter advancer)
def start_realtime_filtering(register_callback,imu,advance_filter_state,filter_state,delta_t):
	imu.set_all_data_period(0)
	filter_state_copy = [float_array(filter_state)]
	def callback(*args):
		advance_filter_state(args,delta_t,filter_state)
		filter_state_copy[0] = float_array(filter_state)
	register_callback(imu, callback)
	imu.set_all_data_period(delta_t*1000)
	return lambda : filter_state_copy[0]

# ----------------------------------------------------------------------------------------

def data_to_quantities(d):
	a = np.array
	return {
		"a_in_mg":a(d[0:3]),
		"B_in_mG":a(d[3:6]),
		"omega_in_rad_per_s":a(d[6:9])*np.pi/(180*14.375),
		"T":d[9]
	}

def compose_with_data_to_quantities(f):
	def g(d):
		return f(data_to_quantities(d))
	return g

def register_callback_adapted(imu,callback):
	imu.register_callback(imu.CALLBACK_ALL_DATA, compose_with_data_to_quantities(callback))

#-----------


def start_realtime_filtering_conv(imu,advance_filter_state,filter_state,delta_t):
	def afs(quantities,dt,filter_state):
		return advance_filter_state(quantities...,dt,filter_state)
	return start_realtime_filtering(register_callback_adapted,imu,afs,filter_state,delta_t)

def correct_imu_vecs(quantities):


def start_realtime_filtering(imu,,,filter_state,delta_t)

def start_realtime_orientation_filtering(imu,C_imu,delta_t):
	def corrected_imu_vectors(canon_data)
	filter_state = ba.initial_state()
	filter_step = 
	return start_realtime_filtering(imu,filter_step,filter_state,delta_t)
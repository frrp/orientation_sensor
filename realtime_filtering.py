import from_imu_brick.brick_ahrs as ba
import utils.quaternions as quat
import numpy as np

#http://stackoverflow.com/questions/4964101/pep-3118-warning-when-using-ctypes-array-as-numpy-array

def stop_realtime_filtering(imu):
	#consider clearing the registered callback
	imu.set_all_data_period(0)

def float_array(a):
	return np.array(a,dtype=float)

def start_realtime_filtering(imu,advance_filter_state,filter_state,delta_t):
	imu.set_all_data_period(0)
	filter_state_copy = [float_array(filter_state)]
	def callback(*args):
		advance_filter_state(args,delta_t,filter_state)
		filter_state_copy[0] = float_array(filter_state)
	imu.register_callback(imu.CALLBACK_ALL_DATA, callback)
	imu.set_all_data_period(delta_t*1000)
	return lambda : filter_state_copy[0]

# ----------------------------------------------------------------------------------------

# why 14.375? see imu brick documentation
#np.pi/(180*14.375) = 0.0012141420883438813
def data_to_quantities(d):
	a = np.array
	return (a(d[0:3]),a(d[3:6]),a(d[6:9])*0.0012141420883438813),d[9]

def get_filter_helper(calculate_corrected_vectors,beta,zeta):
	def f(data,delta_t,filter_state):
		a_B_omega = calculate_corrected_vectors(data_to_quantities(data))
		ba.update_filter((a_B_omega,beta,zeta,delta_t),filter_state)
	return f

def f_current_xyzw_quat_x_north_z_up(imu,calculate_corrected_vectors,beta,zeta,delta_t):
	advance_filter_state = get_filter_helper(calculate_corrected_vectors,beta,zeta)
	current_wxyz_quaternion_z_down = start_realtime_filtering(imu,advance_filter_state,ba.initial_state(),delta_t)
	def f():
		return quat.mul_xyzw_quaternion([1,0,0,0],current_wxyz_quaternion_z_down()[[1,2,3,0]])
	return f

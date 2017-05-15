import from_imu_brick.brick_ahrs as ba
import brick_calibration as bc
import numpy as np


def stop_orientation_streaming(imu):
	#consider clearing the registered callback
	imu.set_all_data_period(0)

#todo: incorporate temperature and calibrate the gyro, too !!!! !!!!
#achjo, ani nevim, jestli to je spravne...

def corrected_a_B(sensor_corrections,(a_raw,B_raw)):
	return (
		bc.apply_affine_transformation(a_raw,sensor_corrections["G_a"]),
		bc.apply_affine_transformation(B_raw,sensor_corrections["G_B"])
	)

# todo: make it possible to use several filter simultaneously


#14.375 is a strange constant documented in imu brick documentation
def get_filter(T_all,beta,zeta):
	def f((omega,a_raw,B_raw,delta_t),private_filter_state):
		a_corr, B_corr = corrected_a_B(T_all["sensor_corrections"],(np.array(a_raw),np.array(B_raw)))
		ba.update_filter((np.array(omega)*np.pi/(180*14.375),a_corr,B_corr,beta,zeta,delta_t),private_filter_state)		
	return f

def start_orientation_streaming(imu,filter,delta_t,target):
	imu.set_all_data_period(delta_t*1000)
	private_filter_state = ba.initial_state()
	def callback(ax,ay,az,Bx,By,Bz,ox,oy,oz,T):
		filter(((ox,oy,oz),(ax,ay,az),(Bx,By,Bz),delta_t),private_filter_state)
		state_copy = ba.initial_state()
		for i in range(len(state_copy)): state_copy[i] = private_filter_state[i]
		target[0] = state_copy # let us hope it is at least somewhat thread safe...
	imu.register_callback(imu.CALLBACK_ALL_DATA, callback)

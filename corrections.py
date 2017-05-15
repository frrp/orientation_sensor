import numpy as np
import utils.transformations as tfms
import utils.quaternions as quat
import sensor_corrections as sc

def identity_imu_corrections():
	return {
		"a":sc.identity_sensor_corrections(),
		"B":sc.identity_sensor_corrections(),
		"omega":sc.identity_sensor_corrections(),
		"common_rotvec":np.zeros(3)
	}

def gas_from_imu_corrections(imu_corrections):
	common_R = quat.R_from_rotvec_row(imu_corrections["common_rotvec"])
	return [tfms.ga_times_gl(sc.ga_from_sensor_corrections(imu_corrections[name]),common_R) for name in "a","B","omega"]

def apply_ga_imu_corrections(a_B_omega,gas):
	return map(tfms.apply_ga,a_B_omega,gas)


#convenience
def f_calculate_corrected_vectors(imu_corrections):
	gas = gas_from_imu_corrections(imu_corrections)
	def f(a_B_omega):
		return apply_ga_imu_corrections(a_B_omega,gas)
	return f

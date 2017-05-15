import utils.quaternions as quat
from utils.normalized import *
import acquire_data as ad

from utils.degree import degree

import utils.transformations as tfms

import corrections as cs

import numpy as np

def modify_array(a,max_delta):
	return a+np.random.uniform(-max_delta,max_delta,a.shape)

# orientation: north = +x, west = +y, up = +z
def geo_unitvecs_from_a_normalized_and_B((a_normalized,B)):
	result = np.zeros(a_normalized.shape[:-1]+(3,3))
	result[...,2,:] = - a_normalized
	result[...,1,:] = normalized_vec(np.cross(B,a_normalized))
	result[...,0,:] = np.cross(result[...,1,:],result[...,2,:])
	return result

def rms(a):
	return np.linalg.norm(a)/np.sqrt(a.size)

def relative_error(mean_func,a,ideal_value):
	return mean_func(a-ideal_value)/ideal_value

def weighted_less(weights):
	def less(a,b):
		return np.dot(a,weights)<np.dot(b,weights)
	return less

# specific and convenience functions

g_0 = 9.80665

constants_prague = {
	"g":9.81373,
	"B_in_mG":490.0, # consider using Teslas instead
	"mag_inclination":66.0/180.0 * np.pi,
	"mag_declination":3.32 / 180.0 *np.pi # 3 deg 19 min 27 sec E changing by  6.5 min E per year
}

def invariants_imu(local_constants):
	return (
		local_constants["g"]/g_0*1000,
		local_constants["B_in_mG"]
	)

#C=cs.identity_corrections()

#m=[]
#m.append(adc.acquire_measurement())

def search_main(C, modify_corrections, evaluate_error, error_less, n_steps):
	n_successful_steps = 0
	error = evaluate_error(C)
	for blah in xrange(n_steps):
		C_candidate = modify_corrections(C)
		error_candidate = evaluate_error(C_candidate)
		if error_less(error_candidate,error):
			error = error_candidate
			C.update(C_candidate)
			n_successful_steps+=1
	print n_successful_steps
	print error

def modify_sensor_correction(C_sensor,(max_d_scaling,max_d_displacement,max_d_rotvec)):
	return {
		"scaling":(modify_array(C_sensor["scaling"][0],max_d_scaling),modify_array(C_sensor["scaling"][1],max_d_scaling)),
		"displacement":modify_array(C_sensor["displacement"],max_d_displacement),
		"rotvec":modify_array(C_sensor["rotvec"],max_d_rotvec)
	}

def modify_imu_corrections(C_imu,(max_d_scaling,max_d_displacement,max_d_B_rotvec,max_d_common_rotvec)):
	return {
		"a":modify_sensor_correction(C_imu["a"],(max_d_scaling,max_d_displacement,0)),
		"B":modify_sensor_correction(C_imu["B"],(max_d_scaling,max_d_displacement,max_d_B_rotvec)),
		"omega":modify_sensor_correction(C_imu["omega"],(0,0,0)),
		"common_rotvec":modify_array(C_imu["common_rotvec"],max_d_common_rotvec)
	}

def f_modify_corrections(max_d_scaling,max_d_B_rotvec,max_d_common_rotvec,relative_max_d_displacement=100):
	max_d_displacement = max_d_scaling*relative_max_d_displacement
	def f(C):
		return {
			"imu":modify_imu_corrections(C["imu"],(max_d_scaling,max_d_displacement,max_d_B_rotvec,max_d_common_rotvec))
		}
	return f

def identity_corrections():
	return {"imu":cs.identity_imu_corrections()}

#weights = [w_error_a,w_error_B,w_sigma_inclination,w_sigma_heading] # a mozna i w_error_heading, pokud bych chtel zuzitkovat heading_dict !!!

def get_a_B(annotated_measurements):
	return np.array([ad.get_a(am["data"])[0] for am in annotated_measurements]),np.array([ad.get_B(am["data"])[0] for am in annotated_measurements])

#def affine_corrected_a_B((a_raw,B_raw),aic):
#	return tfms.apply_ga(a_raw,aic["a"]),tfms.apply_ga(B_raw,aic["B"])

#def f_get_corrected_a_B(C_imu):
#	aic = cs.affine_imu_corrections(C_imu)
#	def f(a_B_raw):
#		return affine_corrected_a_B(a_B_raw,aic)

#convenience or even just helper
#def f_get_corrected_factored_a_B(C_imu):
#	get_corr_a_B = f_get_corrected_a_B(C_imu)
#	def f(a_B_raw):
#		return map(normalized_vec_and_norm,get_corr_a_B(a_B_raw))

def f_evaluate_error(annotated_measurements,mean_func,ideal_a_norm,ideal_B_norm):#,reference_headings_dict=None):
	a_B_raw = get_a_B(annotated_measurements)
	#todo: process annotations
	def f(C):
		gas = cs.gas_from_imu_corrections(C["imu"])
		a_B_corr = map(tfms.apply_ga,a_B_raw,(gas[0],gas[1]))
		((a_normalized,a_norm),(B_normalized,B_norm)) = map(normalized_vec_and_norm,a_B_corr)
		e_a_norm, e_B_norm = [
			relative_error(mean_func,norm,ideal_norm) for norm,ideal_norm in
			(a_norm,ideal_a_norm),(B_norm,ideal_B_norm)
		]
		dot_a_B = np.sum(a_normalized*B_normalized,axis=-1)
		mean_dot_a_B = np.mean(dot_a_B)
		e_mag_inclination = mean_func(dot_a_B-mean_dot_a_B)
		e_heading_1 = 0 # TODO: mean_func of differences from the avg heading in each group
		e_heading_2 = 0 # TODO: mean_func of differences from actual reference headings
		return e_a_norm, e_B_norm, e_mag_inclination, e_heading_1, e_heading_2
	return f


def search_conv(C,evaluate_error,max_delta_scaling,max_delta_B_rotvec,max_delta_common_rotvec,weights,n_steps):
	mc=f_modify_corrections(max_delta_scaling,max_delta_B_rotvec,max_delta_common_rotvec)
	error_less = weighted_less(weights)
	search_main(C,mc,evaluate_error,error_less,n_steps)

def save_data(data,filename): # use with
	f=open(filename,"w")
	f.write(str(data))
	f.close()


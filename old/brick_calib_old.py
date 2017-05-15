# yaw, pitch for x=north,y=west,z=up -> yaw=-azimuth, pitch=-elevation
def yaw_pitch_to_xyz((yaw,pitch)):
	return np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),-np.sin(pitch)])

def azimuth_elevation_to_xyz((a,e)):
	return yaw_pitch_to_xyz((-a,-e))

def azimuth_theta_to_xyz((a,theta)):
	return yaw_pitch_to_xyz((-a,theta-pi/2))

def equirect_pano_xy_to_xyz(width,height):
	def f((x,y)):
		return azimuth_theta_to_xyz(((2*pi*x)/width,(pi*y)/height))
	return f

def create_reference_xyz_heading_dict(convert,source_dict,azimuth_correction):
	def f(heading):
		return apply_rotation(convert(heading),[0,0,-azimuth_correction])
	return mapdict(f,source_dict)

def prepare_data(measured_data,reference_heading_dict):
	a_raw = np.array([get_a(row) for row in measured_data])
	B_raw = np.array([get_B(row) for row in measured_data])
	normalized_reference_heading = np.array([
		normalized_vec(reference_heading_dict[get_name(row)]) for row in measured_data
	])
	return ((a_raw,B_raw),normalized_reference_heading)
#-------------------
def f_evaluate_error_conv(local_constants,f_mean,measured_data,reference_heading_dict):
	f_di = f_deviation_from_invariants(invariants_imu(local_constants),f_mean)
	prepared_data = prepare_data(measured_data,reference_heading_dict)
	def f(T_all):
		return evaluate_error(T_all,prepared_data,f_di,f_mean)
	return f

def f_search(*f_evaluate_error_conv_args):
	f_ee = f_evaluate_error_conv(*f_evaluate_error_conv_args)
	def f(T_all,weights,delta,n_steps):
		search_all(
			T_all,
			f_modify_all_corrections((delta*0.01,delta),delta*0.01),
			f_ee,
			weighted_less(weights),
			n_steps)
	return f
#----------------------

# ....def f_deviation_from_invariants(
# 	(ideal_a_norm,ideal_B_norm,ideal_inclination),
# 	mean_func):
# 	def f(factored_a_B):
# 		((a_normalized,a_norm),(B_normalized,B_norm)) = factored_a_B
# 		e_a_norm, e_B_norm = [
# 			relative_error(mean_func,norm,ideal_norm) for norm,ideal_norm in
# 			(a_norm,ideal_a_norm),(B_norm,ideal_B_norm)
# 		]
# 		e_mag_inclination = mean_func(np.sum(a_normalized*B_normalized,axis=-1)-np.cos(np.pi/2-ideal_inclination))
# 		#sigma_mag_incl = ... np.std(np.sum(a_normalized*B_normalized,axis=-1))#todo: remove this temporary line
# 		return e_a_norm, e_B_norm, e_mag_inclination
# 	return f

# ....

# convenience function # todo consider removing
#def corrected_normalized_imu_vectors(sensor_corrections,(a_raw,B_raw)):
#	(a_normalized,a_norm),(B_normalized,B_norm) =
#		corrected_factored_imu_vectors(sensor_corrections,a_B_raw)
#	return (a_normalized,B_normalized)


#def corrected_sensor_unitvecs(rot_vec,a_normalized_and_B):
#	geo_unitvecs_raw = geo_unitvecs_from_a_normalized_and_B(a_normalized_and_B)
#	sensor_unitvecs_raw = np.rollaxis(geo_unitvecs_raw,-1,-2)#transpose the last two dimensions
#	return apply_rotation(sensor_unitvecs_raw,rot_vec)

def evaluate_error(T_all,prepared_data,f_deviation_from_invariants,mean_func):
	(a_B_raw,normalized_reference_heading) = prepared_data
	(a_normalized,a_norm),(B_normalized,B_norm) = a_B_factored = corrected_factored_imu_vectors(T_all["sensor_corrections"],a_B_raw)
	e_a_norm, e_B_norm, e_mag_inclination = f_deviation_from_invariants(a_B_factored)
	sensor_unitvecs = corrected_sensor_unitvecs(T_all["R_E"],(a_normalized, B_normalized))
	heading_x = sensor_unitvecs[...,0,:]
	e_heading = mean_func(np.linalg.norm(normalized_reference_heading-heading_x,axis=-1))
	#gogo[0] = linalg.norm(normalized_reference_heading-heading_x,axis=-1)
	return e_a_norm, e_B_norm, e_mag_inclination, e_heading

#convenience function
def corrected_sensor_unitvecs_2(T_all,a_B_raw):
	(a_normalized,a_norm),(B_normalized,B_norm) = corrected_factored_imu_vectors(T_all["sensor_corrections"],a_B_raw)
	return corrected_sensor_unitvecs(T_all["R_E"],(a_normalized, B_normalized))

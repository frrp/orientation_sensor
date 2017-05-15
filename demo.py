import mercator2 as m2
import utils.animate as anim
import imu
import utils.quaternions as quat
import realtime_filtering as rf
import corrections as cs
import numpy as np
from numpy import array

content_dir = "../content"

project_name = "proof of principle"

image_name = "main"

a_B_omega_ga_corrections_old = (
	(
		array([[ 1.01079821, -0.02344393, -0.00321683],[-0.02344393,  0.99385636, -0.00358059],[-0.00321683, -0.00358059,  0.97909862]]),
		array([ 31.590834  , -23.76157851,  24.70842715])),
	(
		array([[ 0.99276953,  0.00307519, -0.00610817],[-0.04457501,  0.97221875,  0.03443869],[-0.00433349, -0.00266275,  1.02375344]]),
   		array([-68.93173808,  98.85824018,  -9.75689028])),
	(np.eye(3),np.zeros(3)))

a_B_omega_ga_corrections_separate_sphericity_and_incl_spread = [(array([[ 1.01007018, -0.02342757, -0.00318177],
       [-0.02342757,  0.99314372, -0.00358655],
       [-0.00318177, -0.00358655,  0.97839609]]),
  array([ 32.39010488, -24.43163908,  24.16618496])),
 (array([[ 1.01304999,  0.00272285, -0.0052757 ],
       [-0.04512514,  0.99208932,  0.03472252],
       [-0.0054067 , -0.00233477,  1.04464352]]),
  array([-74.23799463,  97.91019442,  -6.39883499])),
 (array([[ 1.,  0.,  0.],
       [ 0.,  1.,  0.],
       [ 0.,  0.,  1.]]),
  array([ 0.,  0.,  0.]))]

a_B_omega_ga_corrections_together_sphericity_and_incl_spread = [(array([[ 1.01012443, -0.01911432, -0.00200058],
       [-0.01911432,  0.99349189, -0.00102812],
       [-0.00200058, -0.00102812,  0.97779267]]),
  array([ 33.16021537, -23.62277744,  24.30699753])),
 (array([[ 1.01459469,  0.00191331, -0.00577173],
       [-0.04573615,  0.99216624,  0.03433628],
       [-0.00513574, -0.00290812,  1.04266259]]),
  array([-75.18554779,  97.84378296,  -7.07567823])),
 (array([[ 1.,  0.,  0.],
       [ 0.,  1.,  0.],
       [ 0.,  0.,  1.]]),
  array([ 0.,  0.,  0.]))]


a_B_omega_ga_corrections = a_B_omega_ga_corrections_separate_sphericity_and_incl_spread

def default_demo(panorama_name,language,delay,zoom):
	return m2.demo(content_dir,project_name,panorama_name,image_name,language,delay,zoom)

def run_demo(panorama_name,language,delay,zoom,flip = True,rotvec = [0,0,0],corrs = a_B_omega_ga_corrections):
	#ccv = cs.f_calculate_corrected_vectors(C_imu)
	#def ccv2((a_B_omega,T)):
	#	return ccv(a_B_omega)
	def calc_corr_vecs_T((a_B_omega,T)):
		return cs.apply_ga_imu_corrections(a_B_omega,a_B_omega_ga_corrections)
	current_xyzw_quat_x_north_z_up = rf.f_current_xyzw_quat_x_north_z_up(imu.imu,calc_corr_vecs_T,0.02,0.001,0.002)
	frame_source = default_demo(panorama_name,language,delay,zoom)
	q = quat.xyzw_quaternion_from_rotvec_row(rotvec)
	anim.animate(frame_source,lambda: quat.mul_xyzw_quaternion(q,current_xyzw_quat_x_north_z_up()),flip)
	rf.stop_realtime_filtering(imu.imu)

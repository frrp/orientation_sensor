import numpy as np
import utils.transformations as tfms
import utils.quaternions as quat

def identity_sensor_corrections():
	return {
		"scaling":tfms.identity_scaling(),
		"displacement":np.zeros(3),
		"rotvec":np.zeros(3)
	}

def ga_from_sensor_corrections(sc):
	R = quat.R_from_rotvec_row(sc["rotvec"])
	gl_scaling = tfms.gl_from_scaling(sc["scaling"])
	return tfms.ga_times_gl((gl_scaling,sc["displacement"]),R)

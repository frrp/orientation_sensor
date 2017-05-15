import numpy as np

#def normalized(x):
#	return x/np.linalg.norm(x)

def normalized_vec_and_norm(a):
	n = np.linalg.norm(a,axis=-1)
	return a/n[...,np.newaxis],n

def normalized_vec(a):
	return normalized_vec_and_norm(a)[0]

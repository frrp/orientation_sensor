from scipy.linalg import block_diag
from numpy import eye

__all__ = ["linear_as_affine","translation_as_affine"]

def linear_as_affine(a):
	return block_diag(a,1)

def translation_as_affine(a):
	result = eye(len(a)+1)
	result[0:len(a),-1]=a
	return result

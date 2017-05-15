import numpy as np

# all 3d vectors are represented as row vectors!

def apply_gl(v,GL):
	return np.tensordot(v,GL,(-1,-2))

def apply_ga(v,(GL,d)):
	return apply_gl(v,GL)+d
#------
def ga_times_gl((GL1,d),GL2):
	return apply_gl(GL1,GL2),apply_gl(d,GL2)
#------

def gl_from_scaling((diagonal,off_diagonal)):
	GL = np.diag(diagonal)
	GL[1][2]=off_diagonal[0]
	GL[2][1]=off_diagonal[0]
	GL[0][2]=off_diagonal[1]
	GL[2][0]=off_diagonal[1]
	GL[0][1]=off_diagonal[2]
	GL[1][0]=off_diagonal[2]
	return GL

def identity_scaling():
	return (np.ones(3),np.zeros(3))


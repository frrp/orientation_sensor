import brick_ahrs as ba
import numpy as np

def _state_to_array(s):
	return np.array(list(s),dtype=float)

def _state_from_array_like(a):
	tmp = ba.initial_state()
	for i in range(len(tmp)):
		tmp[i]=q[i]
	return tmp

def initial_state():
	return _state_to_array(ba.initial_state())

def update_filter(input,q):
	tmp = _state_from_array_like(q)
	ba.update_filter(input,tmp)
	return _state_to_array(tmp)


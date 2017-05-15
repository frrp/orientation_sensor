import utils.mplayer_pipe as mplayer_pipe
import utils.quaternions as quaternions
from utils.isdata import isData
from utils.normalized import normalized_vec
from time import sleep

from numpy import *

__all__ = ["animate","testing_quaternion_source"]

def testing_quaternion_source(speed=0.1):
	q = array([0,0,0,1],dtype=float)
	omega = array([0,0,0],dtype=float)
	def get_q():
		omega[:] = 0.99*omega+0.01*speed*random.normal((0,0,0))
		v_q = normalized_vec(concatenate([omega,[1]]))
		q[:] = normalized_vec(quaternions.mul_xyzw_quaternion(v_q,q))
		return q
	return get_q

#todo: zjistit, pro se ten kvaternion musel nasobit zleva a ne zprava

def animate(renderer,get_quaternion, flip = False, frame_duration = 0.1):
	one_frame = renderer([0,0,0,1])
	vs = mplayer_pipe.VideoSink((one_frame.shape[0],one_frame.shape[1]),1000,byteorder = "rgb24",flip = flip)
	while(not isData()):
		sleep(frame_duration)
		vs.run(renderer(get_quaternion()))
	vs.close()

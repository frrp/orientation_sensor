import subprocess

__all__ = ["VideoSink"]

class VideoSink(object) :
	def __init__( self, size,rate=100, byteorder="rgb24", flip = False) :
		self.size = size
		cmdstring  = ('mplayer',
				'-')+(('-vf', 'flip') if flip else ())+('-demuxer', 'rawvideo',
				'-rawvideo', 'w=%i:h=%i'%size[::-1]+":fps=%i:format=%s"%(rate,byteorder)
		)
		self.p = subprocess.Popen(cmdstring, stdin=subprocess.PIPE, shell=False)
	def run(self, image) :
		#assert image.shape == self.size does not work
		#image.tofile(self.p.stdin) # should be faster but it is indeed slower
		self.p.stdin.write(image.tostring())
	def close(self) :
		self.p.stdin.close()

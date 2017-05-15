def mapdict(f,d):
	return dict((key,f(value))for key,value in d.iteritems())

import select
import sys

__all__  = ["isData"]

def isData():
	return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

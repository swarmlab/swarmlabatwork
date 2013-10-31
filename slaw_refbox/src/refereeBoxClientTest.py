#!/usr/bin/python

from refereeBoxClient import *


if __name__ == "__main__":
	print obtainTaskSpecFromServer("127.0.0.1", "11111", "swarmlab@work")

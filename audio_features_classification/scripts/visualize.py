#!/usr/bin/env python
import numpy
import rospy
import sys
import os
import matplotlib.pyplot as plt
import socket

if __name__ == '__main__':

	try:
	    classNames = rospy.get_param('/audio_features_classifier/classes', {'silence', 'speech'})
	    for a in classNames:
	        plt.plot(numpy.load(os.path.dirname(os.path.realpath(sys.argv[0]))+'/classifier_data/'+a+'.npy')[:,0])
	    plt.show()
	except socket.error:
		print("Unable to communicate with master! Please run roscore first!")
        

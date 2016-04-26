#!/usr/bin/env python
import numpy
import rospy
import sys
import os
import matplotlib.pyplot as plt

if __name__ == '__main__':

	rospy.init_node("class_viz_node")
	classNames = rospy.get_param('~classes', {'silence', 'speech'})
	classNames = classNames.split()
	for a in classNames:
		plt.plot(numpy.load(os.path.dirname(os.path.realpath(sys.argv[0]))+'/classifier_data/'+a+'.npy')[:,0])
	plt.show()
        

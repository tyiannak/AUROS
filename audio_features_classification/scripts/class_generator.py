#!/usr/bin/env python
import numpy
import rospy
from audio_features_extraction.msg import featMsg
import signal
import sys
import os
from pyAudioAnalysis import audioTrainTest
import matplotlib.pyplot as plt

features_subscriber = None

global mtFeaturesMatrix
global prevTime
global count
global className
global start_time

mtFeaturesMatrix = []
prevTime = 0
count = 0
className = ""
start_time = 0

def signal_handler(signal, frame):
    global mtFeaturesMatrix
    global className
    mtFeaturesMatrix = numpy.array(mtFeaturesMatrix)    
    numpy.save(os.path.dirname(os.path.realpath(sys.argv[0]))+'/classifier_data/'+className,mtFeaturesMatrix)
    print('You pressed Ctrl+C!')
    print mtFeaturesMatrix.shape
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def initSubscriber():
    global features_subscriber
    global className
    rospy.init_node("class_generator_node")
    className = rospy.get_param('~current_class', 'silence')
    topic = rospy.get_param('~features_topic','/audio_features_extraction/features')
    features_subscriber = rospy.Subscriber(topic, featMsg, featuresCallback)
    print "Waiting for features_topic to be published..."
    rospy.spin()


def featuresCallback(feat_msg):
    global prevTime, count, mtFeaturesMatrix, start_time

    curFV = feat_msg.ltWin1mean + feat_msg.ltWin1deviation                                              # merge long term mean and std feature statistics (from the respective topic)        
    curFV = list(curFV)
    del curFV[18]    
    
    if count == 0:
        start_time = feat_msg.time                                                                      # get current timestamp

    mtFeaturesMatrix.append(curFV)
    print "{0:.3f}\t{1:.3f}".format(float(count) / 20.0, feat_msg.time-start_time)

    prevTime = feat_msg.time                                                                            # get current timestamp
    count += 1                                                                                          # update global counter

if __name__ == '__main__':
    initSubscriber()
        

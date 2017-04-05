#!/usr/bin/env python
import numpy
import rospy
from audio_features_extraction.msg import featMsg
import signal
import sys
import os
from pyAudioAnalysis import audioTrainTest
import matplotlib.pyplot as plt
import mlpy
features_subscriber = None

global mtFeaturesMatrix
global className

mtFeaturesMatrix = []
prevTime = 0
count = 0
className = ""
start_time = 0

def signal_handler(signal, frame):
    global mtFeaturesMatrix
    global className
    mtFeaturesMatrix = numpy.array(mtFeaturesMatrix)    
    cls, means, steps = mlpy.kmeans(mtFeaturesMatrix, k=2, plus=True)
    plt.plot(cls)
    plt.show()
    #numpy.save(os.path.dirname(os.path.realpath(sys.argv[0]))+'/classifier_data/'+className,mtFeaturesMatrix)
    print('You pressed Ctrl+C!')
    print mtFeaturesMatrix.shape
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def initSubscriber():
    global features_subscriber
    global className
    rospy.init_node("giannis_node")
    topic = rospy.get_param('~features_topic','/audio_features_extraction/features')
    features_subscriber = rospy.Subscriber(topic, featMsg, featuresCallback)
    print "Waiting for features_topic to be published..."
    rospy.spin()


def featuresCallback(feat_msg):
    global prevTime, count, mtFeaturesMatrix, start_time

    curFV = feat_msg.ltWin1mean + feat_msg.ltWin1deviation                                              # merge long term mean and std feature statistics (from the respective topic)        
    curFV = list(curFV)
    #del curFV[18]    
    
    if count == 0:
        start_time = feat_msg.time                                                                      # get current timestamp

    mtFeaturesMatrix.append(curFV)
    print len(mtFeaturesMatrix) 

    print "{0:.3f}\t{1:.3f}\t - class: {2:s}".format(float(count) / 20.0, feat_msg.time-start_time, className)
    '''
    if count % 10 == 0:
        plt.clf()
        ax1 = plt.subplot(2,1,1)
        plt.plot(curFV[0:len(curFV)/2])
        ax2 = plt.subplot(2,1,2)
        plt.plot(curFV[len(curFV)/2::])
        plt.ion()
        plt.show()
        ax1.set_ylim([0,20])
        ax2.set_ylim([0,20])
        plt.draw()
    '''
    prevTime = feat_msg.time                                                                            # get current timestamp
    count += 1                                                                                          # update global counter

if __name__ == '__main__':
    initSubscriber()
        

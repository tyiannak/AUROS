#!/usr/bin/env python
import cPickle
import numpy
import roslib, rospy
from audio_features_extraction.msg import featMsg
import signal
import sys
import os
from pyAudioAnalysis import audioTrainTest
import matplotlib.pyplot as plt
import collections

features_subscriber = None
NUMBER_OF_LT_WINDOWS = 100

global mtFeaturesMatrix
global prevTime
global count
global className
global modelName
global classifierInfo
global start_time
global energies
start_time = 0
classifierInfo = {}
modelName = ""
prevTime = 0
count = 0
mtFeaturesMatrix = []
energies = collections.deque(maxlen=NUMBER_OF_LT_WINDOWS)

def initSubscriber():
    global features_subscriber
    global modelName                               # if modelName is provided, then the respective model is loaded:
    global classifierInfo 

    rospy.init_node("audio_features_classifier_node")              # start the processing node
    modelName = rospy.get_param('/audio_features_classifier/classifier_name', 'modelSVM')       
    [Classifier, MEAN, STD, classNames, mtWin, mtStep, stWin, stStep, computeBEAT] = audioTrainTest.loadSVModel(os.path.dirname(os.path.realpath(sys.argv[0]))+'/classifier_data/'+modelName)
    classifierInfo["Classifier"] = Classifier
    classifierInfo["MEAN"] = MEAN
    classifierInfo["STD"] = STD
    classifierInfo["classNames"] = classNames
    classifierInfo["mtWin"] = mtWin
    classifierInfo["mtStep"] = mtStep
    classifierInfo["stWin"] = stWin
    classifierInfo["stStep"] = stStep
    classifierInfo["computeBEAT"] = computeBEAT
    print MEAN
    features_subscriber = rospy.Subscriber("/audio_features_extraction/features", featMsg, featuresCallback)    # subscribe the featuresCallback() callback function
    print "Waiting for features_topic to be published..."
    rospy.spin()


def featuresCallback(feat_msg):
    global prevTime, count, mtFeaturesMatrix, start_time

    curFV = feat_msg.ltWin1mean + feat_msg.ltWin1deviation                                              # merge long term mean and std feature statistics (from the respective topic)        
    curFV = list(curFV)
    del curFV[18]    
    
    if count == 0:
        start_time = feat_msg.time
                                                                              # TESTING MODE
    global classifierInfo, energies, energies2        
    curFVOr = curFV
    curFV = (curFV - classifierInfo["MEAN"]) / classifierInfo["STD"]                                # feature normalization                        
    [Result, P] = audioTrainTest.classifierWrapper(classifierInfo["Classifier"], "svm", curFV)      # classification
    classResult = classifierInfo["classNames"][int(Result)]
    #print "{0:.5f}\t{1:.3f}\t{2:.3f}\t{3:s}".format(feat_msg.features[0], float(count) / 20.0, feat_msg.time-start_time, classResult)
    
    #energies = numpy.append(energies, feat_msg.features[0])                                     # append mid-term energy (1st feature)        
    if classResult == "silence":
        energies.append(curFVOr[0])
    else:
        energies.append(sum(energies)/(float(len(energies))+0.0001))
    print "{0:s}\t{1:.5f}\t{2:.5f}\t{3:.5f}".format(classResult, feat_msg.features[0], curFVOr[0],sum(energies)/float(len(energies)))
    
    prevTime = feat_msg.time                                                                            # get current timestamp
    count += 1                                                                                          # update global counter

if __name__ == '__main__':
    initSubscriber()
        

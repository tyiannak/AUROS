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
from audio_features_classification.msg import classificationResult

features_subscriber = None
classification_publisher = None
NUMBER_OF_LT_WINDOWS = 100

global mtFeaturesMatrix
global className
global modelName
global classifierInfo
global energies
classifierInfo = {}
modelName = ""
mtFeaturesMatrix = []
energies = collections.deque(maxlen=NUMBER_OF_LT_WINDOWS)

def initSubscriber():
    global features_subscriber
    global modelName
    global classifierInfo 
    global classification_publisher

    rospy.init_node("audio_features_classifier_node")
    modelName = rospy.get_param('~classifier_name', 'modelSVM')       
    sub_topic = rospy.get_param('~features_topic','/audio_features_extraction/features')
    pub_topic = rospy.get_param('~classification_topic','~audio_classification')
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
    classification_publisher = rospy.Publisher("~"+pub_topic, classificationResult, queue_size=10)
    features_subscriber = rospy.Subscriber(sub_topic, featMsg, featuresCallback)
    print "Waiting for features_topic to be published..."
    rospy.spin()


def featuresCallback(feat_msg):
    global mtFeaturesMatrix
    global classifierInfo, energies, classification_publisher

    curFV = feat_msg.ltWin1mean + feat_msg.ltWin1deviation  #merge long term mean and std feature statistics (from the respective topic)        
    curFV = list(curFV)
    #del curFV[18]    

    curFVOr = curFV
    curFV = (curFV - classifierInfo["MEAN"]) / classifierInfo["STD"]                                # feature normalization                        
    [Result, P] = audioTrainTest.classifierWrapper(classifierInfo["Classifier"], "svm", curFV)      # classification
    classResult = list(classifierInfo["classNames"])[int(Result)]

    EnergyThreshold = 0.90 * sum(energies)/float(len(energies)+0.00000001)
    if classResult == "silence":
        energies.append(curFVOr[0])
    else:
        if curFVOr[0] < EnergyThreshold:
            classResult = "silence"
            energies.append(curFVOr[0])
        else:
            energies.append(sum(energies)/(float(len(energies))+0.0001))

    class_pub = classificationResult()
    class_pub.header.stamp = rospy.Time.now()
    class_pub.class_result.data = str(classResult)
    class_pub.probability.data = float(P[int(Result)])
    classification_publisher.publish(class_pub)
    print curFVOr
    print numpy.nonzero(numpy.isnan(numpy.array(curFVOr).mean(axis = 0))), classResult
    
if __name__ == '__main__':
    initSubscriber()
        

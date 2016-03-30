#!/usr/bin/env python
import cPickle
import numpy
import roslib, rospy
from paexample.msg import featMsg
import signal
import sys
from pyAudioAnalysis import audioTrainTest
import matplotlib.pyplot as plt

features_subscriber = None

global mtFeaturesMatrix
global prevTime
global count
global className
global modelName
global classifierInfo
global start_time
start_time = 0
classifierInfo = {}
modelName = ""
className = ""
prevTime = 0
count = 0
mtFeaturesMatrix = []

def signal_handler(signal, frame):
    global mtFeaturesMatrix
    global className
    mtFeaturesMatrix = numpy.array(mtFeaturesMatrix)    
    numpy.save(className,mtFeaturesMatrix)
    print('You pressed Ctrl+C!')
    print mtFeaturesMatrix.shape
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def initSubscriber():
    global features_subscriber
    global modelName    
    if len(modelName)>0:
        global classifierInfo        
        [Classifier, MEAN, STD, classNames, mtWin, mtStep, stWin, stStep, computeBEAT] = audioTrainTest.loadSVModel(modelName)
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

    rospy.init_node("features_subscriber")
    features_subscriber = rospy.Subscriber("/features_topic", featMsg, featuresCallback)
    print "Waiting for features_topic to be published..."
    rospy.spin()


def featuresCallback(feat_msg):
    global prevTime
    global count
    global mtFeaturesMatrix    
    global start_time
    curFV = feat_msg.ltWin1mean + feat_msg.ltWin1deviation    
    if count == 0:
        start_time = feat_msg.time

    if len(className)>0:
        mtFeaturesMatrix.append(curFV)
        print "{0:.3f}\t{1:.3f}".format(float(count) / 20.0, feat_msg.time-start_time)

    if len(modelName)>0:
        global classifierInfo
        curFV = (curFV - classifierInfo["MEAN"]) / classifierInfo["STD"]                # normalization
        [Result, P] = audioTrainTest.classifierWrapper(classifierInfo["Classifier"], "svm", curFV)    # classification                
        print "{0:.5f}\t{1:.3f}\t{2:.3f}\t{3:s}".format(feat_msg.features[0], float(count) / 20.0, feat_msg.time-start_time, classifierInfo["classNames"][int(Result)])
    
    prevTime = feat_msg.time
    count += 1    

if __name__ == '__main__':

    if sys.argv[1] == "run":
        global modelName
        modelName = sys.argv[2]
        initSubscriber()

    if sys.argv[1] == "addClass":
        global className
        className = sys.argv[2]
        initSubscriber()

    elif sys.argv[1] == "train":
        modelName = sys.argv[2]
        features = []
        classNames = []
        for a in sys.argv[3::]:
            features.append(numpy.load(a))
            classNames.append(a.replace(".npy",""))
        classifierParams = numpy.array([0.001, 0.01,  0.5, 1.0, 5.0])
        nExp = 50
        bestParam = audioTrainTest.evaluateClassifier(features, classNames, nExp, "svm", classifierParams, 0, perTrain = 0.01)
        [featuresNorm, MEAN, STD] = audioTrainTest.normalizeFeatures(features)        # normalize features
        MEAN = MEAN.tolist()
        STD = STD.tolist()
        featuresNew = featuresNorm

        # STEP C: Save the classifier to file    
        Classifier = audioTrainTest.trainSVM(featuresNew, bestParam)
        Classifier.save_model(modelName)
        fo = open(modelName + "MEANS", "wb")
        cPickle.dump(MEAN, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(STD, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(classNames, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(0, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(0, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(0, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(0, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        cPickle.dump(0, fo, protocol=cPickle.HIGHEST_PROTOCOL)
        fo.close()
    elif "temp":
        F1 = numpy.load("silence.npy")
        F2 = numpy.load("activity.npy")
        plt.plot(F1[:,0],'r')
        plt.plot(F2[:,0],'g')
        plt.show()
        

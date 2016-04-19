#!/usr/bin/env python
import cPickle
import numpy
import roslib, rospy
from paexample.msg import featMsg
import signal
import sys
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
#global energies2
start_time = 0
classifierInfo = {}
modelName = ""
className = ""
prevTime = 0
count = 0
mtFeaturesMatrix = []
energies = collections.deque(maxlen=NUMBER_OF_LT_WINDOWS)

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
    if len(modelName)>0:                                # if modelName is provided, then the respective model is loaded:
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

    rospy.init_node("features_subscriber")              # start the processing node
    features_subscriber = rospy.Subscriber("/features_topic", featMsg, featuresCallback)    # subscribe the featuresCallback() callback function
    print "Waiting for features_topic to be published..."
    rospy.spin()


def featuresCallback(feat_msg):
    '''
    This is the main callback of the audio classification procedure
    '''

    # load global vars:
    global prevTime, count, mtFeaturesMatrix, start_time

    curFV = feat_msg.ltWin1mean + feat_msg.ltWin1deviation                                              # merge long term mean and std feature statistics (from the respective topic)
    if count == 0:
        start_time = feat_msg.time                                                                      # get current timestamp

    if len(className)>0:                                                                                # if className exists: TRAINING MODE 
        mtFeaturesMatrix.append(curFV)
        print "{0:.3f}\t{1:.3f}".format(float(count) / 20.0, feat_msg.time-start_time)

    if len(modelName)>0:                                                                                # TESTING MODE
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
            energies.append(sum(energies)/float(len(energies)))
        
        print "{0:s}\t{1:.5f}\t{2:.5f}\t{3:.5f}".format(classResult, feat_msg.features[0], curFVOr[0],sum(energies)/float(len(energies)))
    
    prevTime = feat_msg.time                                                                            # get current timestamp
    count += 1                                                                                          # update global counter

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
        bestParam = audioTrainTest.evaluateClassifier(features, classNames, nExp, "svm", classifierParams, 0, perTrain = 0.1)
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
        

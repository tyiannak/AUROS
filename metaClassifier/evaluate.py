import sys, os, json, numpy, datetime, ntpath
import matplotlib.pyplot as plt
import matplotlib, glob
from pyAudioAnalysis import audioSegmentation as aS
from pyAudioAnalysis import audioTrainTest as aT

allClasses = ["silence","boiler","speech","music","activity","waterwassin"]

def readGTFile(gtLogPath):
    startGT, endGT, labelGT = aS.readSegmentGT(gtLogPath)
    flagsGT, classNamesGT = aS.segs2flags(startGT, endGT, labelGT, 1.0)    
    classNamesGT2 = sortEventNames(classNamesGT)
    flagsGT2 = [classNamesGT2.index(classNamesGT[f]) for f in flagsGT]
    classNamesGT = classNamesGT2
    flagsGT = flagsGT2
    return flagsGT, classNamesGT

def sortEventNames(events):
    # sort events by name, but silence first:
    events = sorted(events)
    events2 = []
    if "silence" in events:
        events2.append("silence")
    for u in events:
        if u != "silence":
            events2.append(u)
    return events2

def readLogFile(fileName, startTime):
    # read data from json file
    with open (fileName) as fp:
        lines = fp.readlines()
    jsonList = [json.loads(f) for f in lines]
    #print "Read %d json lines" % len(jsonList)

    events = [j["event"] for j in jsonList]
    times  = [j["t"] for j in jsonList]    
    energies = [j["energy"] for j in jsonList]    

    uEvents = (list(set(events)))                                   # unique event names    
    eventsI = [uEvents.index(j["event"]) for j in jsonList]

    segStart = [times[i] - times[0] for i in range(len(times)-1)]
    segEnd   = [times[i+1] - times[0] for i in range(len(times)-1)]
    flags, classNames = aS.segs2flags(segStart, segEnd, events, 1)              
    return flags, classNames, energies

def plotLogFile(logPath):    
    hfmt = matplotlib.dates.DateFormatter('%H:%M:%S')
    gtLogPath = logPath + "_GT"
    gtLtLogPath = logPath + "_GTLT"
    if os.path.isfile(gtLogPath):
        flagsGT, classNamesGT = readGTFile(gtLogPath)
    if os.path.isfile(gtLtLogPath):
        flagsGTLT, classNamesGTLT = readGTFile(gtLtLogPath)

    splitLogPath = logPath.split("_")                                               # Get start datetime from filename parsing        
    startTime = datetime.datetime(int(splitLogPath[3]), int(splitLogPath[4]), int(splitLogPath[5]), int(splitLogPath[6]), int(splitLogPath[7]))
    flags, classNames,energies = readLogFile(logPath, startTime)                             # read log file
    classNames2 = sortEventNames(classNames)                                        # sort classnames by name
    flags2 = [classNames2.index(classNames[f]) for f in flags]
    classNames = classNames2
    flags = flags2

    fig = plt.figure()
    ax1 = fig.add_subplot(311)    
    ax1.set_yticks(numpy.array(range(len(classNames))))    
    ax1.set_yticklabels(classNames)
    ax1.xaxis.set_major_formatter(hfmt)
    timestamps = [startTime + datetime.timedelta(seconds=i*1) for i in range(len(flags))]            
    plt.plot(timestamps, numpy.array(flags))

    # plot ground-truth
    if os.path.isfile(gtLogPath):
        timestampsGT = [startTime + datetime.timedelta(seconds=i*1) for i in range(len(flagsGT))]            
        ax2 = fig.add_subplot(312)    
        ax2.set_yticks(numpy.array(range(len(classNamesGT))))
        ax2.set_yticklabels(classNamesGT)
        ax2.xaxis.set_major_formatter(hfmt)
        plt.plot(timestampsGT, flagsGT)    
            
        CM = numpy.zeros((len(classNamesGT), len(classNamesGT)))
        for i in range(len(flags)):
            if i<len(flagsGT):
                CM[flagsGT[i],flags[i]] += 1

        print "Confusion Matrix:"
        print "\t",
        for i in range(len(classNames)):
            print "{0:s}\t".format(classNames[i][0:2]),
        print
        for i in range(len(classNames)):
            print "{0:s}\t".format(classNames[i][0:2]),
            for j in range(len(classNames)):
                print "{0:.1f}\t".format(100.0 * CM[i][j] / CM.sum()),
            print
        print "Overall accuracy: {0:.1f}%%".format(100.0*numpy.diagonal(CM).sum()/CM.sum())
        [Rec, Pre, F1] = aS.computePreRec(CM, classNames)
        print "Class\tRec\tPre\tF1"
        for i in range(len(classNames)):
            print "{0:s}\t{1:.1f}\t{2:.1f}\t{3:.1f}".format(classNames[i][0:2], 100*Rec[i], 100*Pre[i], 100*F1[i])

    if os.path.isfile(gtLtLogPath):
        timestampsGTLT = [startTime + datetime.timedelta(seconds=i*1) for i in range(len(flagsGTLT))]            
        ax2 = fig.add_subplot(313)    
        ax2.set_yticks(numpy.array(range(len(classNamesGTLT))))
        ax2.set_yticklabels(classNamesGTLT)
        ax2.xaxis.set_major_formatter(hfmt)
        plt.plot(timestampsGTLT, flagsGTLT)    
    plt.show()

def highLevelFeatureFile(logPath):
    splitLogPath = logPath.split("_")
    startTime = datetime.datetime(int(splitLogPath[3]), int(splitLogPath[4]), int(splitLogPath[5]), int(splitLogPath[6]), int(splitLogPath[7]))
    flags, classNames,energies = readLogFile(logPath, startTime)    
    F = [0 for i in range(len(allClasses))]        
    diffSilence = 0
    for i in range(len(flags)-1):
        if classNames[flags[i]] == "silence" and classNames[flags[i+1]] != "silence":
            diffSilence += 1

    for i, c in enumerate(classNames):
        F[allClasses.index(c)] = numpy.count_nonzero(flags==i)
    F = F + [diffSilence]
    energies = numpy.array(energies)
    F = F + [numpy.mean(energies), numpy.max(energies), numpy.min(energies), numpy.std(energies)]
    F = numpy.array(F).astype("float")
    F = F / F.sum()

    return F

def highLevelFeatureDirs(listOfDirs):
    features = []
    classNames = []
    for d in listOfDirs:
        tempF = []        
        classNames.append(os.path.basename(os.path.normpath(d)))
        fileList = sorted(glob.glob(os.path.join(d, '*')))
        for f in fileList:
            F = highLevelFeatureFile(f)
            tempF.append(F)
        tempF = numpy.array(tempF)
        features.append(tempF)                
    #bestParam = aT.evaluateClassifier(features, classNames, 500, "knn", [3,5,7,9,11], 0, 0.95)
    bestParam = aT.evaluateClassifier(features, classNames, 500, "svm", [0.05, 0.25, 0.5, 0.75, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 7, 8, 9, 10], 0, 0.95)    
    #bestParam = aT.evaluateClassifier(features, classNames, 500, "randomforest", [10, 25, 50, 75], 0, 0.95)
    #bestParam = aT.evaluateClassifier(features, classNames, 500, "extratrees", [10, 25, 50, 75], 0, 0.95)
    #bestParam = aT.evaluateClassifier(features, classNames, 500, "gradientboosting", [10, 25, 50, 75, 150], 0, 0.95)        


if __name__ == '__main__':    
    if sys.argv[1] == "plotlogfile":
        plotLogFile(sys.argv[2])
    elif sys.argv[1] == "feature_file":
        print highLevelFeatureFile(sys.argv[2])
    elif sys.argv[1] == "feature_dirs":
        highLevelFeatureDirs(sys.argv[2::])

import sys
import numpy as n
import matplotlib.pyplot as plt

def main(argv):
    Features = n.load(argv[1])    
    print Features.shape
    print Features.mean(axis = 0)
    print n.nonzero(n.isnan(Features.mean(axis = 0)))

if __name__ == '__main__':
    main(sys.argv)

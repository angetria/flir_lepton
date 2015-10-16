#!/usr/bin/python
# coding:utf-8

__author__ = 'Aggelos Triantafyllidis', 'Konstantinos Panayiotou'
__maintainer__ = 'Aggelos Triantafyllidis', 'Konstantinos Panayiotou'
__email__ = 'aggelostriadafillidis@gmail.com', 'klpanagi@gmail.com'

import pylab as pl
import json
import numpy as np
import scipy as sp
from scipy.interpolate import interp1d
from scipy.interpolate import splrep
from scipy.interpolate import splev
import matplotlib.pyplot as plt

#class FlirUtils:
    #def __init__(self):


class Utils:
    def __init__(self):
        self.module = 'Interpolation Utilities'

    def normalize(self, dataset):
        normal_dataset = []
        max_value = max(dataset)
        min_value = min(dataset)

        print '\033[32mDataset max value:\033[0m [%s]' %max_value
        print '\033[32mDataset min value:\033[0m [%s]' %min_value
        for value in dataset:
            norm_value = 1.0 * (value - min_value) / (max_value - min_value)
            #print norm_value
            normal_dataset.append(norm_value)

        return {'dataset': normal_dataset, 'max': max_value, 'min': min_value}


def readDataset(fileUri):
    x_data = []
    y_data = []
    linecounter = 0

    with open(fileUri, "r") as file:
        for line in file:
            if (linecounter % 2) == 0:
                x_data.append(int(line.split('\n')[0]))
            else:
                y_data.append(int(line.split('\n')[0]))
            linecounter += 1
    return {'x_data': x_data, 'y_data': y_data}

#### ---- <Linear (1D) interpolation> ---- ####
def linear_interpolation(x_data, y_data):
    max_value_x = max(x_data)
    min_value_x = min(x_data)
    f1d = interp1d(x_data, y_data) # Create 1D interpolation function
    x_data_new = range(min_value_x, max_value_x+1) # Max/Min values from dataset
    y_data_new = f1d(x_data_new) # Calculate linear interpolated temperature values
    return {'x_data': x_data_new, 'y_data': y_data_new}

def cubicSpline_interpolation(x_data, y_data):
    max_value_x = max(x_data)
    min_value_x = min(x_data)
    x_data_new = range(min_value_x, max_value_x+1) # Max/Min values from dataset
    tck = splrep(x_data, y_data, s=0)
    y_data_new = splev(x_data_new, tck, der=0)
    return {'x_data': x_data_new, 'y_data': y_data_new}

def writeData(x_data, y_data, fileUri):
    f = open(fileUri, "w")
    if len(x_data) != len(y_data):
        print 'Something went wrong!! y_data and x_data must be the same size'
        sys.exit(0)
    for i in range(0, len(x_data)):
        f.write('%d\n' % x_data[i])
        f.write('%f\n' % y_data[i])


#### ---- <Load the dataset> ---- ####
dataset_uri = '/home/pandora/pandora_ws/srv/rpi_hardware_interface/data/flir_lepton/dataset05.pandora'
dataset = readDataset(dataset_uri)
signals = dataset['x_data']
temps = dataset['y_data']
######################################

##### ----- <Linear Interpolation 1D> ---- ######
linInterp_data = linear_interpolation(signals, temps)

plt.subplot(2,1,1)
plt.plot(signals, temps, 'o', linInterp_data['x_data'], linInterp_data['y_data'], '-')
plt.legend(['dataset', 'linear_interp'], loc='best')
plt.title('Linear (1D) interpolation')
###############################################

#### ---- <CUBIC-SPLINE Interpolation> ---- ####
splineInterp_data = cubicSpline_interpolation(signals, temps)
plt.subplot(2,1,2)
plt.plot(signals, temps, 'o', splineInterp_data['x_data'], splineInterp_data['y_data'], '-')
plt.legend(['dataset', 'Cubic Spline'], loc='best')
plt.title('Cubic-spline interpolation')


#writeData(splineInterp_data['x_data'], splineInterp_data['y_data'], \
        #'dataset_spline_interp.pandora')

"Uncomment the following line in order to plot the interplolated data"
plt.show()

#diffList = {}
#count = 0
#for val in interp_temps:
    #diffList[val] = interp_temps_spline[count]
    #print '\033[33mRaw Signal Value:\033[0m [%s]  ---> \033[31mLinear:\033[0m [%s],   \033[35mCubic-Spline:\033[0m [%s]'\
            #%(signals_new[count], val,interp_temps_spline[count])
    #count += 1


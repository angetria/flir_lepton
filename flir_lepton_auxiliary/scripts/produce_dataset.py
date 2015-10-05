#!usr/bin/python

import sys

# The area of interest(pixels) to find the mean temperature
x1 = 52
x2 = 54
y1 = 18
y2 = 20

# The area of neutral pixels to find the mean temperature
x3 = 66
x4 = 68
y3 = 6
y4 = 8

# Make the dataset text for 0,5 meters.
try:
    dataset = open("dataset05.pandora","w")
except IOError:
     print "dataset file could not be made"
    sys.exit()

# Process each image raw data for all images
for i in range(20,51):

    fileName = \
        "flir_calibration_pictures/depth_05/d_5_T"\
        + str(i) + ".bitch"

    try:
        data = open(fileName, "r")
    except IOError:
        print "No file was found"
        sys.exit()

    #counter = 0
    signal_val = []

    with  open(fileName, "r") as file:
        for line in file:
            #counter = counter + 1
            signal_val.append(line)
            temp = signal_val[0].split()


# To read specific pixel(x,y) must give : gimp gives inverted x,y coords
# average = the average temperature of the point of interest
# av = the average temperature of a neutral area

    average = 0
    av = 0
    if i < 51:
        for x in range(x1,x2):
            for y in range(y1,y2):
               # for raw data images
               a = y * 80 + x
               average = average + int(temp[a])

        # Find in the other array whats going on in pixel values
        for x in range(x3,x4):
            for y in range(y3,y4):
                b = y * 80 + x
                av = av + int(temp[b])

        average = average / 4
        av = av / 4

    # If we check second area
    #else:
        #for x in range(24,26):
            #temp = signal_val[x].split()
            #for y in range(42,44):
               #a = int(temp[y])
               #average = average + a

        #Find in the other array whats going on in pix values
        #for x in range(10,12):
            #temp = signal_val[x].split()
            #for y in range(65,67):
                #a = int(temp[y])
                #av = av + a

        #average = average / 4
        #av = av / 4

    # Make the file with our own standards, for even numbers of rows place
    # the raw pixel values, for odd the degree's.
    degree = str(i) + "\n"
    raw = str(average) + "\n"

    dataset.write(raw)
    dataset.write(degree)

    print "The average of image", i ,"=",average

    print "The av of image", i ,"=",av

data.close()
dataset.close()

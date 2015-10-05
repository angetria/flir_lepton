thermal_node_cfg
===============

This README file contains information about the configurations of the thermal node.

#Thermal detection method
There are two different detection methods in thermal_node. The first one processes a binary 
image(values are 0 or 255) extracted from the acquired temperatures of the sensor's image.
The second method processes the sensor_msgs/Image(values are 0 - 255) that is directly
acquired from the thermal sensor.

##Binary image processing method
Set detection method, in order to process binary image:
```bash
detection_method = 0
```
**This method is preferred for flir-lepton**.

###Further configurations
When detection method is 0 the most important configurations that must
be set are:

1.low_temperature: This variable is the lower bound(temperature) that we want
to be represented as white pixel.

2.high_temperature: This variable is the upper bound(temperature)that we want
to be represented as white pixel.

**Values outside of these bounds are represented as black**

**Important**:
low_temperature must be set to the value(temperature) of the victim(or a bit less)
or to a value(temperature) higher than the environment's temperature.

high_temperature must be set to the value(temperature) of the thermal source.


##Sensor Image processing method
Set detection method, in order to process the sensor's image:
```bash
detection_method = 1
```
**This method is not preferred**

##Other important configurations
1.min_area is a variable that checks the area of the candidate Poi that was found.
If area < min_area this Poi is rejected.

2.probability_method: The are two methods for the Pois probability extraction:

**Gaussian distribution**:
```bash
probability_method = 0
```
This method is based on two variables:
optimal_temperature: This variable defines the temperature that gives 100% probability.
tolerance: The tolerance of the gaussian distribution. The higher the value, the higher
the tolerance.

**Logistic distribution**:
```bash
probability_method = 1
```
This method is based on four variables:
low_acceptable_temperature: The low acceptable temperature of this method.
high_acceptable_temperature: The high acceptable temperature of this method.
left_tolerance: The tolerance of the logistic distribution's curve left side.
The higher the value, the lower the tolerance.
right_tolerance: The tolerance of the logistic distribution's curve right side. 
The higher the value, the lower the tolerance.

3.Dont forget to check the image matching parameters in thermal_node_maching_variables.yaml.

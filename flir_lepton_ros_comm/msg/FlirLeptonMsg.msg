Header header

# The thermal image acquired from flir_lepton sensor
sensor_msgs/Image thermalImage

# The raw pixel values from the sensor
std_msgs/UInt16MultiArray rawValues

# The temperatures in each pixel after calibration
std_msgs/Float32MultiArray temperatures

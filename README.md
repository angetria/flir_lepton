## Synopsis

Flir-Lepton LWIR sensor, ROS package.


 
## Packages

Integrated packages:
- flir_lepton_sensor:
- flir_lepton_image_processing:
- flir_lepton_msgs:
- flir_lepton_launchers:


## How to run nodes

This package consists of the following nodes:

- flir_lepton_sensor node: This node handles communication and data acquisition from Flir-Lepton sensor.
- flir_lepton_image_processing: This node performs image-processing, in order to extract higher level information regarding regions of interest from thermal image frames.



### Launch the standalone flir_lepton_sensor node

Simply execute:

```shell
$ roslaunch flir_lepton_sensor flir_lepton_sensor.launch
```

Or through the global launcher (Disable image processing node execution):

```shell
$ roslaunch flir_lepton_launchers flir_lepton.launch image_processing:=false
```

The node exposes two image message topics, a temperatures topic and a batch message topic (usefull if you want to do some **COOL** image processing stuff):
- /flir_lepton/image/rgb: RGB image (rainbow colorscheme).  ``` sensor_msgs/Image.```
- /flir_lepton/image/gray: GrayScale image. ``` sensor_msgs/Image.```
- /flir_lepton/temperatures: Scene frame temperatures values in Celcius degrees. [flir_lepton_msgs/TemperaturesMsg](https://github.com/angetria/flir_lepton/blob/master/flir_lepton_msgs/msg/flir_lepton_sensor/TemperaturesMsg.msg)
- /flir_lepton/batch: Constists of a sensor_msgs/Image (GrayScale values) plus temperatures, as above described.

By default, both rgb and grayscale image topics are published. You can disable any of them through the relevant launcher.
For example you can disable both rgb and gray publications by executing:

```shell
$ roslaunch flir_lepton_launchers flir_lepton.launch image_processing:=false gray_image:=false rgb_image:=false
```

We measured that maximum VoSPI frames/sec, for a stable communication, is around 25fps.
Though, global node publishing rate of the relevant topics is independed from VoSPI frames acquisition, as this implementation uses a thread that continuously reads through the SPI interface.
You can change node publishing rate through configuration file located under flir_lepton_sensor/config/ directory, [here](https://github.com/angetria/flir_lepton/blob/master/flir_lepton_sensor/config/params.yaml). 
Some other usefull configuration parameters are also defined into the aforementioned configuration file:

- SPI interface port.
- SPI clock speed.
- Publishing topic names.


### Launch flir_lepton_sensor with image processing capabilities

Like already mentioned, this package integrates some image processing functionalities. In order to launch with those image processing functionalities, simply execute:

```shell
$ roslaunch flir_lepton_launchers flir_lepton.launch
```

The image processing node is used by default.

## Tests

This metapackage does not include any tests. Node (package) specific tests can be found under the relevant package directory.

## Contributors

- Konstaninos Panayiotou, **[klpanagi@gmail.com]**
- Angelos Triantafyllidis, **[aggelostriadafillidis@gmail.com]**


## Contact 

Angelos Triantafyllidis, **[aggelostriadafillidis@gmail.com]**
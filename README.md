## Synopsis

Flir-Lepton LWIR sensor, ROS package.

As this is a ROS package, ROS has to be installed onto the system. For instructions regarding installation of ROS, follow [this link](http://wiki.ros.org/indigo/Installation)
A minimal installation of ROS is only required.

The package has been tested under [Hydro](http://wiki.ros.org/hydro/Installation) and [Indigo](http://wiki.ros.org/indigo/Installation) releases, both armhf and x86_64 architectures.

 
## Packages

Integrated packages:
- flir_lepton_sensor:
- flir_lepton_image_processing:
- flir_lepton_msgs:
- flir_lepton_launchers:

## Build procedure

As this is a ROS package, build procedures are done using the ROS framework.

First create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

```shell
$ mkdir -p ~/lepton_catkin_ws/src
```

Clone this repository under the src/ directory of the previously created catkin workspace directory.

```shell
$ cd ~/lepton_catkin_ws/src && git clone git@github.com:angetria/flir_lepton.git
```

Build sources.

```shell
$ cd ~/lepton_catkin_ws && catkin_make
```

Don't forget to source the setup.bash file.

```shell
$ source /opt/ros/<distro>/setup.bash
```

Using the short name of your ROS distribution instead of <distro>. Or you can either add this line to your .bashrc

No other external dependencies exists. Has been tested on **Debian** and **Ubuntu** distributions.


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

Launching with image processing capabilities is an extension. It does not overwrite the default behaviors (flir_lepton_sensor)

The image processing node is used by default.


###Launch the  standalone flir_lepton_image_processing node

In case of an independent use of this node, for exaple using a rosbag file,simply execute:

```shell
$ roslaunch flir_lepton_image_processing flir_lepton_image_processing.launch
```

By default rosbag mode is set to false. To be enabled simply execute:

```shell
$ roslaunch flir_lepton_image_processing flir_lepton_image_processing.launch  bag_mode:=true
```

**Dont forget to add the full path of your bag file in the launcher [here](https://github.com/angetria/flir_lepton/blob/master/flir_lepton_image_processing/launch/flir_lepton_image_processing.launch)**

## Tests

This metapackage does not include any tests. Node (package) specific tests are developed under each package directory.

## Contributors

- Konstaninos Panayiotou, [klpanagi@gmail.com]
- Angelos Triantafyllidis, [aggelostriadafillidis@gmail.com]


## Contact 

Angelos Triantafyllidis, [aggelostriadafillidis@gmail.com]

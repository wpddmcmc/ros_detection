# ros_detection

[![](https://img.shields.io/badge/TGeek-project-blue.svg)](47.93.7.151)

![rospass](<https://img.shields.io/badge/ROS%3A%3AKinetic-Pass-green>)

Objects detection ROS pakage based on OpenCV::dnn.

**Authot**: Michael.Chen

**Website**: www.tgeek.tech

[中文](./README_cn.md)

---

## Requirments

ROS::Kinect, OpenCV with OpenCV_contrib no less than 3.3.

## Files discription

#### build/ : build space

#### devel/ : Devel space

#### src/ : source space :

##### detector/ : detector package

######dnn_nets/ : The configration, labels and pre-trained model of networks

- yolo/ -Yolo Configurations
- ssd/  -SSD Configurations

###### include：head files

- DetectorNode.hpp
- dnndetector.hpp

###### msg/: massages for ROS

###### param/: configurations

- param_config.xml -configuration of video stream
- dnn_param.xml -configuration of dnn

###### src/ : source code

- listener/ -source of listener
- talker/ -source of the main functions

video/ : test video folder

###### package.xml: discription file of package

###### CMakeLists.txt : cmake configration

##### CMakeLists.txt : cmae configration of the package

## installation

### Configure Cmake

Configure```CMakeLists.txt ```

```bash
gedit CMakeLists.txt
```

if you have OpenCV version more than one, uncommit and change line9,set the build path which one you want to use.

```cmake
#if u have OpenCV version more than one, set the build path which one u want to use
set(OpenCV_DIR "YOUR_PATH")
```

Ex:

```cmake
#if u have OpenCV version more than one, set the build path which one u want to use
set(OpenCV_DIR "/home/test/app/opencv-3.4.0/build/")
```
### Compile

make sure you are in the **ROS catkin workspace**.

```bash
$ catkin_make
```

## Run

Open a terminal and run the ros master

```bash
$ roscore
```

Open a new terminal. **Set this workspace on top!**

```bash
$ source devel/setup.sh	#(option, if workspace not on top)
$ rosrun detector detector_node
```

![](http://tgeek.tech/wp-content/uploads/2019/08/Detection_screenshot_01.08.2019-e1564644184728.png)

### Option: Listener

If you want to hear the message from the detctor_node. Run the listener node

massages：

[time stemp]

[detected object type] 

[detected object confidence]

[detected object center positon]

Open a new terminal. **Set this workspace on top!**

```bash
$ rosrun detector listener
```

![](http://tgeek.tech/wp-content/uploads/2019/08/2019-08-01-22-50-04-的屏幕截图-e1564644199321.png)

## Usage

### Video stream configurations

**Do not need to re-compile**

videl_file location is {ROS_Package Path}/video/{VIdeo Name}

```xml
<?xml version="1.0"?>
<opencv_storage>
                                            <!--0-disable     1-enable-->
<show_time>1</show_time>                    <!--display time on output image-->
<debug_mode>1</debug_mode>                  <!--wait for keyboard to process next frame-->
<show_fps>0</show_fps>                      <!--display fps on output image-->

<use_camera>1</use_camera>                  <!--use camera or video-->
<video_file>test.mp4</video_file>           <!--video file name in {ProjectFolder}/video-->
<camera_index>1</camera_index>              <!--camera index-->
<window_width>1280</window_width>           <!--window size width-->
<window_height>720</window_height>          <!--window size height-->

</opencv_storage>
```

### Network configurations

**Do not need to re-compile**

network files location is {ROS_Package Path}/dnn/{File Name}

```xml
<?xml version="1.0"?>
<opencv_storage>

<!-->Configration<-->
<net_type>1</net_type>              <!-->0-ssd 1-yolo<-->
<thresh>0.35</thresh>               <!-->confidence threshold<-->
<nms_thresh>0.25</nms_thresh>       <!-->nms threshold<-->

<!-->Yolo configration files<-->
<Yolo_meanVal>1</Yolo_meanVal> 
<Yolo_scaleFactor>0.003921569</Yolo_scaleFactor>
<Yolo_config>/dnn_nets/yolo/yolov3-tiny.cfg</Yolo_config>
<Yolo_model>/dnn_nets/yolo/yolov3-tiny.weights</Yolo_model>
<coco_name>/dnn_nets/yolo/coco.names</coco_name>

<!-->ssd configration files<-->
<ssd_meanVal>127.5</ssd_meanVal> 
<ssd_scaleFactor>0.007843</ssd_scaleFactor>
<ssd_config>/dnn_nets/ssd/deploy.prototxt</ssd_config>
<ssd_model>/dnn_nets/ssd/mobilenet_iter_73000.caffemodel</ssd_model>
<ssd_name>/dnn_nets/ssd/ssd.names</ssd_name>
</opencv_storage>
```


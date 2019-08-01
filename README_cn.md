# ros_detection

[![](https://img.shields.io/badge/TGeek-project-blue.svg)](47.93.7.151)

![rospass](<https://img.shields.io/badge/ROS%3A%3AKinetic-Pass-green>)

Objects detection ROS pakage based on OpenCV::dnn.

**Authot**: Michael.Chen

**Website**: www.tgeek.tech
[English](./README.md)

---

## 要求

ROS::Kinect, OpenCV with OpenCV_contrib no less than 3.3.

## 文件描述

#### build/ : 编译空间

#### devel/ : 开发空间

#### src/ : 源码空间 :

##### detector/ : detector ROS包

######dnn_nets/ : 网络结构，标签，预训练模型

- yolo/ -Yolo 配置文件
- ssd/  -SSD 配置文件

###### include：头文件

- DetectorNode.hpp
- dnndetector.hpp

###### msg/: mesages for ROS

###### param/: configurations

- param_config.xml -视频流配置
- dnn_param.xml -神经网络配置

###### src/ : 源码

- listener/ -订阅器
- talker/ -发布器

video/ : 测试视频文件夹

###### package.xml: 包描述文件

###### CMakeLists.txt : cmake 配置文件

##### CMakeLists.txt : 包 cmake 配置文件

## 安装

### 配置 Cmake

Configure```CMakeLists.txt ```

```bash
gedit CMakeLists.txt
```

多版本OpenCV设置路径，否则注释此行

```cmake
#if u have OpenCV version more than one, set the build path which one u want to use
set(OpenCV_DIR "YOUR_PATH")
```

Ex:

```cmake
#if u have OpenCV version more than one, set the build path which one u want to use
set(OpenCV_DIR "/home/test/app/opencv-3.4.0/build/")
```
### 编译

**确保在工作空间**

```bash
$ catkin_make
```

## 运行

打开一个ros master

```bash
$ roscore
```

打开新终端，**确保当然工作空间在顶部**

```bash
$ source devel/setup.sh	#(option, if workspace not on top)
$ rosrun detector detector_node
```

![](http://tgeek.tech/wp-content/uploads/2019/08/Detection_screenshot_01.08.2019-e1564644184728.png)

### 可选: Listener 订阅器

如果想监听消息，可以打开一个订阅器

消息为：

[时间戳]

[检测到物体类别] 

[检测到物体置信度]

[检测到物体中心点坐标]

打开新终端，**确保当然工作空间在顶部**

```bash
$ rosrun detector listener
```

![](http://tgeek.tech/wp-content/uploads/2019/08/2019-08-01-22-50-04-的屏幕截图-e1564644199321.png)

## 使用

### 视频流配置

**不需要重新编译**

视频路径为 {ROS_Package Path}/video/{VIdeo Name}

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

**不需要重新编译**

网络文件为 {ROS_Package Path}/dnn/{File Name}

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

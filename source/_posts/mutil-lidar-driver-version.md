---
title: 多激光雷达驱动程序
date: 2018-07-11 10:12:38
categories: 
- 传感器
---

**本文介绍一台计算机连接多个Velodyne激光雷达的教程。**

# 驱动程序下载

ROS系统中的Velodyne激光雷达驱动可以从[ROS-velodyne](https://github.com/ros-drivers/velodyne)获取：

```
sudo apt-get install ros-(your version)-velodyne
cd PATH/TO/CATKIN/WORKSPACE/SRC
git clone  https://github.com/ros-drivers/velodyne.git
cd ..
catkin_make
```

# 处理激光雷达标定文件

一般来讲，激光雷达在出厂包装时会附带含有标定文件、驱动程序、User Manual等文件的U盘，从中可以获得.xml的标定文件。而launch文件中需要的是.yaml格式的文件，可以用自带脚本转换：

```
cd VELODYNEPATH/velodyne_pointcloud/scripts
python gen_calibration.py PATH/TO/XML/FILE
```

将脚本目录的.yaml文件保存到`velodyne_pointcloud/params/*.yaml`路径下。

# 修改launch启动文件

## 单个激光雷达启动

**setup 1.** 设置电脑IP，地址：192.168.1.200，子网掩码255.255.255.0，网关192.168.1.1。

**setup 2.** 连接激光雷达到电脑，浏览器输入192.168.1.201（默认的激光雷达IP地址），出现激光雷达的设置界面，确认IP和端口是否正常。

**setup 3.** 以下launch文件，将calibration的标定文件名修改为上一步生成的标定文件名，按需修改device_ip和frame_id。

```
<!-- -*- mode: XML -*- -->
<!-- run velodyne_pointcloud/CloudNodelet in a nodelet manager for an VLP-32C -->

<launch>

  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP-32c-no1.yaml"/>
  <arg name="device_ip" default="" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2368" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <arg name="laserscan_ring" default="-1" />
  <arg name="laserscan_resolution" default="0.007" />

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="32C"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include>

</launch>
```


## 2个或多个激光雷达驱动

**setup 1.** 设置电脑IP，地址：192.168.1.200，子网掩码255.255.255.0，网关192.168.1.1。

**setup 2.** 依次连接激光雷达到电脑，浏览器输入192.168.1.201（默认的激光雷达IP地址），出现激光雷达的设置界面，修改IP与端口，并记录清楚。

**setup 3.** 以下launch文件，每个ns代表一个激光雷达，其中需要修改calibration文件、device_ip、frame_id、port，对应激光雷达区分清楚。

```
<!-- -*- mode: XML -*- -->
<!-- run velodyne_pointcloud/CloudNodelet in a nodelet manager for an VLP-32C -->

<launch>


<!-- For 1st velodyne -->
<group ns="ns1">

  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP32C-NO1.yaml"/>
  <arg name="device_ip" default="192.168.1.201" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2368" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <!-- arg name="laserscan_ring" default="-1" /-->
  <!-- arg name="laserscan_resolution" default="0.007" /-->

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="32C"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet 
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include> -->

</group>



<!-- For 2nd velodyne -->
<group ns="ns2">

  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP32C-NO2.yaml"/>
  <arg name="device_ip" default="192.168.1.202" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2369" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <!-- arg name="laserscan_ring" default="-1" /-->
  <!-- arg name="laserscan_resolution" default="0.007" /-->

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="32C"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet 
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include> -->

</group>



<!-- For 3rd velodyne -->
<group ns="ns3">

  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP32C-NO3.yaml"/>
  <arg name="device_ip" default="192.168.1.203" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2370" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <!-- arg name="laserscan_ring" default="-1" /-->
  <!-- arg name="laserscan_resolution" default="0.007" /-->

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="32C"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet 
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include> -->

</group>



<!-- For 4th velodyne -->
<group ns="ns4">

  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP32C-NO4.yaml"/>
  <arg name="device_ip" default="192.168.1.204" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2371" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <!-- arg name="laserscan_ring" default="-1" /-->
  <!-- arg name="laserscan_resolution" default="0.007" /-->

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="32C"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet 
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include> -->

</group>



</launch>
```
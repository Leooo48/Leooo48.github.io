---
title: rosbag中各指令的使用
date: 2018-11-28 10:43:39
categories: 
- 实用教程
---

以下是rosbag常用的操作指令：
* record：录制特定topic的rosbag包
* info：bag包的信息
* play：回放一个或多个bag包
* check：核对某bag包是否可播放
* fix：修复bag包以在当前系统播放
* filter：转换包
* compress：压缩包
* decompress：接压缩包

# rosbag record

> **record &lt;topic-names&gt;**
> \$ rosbag record rosout tf cmd_vel
> * -a, --all : Record all topics.
> * -e, --regex : Match topics using regular expressions.
> * -d, --duration : Specify the maximum duration of the recorded bag file.
> \$ rosbag record -d=30 /chatter
> \$ rosbag record -d=5m /chatter
> \$ rosbag record --duration=2h /chatter
> * -o PREFIX, --output-prefix=PREFIX : Prepend PREFIX to beginning of bag name before date stamp.
> \$ rosbag record -o session1 /chatter
> * -O NAME, --output-name=NAME : Record to bag with name NAME.bag.
> \$ rosbag record -O session2_090210.bag /chatter
> * --node=NODE : Record all topics subscribed to by a specific node
> \$ rosbag record --node=/joy_teleop

# rosbag info

> **info &lt;bag-files&gt;**
> * -y, --yaml : Print information in YAML format.
> \$ rosbag info -y /path/to/my.bag

# rosbag play

> **play &lt;bag-files&gt;**

> * -q, --quiet : Suppress console output.
> \$ rosbag play -q recorded1.bag
> * -i, --immediate : Play back all messages without waiting.
> \$ rosbag play -i recorded1.bag
> * --pause : Start in paused mode.
> \$ rosbag play --pause recorded1.bag
> * --queue=SIZE : Use an outgoing queue of size SIZE (defaults to 0. As of 1.3.3 defaults to 100).
> \$ rosbag play --queue=1000 recorded1.bag
> * --clock : Publish the clock time
> \$ rosbag play --clock recorded1.bag
> * --hz=HZ : Publish clock time at frequency HZ Hz (default: 100).
> \$ rosbag play --clock --hz=200 recorded1.bag
> * -d SEC, --delay=SEC : Sleep SEC seconds after every advertise call (to allow subscribers to connect).
> \$ rosbag play -d 5 recorded1.bag
> * -r FACTOR, --rate=FACTOR : Multiply the publish rate by FACTOR.
> \$ rosbag play -r 10 recorded1.bag
> * -s SEC, --start=SEC : Start SEC seconds into the bags.
> \$ rosbag play -s 5 recorded1.bag
> * -u SEC, --duration=SEC : Play only SEC seconds from the bag files.
> \$ rosbag play -u 240 recorded1.bag
> * -l, --loop : Loop playback.
> \$ rosbag play -l recorded1.bag
> * -k, --keep-alive : Keep alive past end of bag (useful for publishing latched topics).
> \$ rosbag play -k recorded1.bag

# rosbag check

> **check &lt;bag-file&gt;**
> \$ rosbag check old.bag

# rosbag filter

> **filter &lt;in-bag&gt; &lt;out-bag&gt; &lt;expression&gt;**
> \$ rosbag filter my.bag only-tf.bag "topic == '/tf'"
> * topic : the topic of the message
> * m : the message
> * t : time of message. The time is represented as a rospy Time object (t.secs, t.nsecs)
> * To filter based on time, convert the time to a floating point number (use UNIX time, to get this value, use rosbag info):
>   rosbag filter input.bag output.bag "t.to_sec() <= 1284703931.86"
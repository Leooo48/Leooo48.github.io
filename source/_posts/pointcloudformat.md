---
title: ROS与PCL中点云格式（Pointcloud）详解
date: 2018-08-03 16:56:07
categories: 
- 传感器
---

# 1. ROS中点云消息格式(sensor_msg::PointCloud2)

```
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
```

* `std_msgs/Header header`由以下组成：
```
uint32 seq
time stamp
string frame_id
```

* `height`与`width`是指点云数据的高和宽，一般无序点云的高为1，宽为点云中激光点的个数；结构化点云的高和宽均大于1。

* `sensor_msgs/PointField[] fields`由以下组成，其中`name`是指点云包含的域的名称，如“x”、“y”、“z”、“intensity”、“ring”等；`offset`是指在`data`的每个数据中该域的偏移量；`datatype`是指以上1\~8的数据类型；`count`是指该域有多少个元素，一般为1。每一帧点云均有该项数据。
```
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count
```

* `is_bigendian`数据是大端存储还是小端存储的标志。

* `pointstep`表示每个点的字节长度，常见的为32。

* `rowstep`表示每行的字节长度。

* `data`表示所有的点的数据，以字节存储。`uint8[] data`代表`vector`类型，有size、push_back、clear等操作。

* `is_dense`若是`true`，代表点云数据中不包含无效点（nan点）；若是`false`，代表点云中包含无效点。

# 2. PCL中点云消息格式(pcl::PointCloud)

```
pcl::PCLHeader header
std::vector<PointT, Eigen::aligned_allocator<PointT> > points
uint32_t width;
uint32_t height;
bool is_dense;
Eigen::Vector4f sensor_origin_;
Eigen::Quaternionf sensor_orientation_;
```

* `pcl::PCLHeader header`是pcl格式的header，包括：
```
pcl::uint32_t seq;
pcl::uint64_t stamp;
std::string frame_id;
```

* `points`是pcl点云中所有点的vector。

* `height`与`width`是指点云数据的高和宽。

* `is_dense`若是`true`，代表点云数据中不包含无效点（nan点）；若是`false`，代表点云中包含无效点。

* pcl点云可以使用的函数参考(pcl::PointCloud)[http://docs.pointclouds.org/trunk/common_2include_2pcl_2point__cloud_8h_source.html]。常用的包括size、clear、insert、resize、

# 3. 相互转换
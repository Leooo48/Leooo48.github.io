---
title: kd-tree理论及应用
date: 2018-12-04 14:52:45
categories: 
- 算法
---

通过激光雷达获取的点云数据，具有数据量大、分布不均匀等特点。作为三维领域中一个重要的数据来源，点云主要是表征目标表面的海量点的集合，并不具备传统网格数据的几何拓扑信息，所以点云数据处理中最为核心的问题就是建立离散点间的**拓扑关系**，实现基于邻域关系的**快速查找**。

kd-tree（k-dimensional tree），是一种分割k维数据空间的数据结构，主要应用于多维空间关键数据的搜索（如：范围搜索和最近邻搜索）。kd-tree是二进制空间分割树的特殊的情况，用来组织表示k维空间中点的拓扑关系，是一种带有其他约束的二分查找树。

kd-tree算法可以分为两大部分：一部分是有关kd-tree本身这种数据结构建立的算法，另一部分是在建立的kd-tree上如何进行最邻近查找的算法。

# 构建算法

kd-tree是一个二叉树，每个节点表示一个空间范围：
> * Node-data：数据矢量，数据集中具体的某个数据点，是k维矢量
> * Range：空间矢量，该节点所代表的空间范围
> * split：整数，垂直于分割超平面的方向轴序号
> * Left：kd-tree，由位于该节点分割超平面左子空间内所有数据点所构成的kd-tree
> * Right：kd-tree，由位于该节点分割超平面右子空间内所有数据点所构成的kd-tree
> * parent：kd-tree，父节点

先以一个简单直观的实例来介绍kd-tree算法。假设有6个二维数据点{（2,3），（5,4），（9,6），（4,7），（8,1），（7,2）}，数据点位于二维空间内。kd-tree算法就是要确定图1中这些分割空间的分割线（多维空间即为分割平面，一般为超平面）。下面就要通过一步步展示kd-tree是如何确定这些分割线的。

![](https://raw.githubusercontent.com/Leooo48/markdownimages/master/p1.png)

由于此例简单，数据维度只有2维，所以可以简单地给x，y两个方向轴编号为0,1，也即split={0,1}。

（1）确定split域的首先该取的值。分别计算x，y方向上数据的方差得知x方向上的方差最大，所以split域值首先取0，也就是x轴方向；

（2）确定Node-data的域值。根据x轴方向的值2,5,9,4,8,7排序选出中值为7，所以Node-data = （7,2）。这样，该节点的分割超平面就是通过（7,2）并垂直于split = 0（x轴）的直线x = 7；

![](https://raw.githubusercontent.com/Leooo48/markdownimages/master/p2.png)

（3）确定左子空间和右子空间。分割超平面x = 7将整个空间分为两部分，如图2所示。x < = 7的部分为左子空间，包含3个节点{（2,3），（5,4），（4,7）}；另一部分为右子空间，包含2个节点{（9,6），（8,1）}。

（4）kd-tree的构建是一个递归的过程。然后对左子空间和右子空间内的数据重复根节点的过程就可以得到下一级子节点（5,4）和（9,6）（也就是左右子空间的根节点），同时将空间和数据集进一步细分。如此反复直到空间中只包含一个数据点，最后生成的kd-tree如图3所示。

![](https://raw.githubusercontent.com/Leooo48/markdownimages/master/p3.png)

# 搜索算法

在kd-tree中进行数据的查找也是特征匹配的重要环节，其目的是检索在kd-tree中与查询点距离最近的数据点。这里先以一个简单的实例来描述最邻近查找的基本思路。

![](https://raw.githubusercontent.com/Leooo48/markdownimages/master/p4.png)

星号表示要查询的点（2.1,3.1）。通过二叉搜索，顺着搜索路径很快就能找到最邻近的近似点，也就是叶子节点（2,3）。而找到的叶子节点并不一定就是最邻近的，最邻近肯定距离查询点更近。为了找到真正的最近邻，还需要进行回溯操作：算法沿搜索路径反向查找是否有距离查询点更近的数据点。

此例中先从（7,2）点开始进行二叉查找，然后到达（5,4），最后到达（2,3），此时搜索路径中的节点为<（7,2），（5,4），（2,3）>，首先以（2,3）作为当前最近邻点，计算其到查询点（2.1,3.1）的距离为0.1414，然后回溯到其父节点（5,4），并判断在该父节点的其他子节点空间中是否有距离查询点更近的数据点。以（2.1,3.1）为圆心，以0.1414为半径画圆，如图4所示。发现该圆并不和超平面y=4交割，因此不用进入（5,4）节点右子空间中去搜索。

# 应用

PCL中kd_tree模块类KdTree关键成员函数:

```
virtual void pcl::KdTree<PointT>::setInputCloud(const PointCloudConstPtr & cloud, const IndicesConstPtr & indices = IndicesConstPtr ())	 
```
作用：设置输入点云，参数cloud作为输入点云的共享指针引用，indices为在kd_tree中使用的点对应的索引，如果不设置，则默认使用整个点云填充kd_tree。 


```
virtual int pcl::KdTree<PointT>::nearestKSearch(int index, int k, std::vector<int>& k_indices, std::vector<float> & k_sqr_distances)		
```
作用：纯虚函数，具体实现在其子类KdTreeFLANN中，其用来进行k邻域搜索，k_sqr_distances为搜索完成后每个邻域点与查询点的欧式距离。

应用实例：
```
#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  srand (time (NULL));   //用系统时间初始化随机种子
  //创建一个PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // 随机点云生成
  cloud->width = 1000;             //此处点云数量
  cloud->height = 1;                //表示点云为无序点云
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)   //循环填充点云数据
  {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
 //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
 //设置搜索空间
  kdtree.setInputCloud (cloud);
  //设置查询点并赋随机值
  pcl::PointXYZ searchPoint;
  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // K 临近搜索
  //创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
  std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
  //打印相关信息
  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )  //执行K近邻搜索
  {
     //打印所有近邻坐标
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }
  /**********************************************************************************
   下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
   pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
   ********************************************************************************/
  // 半径 R内近邻搜索方法

  std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
  std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);   //随机的生成某一半径
  //打印输出
  std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )  //执行半径R内近邻搜索方法
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}
```

结果：
![](https://raw.githubusercontent.com/Leooo48/markdownimages/master/p5.png)
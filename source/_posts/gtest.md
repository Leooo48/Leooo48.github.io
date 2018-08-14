---
title: gtest在ROS中的使用
date: 2018-08-14 14:05:27
categories: 
- 实用教程
---

gtest是对程序进行单元测试的有力工具，能帮助程序员规范化编程，消灭BUG！

# 安装gtest

```
sudo apt-get install libgtest-dev
```

# 编写test node

在待测试的package目录下创建一个子目录，命名为test，所有的测试代码在test文件夹下添加。

新建test.cpp：

```
// ROS头文件
#include <ros/ros.h>
// 待测函数单元的头文件
#include "foo/foo.h"
// gtest头文件
#include <gtest/gtest.h>

// 声明第一个测试例
// TestSuite是测试对象的名称，这个名称是任意的，但是最好具有针对性
// testCase1是测试用例的名称，这个是唯一的，最好说明测试用例的用途
TEST(TestSuite, testCase1) {
  <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
  ASSERT_EQ(a, foo::func(b, c)); // 仅做示范，作用：测试函数foo::func返回结果与a是否相等
}

// 声明第二个测试例
TEST(TestSuite, testCase2) {
  <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

int main(int argc, char **argv) {
  // 程序运行之后，可以在终端显示测试结果，也会以.xml文件的格式存储到/home/jd/文件夹，且.xml文件的名字默认是测试程序的节点的名字，多次运行测试程序，不会覆盖之前的测试文件。
  testing::GTEST_FLAG(output) = "xml:/home/jd/";
  // 初始化测试器
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  // 开始执行所有的测试，如果全部通过则返回0，否则返回1
  // 此步是必须的，否则不会执行测试
  return RUN_ALL_TESTS();
}
```

不带注释版code：

```
#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestSuite, testCase1) {
  ASSERT_*;
}

int main(int argc, char **argv) {
  testing::GTEST_FLAG(output) = "xml:/home/jd/";
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
```

# 编写CMakeLists.txt

修改CMakeLists.txt，在合适的地方加入：

```
find_package(GTest REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

catkin_add_gtest(test_node test/test.cpp src/foo.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
```

# 编译运行test node

用`catkin_make`编译，`rosrun`开始运行测试节点。

还可以把运行写入launch文件中

```
<launch>
  <node pkg="mypkg" type="mynode" name="mynode" />
  <test test-name="test_node" pkg="mypkg" type="test_node" />
</launch>
```

# gtest断言

gtest中，断言的宏可以分为两类，一类是ASSERT系列，一类是EXPECT系列。一个直观的解释就是：

1. ASSERT_\* 系列的断言，当检查点失败时，退出当前函数（**注意：并非退出当前案例**）
2. EXPECT_\* 系列的断言，当检查点失败时，继续往下执行

| Test	| ASSERT_*	| EXPECT_* |
| --------   | :-----:   | :----: |
| True	| ASSERT_TRUE(condition)	| EXPECT_TRUE(condition) |
| False	| ASSERT_FALSE(condition)	| EXPECT_FALSE(condition) |
| Equal	| ASSERT_EQ(arg1,arg2)	| EXPECT_EQ(arg1,arg2) |
| Not Equal	| ASSERT_NE(arg1,arg2)	| EXPECT_NE(arg1,arg2) |
| Less Than	| ASSERT_LT(arg1,arg2)	| EXPECT_LT(arg1,arg2) |
| Less Than or Equal	| ASSERT_LE(arg1,arg2)	| EXPECT_LE(arg1,arg2) |
| Greater Than	| ASSERT_GT(arg1,arg2)	| EXPECT_GT(arg1,arg2) |
| Greater Than or Equal	| ASSERT_GE(arg1,arg2)	| EXPECT_GE(arg1,arg2) |
| C String Equal	| ASSERT_STREQ(str1,str2)	| EXPECT_STREQ(str1,str2) |
| C String Not Equal	| ASSERT_STRNE(str1,str2)	| EXPECT_STRNE(str1,str2) |
| C String Case Equal	| ASSERT_STRCASEEQ(str1,str2)	| EXPECT_STRCASEEQ(str1,str2) |
| C String Case Not Equal	| ASSERT_STRCASENE(str1,str2)	| EXPECT_STRCASENE(str1,str2) |

以上是gtest在ROS中的基础应用，有想详细了解gtest的移步博客[玩转Google开源C++单元测试框架Google Test系列(gtest)](http://www.cnblogs.com/coderzh/archive/2009/03/31/1426758.html)
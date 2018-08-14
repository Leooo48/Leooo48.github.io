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
// gtest头文件
#include <gtest/gtest.h>
// 待测函数单元的头文件
#include "foo/foo.h"

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
  // 程序运行之后，可以在终端显示测试结果，也会以.xml文件的格式存储到指定路径文件夹，
  // 且.xml文件的名字默认是测试程序的节点的名字，多次运行测试程序，不会覆盖之前的测试文件。
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

不带注释版code，可以直接复制，然后编写测试用例：

```
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "foo/foo.h"

TEST(TestSuite, testCase1) {
  // ASSERT_*;
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
add_executable(test_node test/test.cpp src/foo.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} gtest)
```

# 编译运行test node

用`catkin_make`编译，`rosrun`运行测试节点。

# gtest断言

gtest中，断言的宏可以分为两类，一类是ASSERT系列，一类是EXPECT系列。一个直观的解释就是：

1. ASSERT_\* 系列的断言，在失败时，会立即退出当前的测试用例（即其所在的函数，但不会结束整个测试）
2. EXPECT_\* 系列的断言，在失败时，会继续执行，不会退出当前测试用例

在每一个大类中，又分为多个小类别，它们分别用于不同目的的测试，如布尔测试、数值测试、字符串测试等等。

* 布尔测试
布尔测试用于测试给定的值为真还是假，它们包括：
\*\_TRUE(condition)：期望condition为true，若condition为false，则断言失败；
\*\_FALSE(condition)：期望condition为false，若condition为true，则断言失败；
其中\*为ASSERT或者EXPECT，后续所有的宏都将采用这种形式。

* 数值比较测试    
数值比较测试即比较两个数值之间的大小关系，它们包括：
\*\_EQ(expected, actual)：expected == actual则成功，否则失败；
\*\_NE(expected, actual)：expedted !=  actual则成功，否则失败；
\*\_LT(val1, val2)：val1 < val2则成功，否则失败；
\*\_LE(val1, val2)：val1 <= val2 则成功，否则失败；
\*\_GT(val1, val2)：val1 > val2则成功，否则失败；
\*\_GE(val1,val2)： val1 >= val2则成功，否则失败；
    
* 浮点数类型比较
gtest针对浮点数是否相等专门定义了宏，它们包括：
\*\_FLOAT_EQ(expected, actual)：expected与actual相差很小时成功，否则失败；
\*\_DOUBLE_EQ(expected, actual)：expected与actual相差很小时成功，否则失败；
\*\_NEAR(val1, val2, abs)：|val1 - val2| <= abs时成功，否则失败；

* 字符串类型比较
对于字符串，gtest提供字符串相等及不等断言，但它们都只支持C类型的字符串，不支持C++中的std::string和std::wstring,它们包括：
\*\_STREQ(expected, actual)：同时支持char\*和wchar_t\*，expected和actual的字符串内容相同则成功，否则失败；
\*\_STRNE(str1, str2)：同时支持char\*和wchar_t\*，str1和str2字符串内容不同则成功，否则失败；
\*\_STRCASEEQ(expected, actual)：只支持char\*， expected和actual的字符串内容相同则成功，否则失败；
\*\_STRCASENE(str1, str2)：只支持char\*，str1和str2字符串内容不同则成功，否则失败；

* 执行成功与失败标记
在gtest中，测试通过与否有三种状态，它们对应于一个枚举：
```
enum Type {
    kSuccess,          // Succeeded.
    kNonFatalFailure,  // Failed but the test can continue.
    kFatalFailure      // Failed and the test should be terminated.
};
```
    每一个枚举值都对应一个宏，通过这个宏我们可以返回相应的执行状态：
    kSuccess：成功，对应的宏为SUCCEED()；
    kNonFatalFailure：虽然失败，但当前测试用例的后续测试仍然继续运行，对应ADD_FAIL();
    kFatalFailure：致命错误，当前测试用例后续测试不会执行，对应FAIL();

* 异常检查
gtest中提供检查代码是否抛出异常的方法，它们包括：
\*\_THROW(statement, exception_type)：statement如果抛出exception_type类型异常则成功，否则失败；
\*\_ANY_THROW(statement)：只要statement抛出任何异常则成功，否则失败；
\*\_NO_THROW(statement)：只要statement抛出任何异常则失败，否则成功；



> 以上是gtest在ROS中的基础应用，有想详细了解gtest的移步博客：[玩转Google开源C++单元测试框架Google Test系列(gtest)](http://www.cnblogs.com/coderzh/archive/2009/03/31/1426758.html)。
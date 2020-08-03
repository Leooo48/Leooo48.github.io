---
title: Could not find compiler set in environment variable CXX
date: 2020-08-03 11:00:00
categories:
- C++
---





在Ubuntu系统中，运行`cmake ..`时，会报出

```
Could not find compiler set in environment variable CXX:

  clang++.

Call Stack (most recent call first):
  CMakeLists.txt:3 (enable_language)


CMake Error: CMAKE_CXX_COMPILER not set, after EnableLanguage
-- Configuring incomplete, errors occurred!
```

这个问题的根源是环境变量中，默认对C/C++文件的编译器设置为了clang，可以输入`env`确认。

修复方法：

```
sudo apt install gcc g++
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
```


---
title: Ubuntu 16.04中Sublime配置ClangFormat
date: 2019-01-24 10:13:04
categories: 
- Ubuntu
---

Clangformat可以很方便的调整代码风格，规范代码格式。在Sublime中可以配置Clangformat插件，一键规范化代码。

#### Step 1

- Sublime中，手动安装[Package Control](https://packagecontrol.io/installation)
- Ctrl+Shift+P打开搜索框，输入pcip，点击Package Control: Install Package
- 输入clang，选择clangformat
- 等待安装完毕

#### Step 2

- 解压[clangformat文档中的bin文件](https://pan.baidu.com/s/1w0zNAcJw0F2DXcCFnWisGg)到/usr/local/bin/
- Sublime中，Ctrl+Shift+P打开搜索框，输入clang，选择Clang Format: Set Path
- 输入/usr/local/bin/clang-format
- Ctrl+Shift+P打开搜索框，输入clang，选择Clang Format: Select Style
- 选择Google

#### Step 3

- 快捷键：Ctrl+Alt+A 一键规范代码

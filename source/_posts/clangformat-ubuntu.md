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
> - 若打开Package Control: Install Package失败，则用浏览器访问[这个网址](https://packagecontrol.io/channel_v3.json)，复制内容并保存为channel\_3.json的文件。将channel\_3.json复制到sublime文件夹下任意路径
> - 打开sublime -> Preferences -> Package Settings -> Package Control -> Settings-user，粘贴，就可以打开Package Control: Install Package
```
"channels":
[
  "PATH/TO/channel_v3.json"
],
```
- 输入clang，选择clangformat
- 等待安装完毕

如果sublime无法安装插件，则手动安装clang-format：

- 从[官方github](https://github.com/rosshemsley/SublimeClangFormat)上下载zip格式文件，解压
- 将该文件夹放在`.config/sublime-text-3/Packages/`目录下
- 重启sublime，加载成功


#### Step 2

- 解压[clangformat文档中的bin文件](https://pan.baidu.com/s/1w0zNAcJw0F2DXcCFnWisGg)到`/usr/local/bin/`
- Sublime中，Ctrl+Shift+P打开搜索框，输入clang，选择Clang Format: Set Path
- 输入`/usr/local/bin/clang-format`
- Ctrl+Shift+P打开搜索框，输入clang，选择Clang Format: Select Style
- 选择Google

#### Step 3

- 快捷键：Ctrl+Alt+A 一键规范代码

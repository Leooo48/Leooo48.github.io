---
title: Ubuntu 16.04 + NVIDIA + cuda安装配置教程
date: 2018-07-11 10:08:56
categories: 
- 实用教程
---

NVIDIA驱动的安装有两种方式，一种直接从“附加启动”里面自动下载安装，另一种是从官网下载对应显卡与系统的.sh驱动文件手动安装。这里仅介绍第二种的具体操作指令。

# 1. 全新安装的ubuntu系统安装部分依赖项
```
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev
sudo apt-get install libopencv-dev libhdf5-serial-dev protobuf-compiler
sudo apt-get install --no-install-recommends libboost-all-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install python-dev
sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
sudo apt-get install g++
sudo apt-get install git
sudo apt-get install freeglut3-dev
```

# 2. 安装CUDA

## 2.1 禁用nouveau驱动

* 新建blacklist-nouveau.conf文件
```
sudo gedit /etc/modprobe.d/blacklist-nouveau.conf
```

* 在文件中输入并保存
```
blacklist nouveau
options nouveau modeset=0
```

* 重新生成kernel initramfs
```
sudo update-initramfs –u
```

* 重启系统，如果以下指令无输出则禁用生效
```
lsmod | grep nouveau
```

## 2.2 NVIDIA官网下载对应系统、对应版本的CUDA库.sh文件

[下载地址](https://developer.nvidia.com/cuda-downloads)

## 2.3 安装CUDA

* 按下Ctrl+Alt+F1进入tty1界面，输入用户名和密码。笔记本的Fx键可能会被Fn锁住，如果按下Ctrl+Alt+F1没有反应，可以尝试Ctrl+Alt+Fn+F1。
```
chmod +x cuda_x.x.x_linux.run
sudo service lightdm stop
sudo sh cuda_x.x.x_linux.run
```

除了在“Install NVIDIA Accelerate Graphic driver”时选择“no”，其余的一路“yes”即可。

* 重新启动桌面
```
sudo service lightdm start
```

## 2.4 配置环境变量
```
export PATH=/usr/local/cuda-7.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-7.0/lib64:$LD_LIBRARY_PATH
sudo ldconfig /usr/local/cuda/lib64
```

# 3. 安装NVIDIA驱动

## 3.1 Ctrl+Alt+F1进入tty模式

## 3.2 安装驱动
```
sudo chmod +x NVIDIA-*.sh
sudo service lightdm stop
sudo sh NVIDIA-*.sh
```

一路“Accept”即可。

# 3.3 重新启动桌面
```
sudo service lightdm start
```

# 3.4 验证是否安装成功

* 系统右上角-About This Computer，如果显示了显卡名称则安装成功
* 在Terminal中输入nvidia-smi，如果显示了显卡使用情况则安装成功
* 输入以下指令，如果如果显示“PASS”并有显卡名称，则CUDA与显卡驱动均安装成功
```
cd NVIDIA_CUDA-x.x_Samples
make –j8
./bin/deviceQuiry
```
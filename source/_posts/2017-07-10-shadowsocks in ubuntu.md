---
title: shadowsocks科学上网教程
date: 2017-07-11 12:00:00
categories: 
- 实用教程
---

shadowsocks是常用的科学上网方法，有两个必要因素：

* ss客户端软件（windows与ubuntu均需要客户端才能科学上网）
* 配置ss服务的墙外云主机

# 1 客户端软件安装

# 2 墙外云主机配置

云主机服务提供商有很多，充分考虑使用需求与价格进行选择即可。这里记录一下使用谷歌云搭建ss服务的教程。谷歌云目前有免费用一年的活动，只有有一张VISA信用卡，即可在谷歌云上购买并免费使用云主机一年。

## 1 安装GUI 图形界面程序

按照提示配置相对应的参数，安装教程[地址](https://github.com/shadowsocks/shadowsocks-qt5/wiki/%E5%AE%89%E8%A3%85%E6%8C%87%E5%8D%97)

在ubuntu上可以这样，通过PPA源安装，仅支持Ubuntu 14.04或更高版本。

```
sudo add-apt-repository ppa:hzwhuang/ss-qt5
sudo apt-get update
sudo apt-get install shadowsocks-qt5
```

由于是图形界面，配置和windows基本没啥差别就不赘述了。经过上面的配置，你只是启动了sslocal 但是要上网你还需要配置下浏览器到指定到代理端口比如1080才可以正式上网。

## 2 在chrome中添加switchyOmega

打开switchyOmega 的[源码托管网站](https://github.com/FelisCatus/SwitchyOmega/releases)，点击 download

下载完成后，在浏览器地址栏输入

```
chrome://extensions/
```

将插件拖入，进行安装，跳过教程

switchyOmega配置详细说明[参考](https://github.com/FelisCatus/SwitchyOmega/wiki/GFWList)

配置代理时，输入方式为SOCKET5，地址为127.0.0.1，端口为1080

之后需要科学上网时，只需打开ss-qt5，选择需要连接的节点，然后浏览器点击插件switchyOmega - GFWed

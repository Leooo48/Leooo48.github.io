---
title: shadowsocks in windows
date: 2017-07-11 12:00:00
categories: 
- Windows
---

网址：Soloss.me 需科学上网并开启全局模式才可访问

付费+注册+登陆

# 1 下载客户端

shadowsocks客户端[百度云下载地址](http://pan.baidu.com/s/1c1vejmw)，密码：a46a。解压会发现两个exe格式的客户端，win7以下系统使用2.0，win8以上系统使用4.0。
（经测试这个用不了，遂找了ZL的客户端，[链接](http://pan.baidu.com/s/1nvLSTKX)，密码：z7z7）

# 2 下载配置文件

进入本站的节点中心，点击配置文件，然后就会下载一个gui-config.json文件，这个json文件包含了你所开通的套餐里所有节点信息（如果后面你又开通了其他计划，请更新本gui-config.json文件）。

# 3 将客户端和配置文件同地址

找到第一步的exe文件和第二步的json文件，并把它们放在同一个文件夹下

# 4 启动客户端

双击文件夹里的shadowsocksR.exe文件，启动客户端此时此刻右下角任务栏应有个小飞机图标。此时右键点击小飞机把启用系统代理，然后再选择PAC-更新本地PAC为GFWList。

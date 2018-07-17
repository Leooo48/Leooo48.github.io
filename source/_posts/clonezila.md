---
title: clonezila再生龙使用教程
date: 2018-07-13 09:22:47
categories: 
- 实用教程
---

再生龙是克隆硬盘或分区的常用工具，可以便捷的克隆整个硬盘，也可以克隆指定分区。Clonezila官网地址：[https://clonezilla.org/clonezilla-live-doc.php](https://clonezilla.org/clonezilla-live-doc.php)。

# 制作再生龙U盘启动盘

再生龙需要制作U盘启动盘才能使用，基本步骤是下载最新的Clonezila Live固件，然后将其制作成一个U盘启动盘。官网制作启动盘的[参考教程](https://clonezilla.org/clonezilla-live-doc.php)整理如下：

1. 系统要求：Windows 2000/XP/Vista/7, or GNU/Linux
2. 如果使用Windows系统，下载[Tuxboot软件](https://tuxboot.org/download/)，双击打开exe文件。
![](https://tuxboot.org/images/Tuxboot.png "Tuxboot主界面")
3. 启动盘的制作分为在线和离线两种方式：
![](https://tuxboot.org/images/Tuxboot-sel-dist.png "在线方式：选择Clonezila Live版本")
![](https://tuxboot.org/images/Tuxboot-sel-version.png "在线方式：选择Clonezila Live版本")
![](https://tuxboot.org/images/Tuxboot-support-zip.png "离线方式：选择预先下载的Clonezila Live文件")
4. 选择好U盘位置，点击OK就开始制作
![](https://tuxboot.org/images/Tuxboot-progress.png "开始制作")
5. 如果使用的是Linux或Ubuntu系统，安装Tuxboot流程如下，其余步骤见3和4。

```
sudo apt-add-repository ppa:thomas.tsai/ubuntu-tuxboot
sudo apt-get update
sudo apt-get install tuxboot
sudo tuxboot
```

# 再生龙克隆（恢复）硬盘（分区）流程

官方教程[在此](https://clonezilla.org/fine-print-live-doc.php?path=./clonezilla-live/doc/01_Save_disk_image/00-boot-clonezilla-live-cd.doc#00-boot-clonezilla-live-cd.doc)。

1. 重启电脑，进入BIOS，设置从再生龙的U盘启动。
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-01-bootmenu.png)
2. 选择合适的分辨率（如800×600），回车，选择语言“简体中文”
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-03-lang.png)
3. 选择使用默认键盘配置
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-04-keymap.png)
4. 选择使用再生龙
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-05-start-clonezilla.png)
5. 选择硬盘分区存到/来自镜像文件
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-06-dev-img.png)
6. 选择使用本机的分区
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-07-img-repo.png)
7. 插入镜像要保存的U盘或移动硬盘，回车
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-07-plug-and-play-dev-prompt.png)
8. 查看列表中是否有刚插入的设备，如果有，Crtl+c
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-07-dev-scan.png)
9. 选择保存镜像的设备
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-08-sdb1-as-img-repo.png)
10. TAB选择Done，默认将镜像保存在根目录下
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-08-sdb1-dir-list.png)
11. 选择初学者模式
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-08-beginner-expert-mode.png)
12. 按需选择储存本机硬盘位镜像文件或储存本机分区为镜像文件
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-08-save-img.png)
13. 输入镜像名称
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-10-img-name.png)
14. 在需要备份的硬盘或分区前按空格，如出现\*则代表选择该硬盘或分区
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-10-disk-selection.png)
15. 选择跳过检查
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-10-check-source-fs.png)
16. 选择检查保存镜像
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-10-check-if-image-restorable.png)
17. 选择不加密
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-10-encrypt-image.png)
18. 克隆完成的操作，三项任选其一
![](https://clonezilla.org/clonezilla-live/doc/01_Save_disk_image/images/ocs-10-reboot-poweroff.png)
19. 等待克隆结束

**恢复过程与克隆过程类似，操作时注意选择对应选项。**
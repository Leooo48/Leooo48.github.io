---
title: 基于CaffeNet完成的一个车辆朝向检测网络
date: 2017-11-20 11:42:44
---

本文是基于caffe自带的caffenet网络，采用KITTI数据集完成的一个车辆朝向检测网络。

## 数据集准备

本文将KITTI数据集的object类数据中，车辆所在的包围框进行裁切，保证每一个车辆目标单独成一张图片，并依次排序，最终获得了33260个车辆目标图。每个车辆会有一个朝向的真值，在KITTI的label里面，是属性为rotate\_y的这一项，即最后一项。通过观察，label将朝向分为了-3.14~3.14共629类，因此采用一个629类的分类网络对朝向进行分类。

程序如下，在ubuntu下的c++程序，基本思路是依次读入图片与对应的label文件。将label文件的car、truck、trum等车辆类型的行进行读入，将该行中的车辆包围框坐标（4个）、朝向信息（1个）保存起来。完成之后，对于每一个目标进行图像裁切，并将裁切后的图像保存，序号依次递增。将图片序号与对应的朝向真值写入txt文件中，格式为图像路径+空格+朝向真值。
```

```

将图像数据集分为训练集和测试集。

caffe网络的图像数据输入需要计算mean file。首先用caffe自带的convert_imageset工具将图像数据集转换为LMDB格式，再调用compute_image_mean工具计算图像均值。

caffenet网络输入改为image_data_layer，直接读入图像。参考自[关于caffe里面image_data_layer的应用](http://blog.csdn.net/kuaitoukid/article/details/51323940)

```
layer {  
  name: "data"  
  type: "ImageData"  
  top: "data"  
  top: "label"  
  include {  
    phase: TRAIN # 测试则改为TEST  
  }  
  transform_param {  
    scale: 0.00390625  
    mean_file: "path to meanfile"  
  }  
  image_data_param {  
    source: "data/hwdb/trainall.txt"  
    root_folder: "data/hwdb/DstImg/"  
    new_height: 48  
    new_width: 48  
    is_color: true 
    batch_size: 256  
    shuffle: true  
  }  
}  
```

至此，训练数据集全部准备完毕。

## 训练

```
./tools/caffe train --solver=solver.prototxt
```

## 测试
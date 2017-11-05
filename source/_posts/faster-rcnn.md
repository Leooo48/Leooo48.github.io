---
title: Faster R-CNN用自己的数据集训练
date: 2017-11-02 17:31:31
categories: 
- Caffe
---

Faster R-CNN的原理就不多说了，在做课题过程中需要重新训练车辆检测模型，因此本文记录一下操作的过程，以备日后复现。

# 环境

我是用的环境是Ubuntu 14.04 + Titan Xp(12Gb) + cuda 8.0

# 编译

1. 从github上把Faster RCNN项目的code给clone下来
```
git clone –recursive https://github.com/rbgirshick/py-faster-rcnn.git
```
2. 编译lib
```
cd $FRCN_ROOT/lib
make
```
3. 编译caffe和pycaffe
```
cd $FRCN_ROOT/caffe-fast-rcnn
cp Makefile.config.examplem Makefile.config
```
修改Makefile.config
```
WITH_PYTHON_LAYER := 1
USE_CUDNN := 1
```
然后编译
```
mkdir build
cd build
cmake ..
make -j12 && make pycaffe
```
等待编译完成即可。

# 用demo.py测试代码是否正常

下载与训练好的模型，并运行demo程序。

```
cd $FRCN_ROOT
./data/scripts/fetch_faster_rcnn_models.sh

./tools/demo.py
```

如果正常出图，并且做好了物体检测，则代码正常。

# 训练自己的数据集

Faster RCNN是在PASCAL VOC 2007的数据集上进行训练的，默认的数据集结构包含
> JPEGImages：存放用来训练的原始图像
> Annotations：存放原始图像中的目标坐标信息，xml格式
> ImageSets/Main：指定train、trainval、test、val的图片编号，txt格式

如果使用其他数据集的话，如KITTI，则需要对数据集进行转换。这里提供两个实测非常方便的python脚本，用来合并目标类，生成xml文件。

```
# modify_annotations_txt.py
# coding=UTF-8
import glob
import string

txt_list = glob.glob('./Labels/*.txt') # 存储Labels文件夹所有txt文件路径
def show_category(txt_list):
    category_list= []
    for item in txt_list:
        try:
            with open(item) as tdf:
                for each_line in tdf:
                    labeldata = each_line.strip().split(' ') # 去掉前后多余的字符并把其分开
                    category_list.append(labeldata[0]) # 只要第一个字段，即类别
        except IOError as ioerr:
            print('File error:'+str(ioerr))
    print(set(category_list)) # 输出集合

def merge(line):
    each_line=''
    for i in range(len(line)):
        if i!= (len(line)-1):
            each_line=each_line+line[i]+' '
        else:
            each_line=each_line+line[i] # 最后一条字段后面不加空格
    each_line=each_line+'\n'
    return (each_line)

print('before modify categories are:\n')
show_category(txt_list)

for item in txt_list:
    new_txt=[]
    try:
        with open(item, 'r') as r_tdf:
            for each_line in r_tdf:
                labeldata = each_line.strip().split(' ')
                if labeldata[0] in ['Truck','Van','Tram','Car']: # 合并汽车类
                    labeldata[0] = labeldata[0].replace(labeldata[0],'car')
                if labeldata[0] in ['Person_sitting','Pedestrian']: # 合并行人类
                    continue
                #    labeldata[0] = labeldata[0].replace(labeldata[0],'pedestrian')
                if labeldata[0] == 'Cyclist': # 合并行人类
                    continue
                #    labeldata[0] = labeldata[0].replace(labeldata[0],'cyclist')
                if labeldata[0] == 'DontCare': # 忽略Dontcare类
                    continue
                if labeldata[0] == 'Misc': # 忽略Misc类
                    continue
                new_txt.append(merge(labeldata)) # 重新写入新的txt文件
        with open(item,'w+') as w_tdf: # w+是打开原文件将内容删除，另写新内容进去
            for temp in new_txt:
                w_tdf.write(temp)
    except IOError as ioerr:
        print('File error:'+str(ioerr))

print('\nafter modify categories are:\n')
show_category(txt_list)
```

```
# txt_to_xml.py
# encoding:utf-8
# 根据一个给定的XML Schema，使用DOM树的形式从空白文件生成一个XML
from xml.dom.minidom import Document
import cv2
import os
import math

def generate_xml(name,split_lines,img_size,class_ind):
    doc = Document()  # 创建DOM文档对象

    annotation = doc.createElement('annotation')
    doc.appendChild(annotation)

    title = doc.createElement('folder')
    title_text = doc.createTextNode('VOC2007')#KITTI
    title.appendChild(title_text)
    annotation.appendChild(title)

    img_name=name+'.png'

    title = doc.createElement('filename')
    title_text = doc.createTextNode(img_name)
    title.appendChild(title_text)
    annotation.appendChild(title)

    source = doc.createElement('source')
    annotation.appendChild(source)

    title = doc.createElement('database')
    title_text = doc.createTextNode('The KITTI Database')
    title.appendChild(title_text)
    source.appendChild(title)

    title = doc.createElement('annotation')
    title_text = doc.createTextNode('KITTI')
    title.appendChild(title_text)
    source.appendChild(title)

    size = doc.createElement('size')
    annotation.appendChild(size)

    title = doc.createElement('width')
    title_text = doc.createTextNode(str(img_size[1]))
    title.appendChild(title_text)
    size.appendChild(title)

    title = doc.createElement('height')
    title_text = doc.createTextNode(str(img_size[0]))
    title.appendChild(title_text)
    size.appendChild(title)

    title = doc.createElement('depth')
    title_text = doc.createTextNode(str(img_size[2]))
    title.appendChild(title_text)
    size.appendChild(title)

    for split_line in split_lines:
        line=split_line.strip().split()
        if line[0] in class_ind:
            object = doc.createElement('object')
            annotation.appendChild(object)

            title = doc.createElement('name')
            title_text = doc.createTextNode(line[0])
            title.appendChild(title_text)
            object.appendChild(title)

            bndbox = doc.createElement('bndbox')
            object.appendChild(bndbox)
            title = doc.createElement('xmin')
            title_text = doc.createTextNode(str(int(float(line[4]))))
            title.appendChild(title_text)
            bndbox.appendChild(title)
            title = doc.createElement('ymin')
            title_text = doc.createTextNode(str(int(float(line[5]))))
            title.appendChild(title_text)
            bndbox.appendChild(title)
            title = doc.createElement('xmax')
            title_text = doc.createTextNode(str(int(float(line[6]))))
            title.appendChild(title_text)
            bndbox.appendChild(title)
            title = doc.createElement('ymax')
            title_text = doc.createTextNode(str(int(float(line[7]))))
            title.appendChild(title_text)
            bndbox.appendChild(title)

            orient = doc.createElement('orient')
            title_text = doc.createTextNode(line[3])
            orient.appendChild(title_text)
            object.appendChild(orient)

    # 将DOM对象doc写入文件
    f = open('Annotations/'+name+'.xml','w')
    f.write(doc.toprettyxml(indent = ''))
    f.close()

if __name__ == '__main__':
    class_ind=('car')#, 'pedestrian', 'cyclist')
    cur_dir=os.getcwd()
    labels_dir=os.path.join(cur_dir,'Labels')
    for parent, dirnames, filenames in os.walk(labels_dir): # 分别得到根目录，子目录和根目录下文件   
        for file_name in filenames:
            full_path=os.path.join(parent, file_name) # 获取文件全路径
            f=open(full_path)
            split_lines = f.readlines()
            name= file_name[:-4] # 后四位是扩展名.txt，只取前面的文件名
            img_name=name+'.png' 
            img_path=os.path.join('/home/inin/catkin_ws/src/idr_vehdet/py-faster-rcnn/data/VOCdevkit2007/VOC2007/JPEGImages',img_name) # 路径需要自行修改            
            img_size=cv2.imread(img_path).shape
            generate_xml(name,split_lines,img_size,class_ind)
print('all txts has converted into xmls')
```

# 代码修改

我使用的是faster_rcnn_end2end.sh进行训练。需要调整的文件有：

1. ./lib/datasets/pascal_voc.py
```
# line 30
self._classes = ('__background__', 'car') # always index 0
                 #'aeroplane', 'bicycle', 'bird', 'boat',
                 #'bottle', 'bus', 'car', 'cat', 'chair',
                 #'cow', 'diningtable', 'dog', 'horse',
                 #'motorbike', 'person', 'pottedplant',
                 #'sheep', 'sofa', 'train', 'tvmonitor')

# line 188-195 全部注释掉
#if not self.config['use_diff']:
    # Exclude the samples labeled as difficult
#    non_diff_objs = [
#        obj for obj in objs if int(obj.find('difficult').text) == 0]
    # if len(non_diff_objs) != len(objs):
    #     print 'Removed {} difficult objects'.format(
    #         len(objs) - len(non_diff_objs))
#    objs = non_diff_objs

# line 208
x1 = float(bbox.find('xmin').text) #- 1
y1 = float(bbox.find('ymin').text) #- 1
x2 = float(bbox.find('xmax').text) #- 1
y2 = float(bbox.find('ymax').text) #- 1
```
2. ./lib/datasets/imdb.py
```
# line 102
def append_flipped_images(self): #modfied
    num_images = self.num_images
    #widths = self._get_widths()
    widths = [PIL.Image.open(self.image_path_at(i)).size[0] for i in xrange(num_images)]
    for i in xrange(num_images):
        boxes = self.roidb[i]['boxes'].copy()
        oldx1 = boxes[:, 0].copy()
        oldx2 = boxes[:, 2].copy()
        boxes[:, 0] = widths[i] - oldx2 - 1
        print boxes[:, 0]
        boxes[:, 2] = widths[i] - oldx1 - 1
        print boxes[:, 0]
        assert (boxes[:, 2] >= boxes[:, 0]).all()
        entry = {'boxes' : boxes,
                 'gt_overlaps' : self.roidb[i]['gt_overlaps'],
                 'gt_classes' : self.roidb[i]['gt_classes'],
                 'flipped' : True}
        self.roidb.append(entry)
    self._image_index = self._image_index * 2
```
3. ./models/pascal_voc/VGG_CNN_M_1024/faster_rcnn_end2end/train.protxt
修改类别为你需要检测类别数目，例如我检测车辆和背景，则将类别数21改为2
4. ./models/pascal_voc/VGG_CNN_M_1024/faster_rcnn_end2end/test.protxt
修改类别为你需要检测类别数目，例如我检测车辆和背景，则将类别数21改为2
5. ./experiments/scripts/faster_rcnn_end2end.sh
修改line 53的finetune模型

# 开始训练

所有准备工作就绪，可以开始训练了
```
./experiments/scripts/faster_rcnn_end2end.sh 0 VGG_CNN_M_1024 pascal_voc
```

训练结果在./output文件夹中，可直接使用。

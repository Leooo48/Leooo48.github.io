---
title: opencv编写的image crop函数
date: 2017-07-11 12:00:00
categories:
- OpenCV
---

# 裁剪图像

传递参数：

* src：输入图像
* dst：输出图像
* rect：裁剪区域，为Rect类型，例如Rect(1,1,480,360)

```
int imageCrop(InputArray src, OutputArray dst, Rect rect)  
{  
    Mat input = src.getMat();  
    if( input.empty() ) {  
        return -1;  
    }  
  
    //计算剪切区域：  剪切Rect与源图像所在Rect的交集  
    Rect srcRect(0, 0, input.cols, input.rows);  
    rect = rect & srcRect;  
    if ( rect.width <= 0  || rect.height <= 0 ) return -2;  
  
    //创建结果图像  
    dst.create(Size(rect.width, rect.height), src.type());  
    Mat output = dst.getMat();  
    if ( output.empty() ) return -1;  
  
    try {  
        //复制源图像的剪切区域 到结果图像  
        input(rect).copyTo( output );  
        return 0;  
    } catch (...) {  
        return -3;  
    }  
}
```

Done!

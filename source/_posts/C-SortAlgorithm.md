---
title: C++-排序算法总结
date: 2017-07-19 13:27:41
categories: 
- C++
---

排序算法有快速排序法、冒泡法、选择法等，现对其总结并附代码如下。

参考[真实的归宿博客](http://blog.csdn.net/hguisu/article/details/7776068/)。

### **各种排序的稳定性、时间复杂度和空间复杂度总结：**

![各种排序的稳定性、时间复杂度和空间复杂度总结](http://my.csdn.net/uploads/201207/19/1342700879_2982.jpg)

# 快速排序法

#### **步骤：**

* 从数列中挑出一个元素，称为 “基准”（pivot），一般取第一个元素或随机选取；
* 重新排序数列，所有元素比基准值小的摆放在基准前面，所有元素比基准值大的摆在基准的后面（相同的数可以到任一边）。在这个分区退出之后，该基准就处于数列的中间位置。这个称为**分区（partition）**操作。
* 递归地（recursive）把小于基准值元素的子数列和大于基准值元素的子数列排序。

#### **程序：**

```    
void quickSort(int a[], int low, int high)
{
    if(low < high)
    {
    	int start=low;
    	int end=high;

        int privotKey = a[low];//基准元素选择第一个数组元素
        while(low < high)
        {                                   
            while(low < high  && a[high] >= privotKey)
                --high; //由右往左搜索小于基准值的元素
            int tmp = a[low];  
            a[low] = a[high]
            a[high] = tmp;

            while(low < high  && a[low] <= privotKey )
                ++low; //由左往右搜索大于基准值的元素
            int tmp = a[low];  
            a[low] = a[high]
            a[high] = tmp;
        }  

        int privotLoc =  low;
        quickSort(a,  start,  privotLoc - 1);   //递归对低子表递归排序  
        quickSort(a,  privotLoc + 1, end);   //递归对高子表递归排序  
    }  
} 
```

快速排序是通常被认为在同数量级的排序方法中平均性能最好的；同时快速排序是一个不稳定的排序方法。

# 选择排序法

在要排序的一组数中，选出最小（或者最大）的一个数与第1个位置的数交换；然后在剩下的数当中再找最小（或者最大）的与第2个位置的数交换，依次类推，直到第n-1个元素（倒数第二个数）和第n个元素（最后一个数）比较为止。

#### **步骤：**

* 从n个记录中找出关键码最小的记录与第一个记录交换；
* 第二趟，从第二个记录开始的n-1个记录中再选出关键码最小的记录与第二个记录交换；
* 以此类推.....
* 第i趟，则从第i个记录开始的n-i+1个记录中选出关键码最小的记录与第i 个记录交换，直到整个序列按关键码有序。

#### **程序：**

```
void selecteSort(int a[], int n)
{
    for(int i=0; i<n-1; i++)
    {
    	int amin=a[i];
    	int loc=i;
        for(int j=i+1; j<n; j++)
        {
            if(a[j]<amin)
            {
                amin=a[j];
                loc=j;
            }
        }

        int tmp=a[i];
        a[i]=a[loc];
        a[loc]=tmp;
    }
}
```

简单选择排序每趟循环只能确定一个元素排序后的定位，改进的选择排序法每趟循环确定两个元素（当前趟最大和最小记录）的位置，从而减少排序所需的循环次数。改进后对n个数据进行排序，最多只需进行n/2趟循环即可。

#### **程序：**

```
void SelectSort(int r[], int n) 
{  
    int i, j, min, max, tmp;  
    for (i=1; i<=n/2; i++) // 做不超过n/2趟选择排序   
    { 
        min = i; 
        max = i; //分别记录最大和最小关键字记录位置  
        for (j=i+1; j<=n-i; j++) 
        {  
            if (r[j] > r[max]) 
            {   
                max = j; 
                continue;   
            }    
            if (r[j]< r[min]) 
            {   
                min = j;   
            }     
        }    
      
        tmp = r[i-1];
        r[i-1] = r[min]; 
        r[min] = tmp; 

        tmp = r[n-i]; 
        r[n-i] = r[max]; 
        r[max] = tmp;   
    }   
}  
```

# 冒泡排序法

#### **步骤：**

在要排序的一组数中，对当前还未排好序的范围内的全部数，自上而下对相邻的两个数依次进行比较和调整，让较大的数往下沉，较小的往上冒。即：每当两相邻的数比较后发现它们的排序与排序要求相反时，就将它们互换。

由小到大：

```
void bubbleSort(int a[], int n)
{  
    for(int i =0 ; i< n-1; i++) //排序轮次，数组包含n个元素，需要n-1轮冒泡才能完成，因此i取值为[0,n-1)
    {  
        for(int j = 0; j < n-i-1; j++) //两两依次比较，大的靠后
        {  
            if(a[j] > a[j+1])  
            {  
                int tmp = a[j] ; 
                a[j] = a[j+1] ;  
                a[j+1] = tmp;  
            }  
        }  
    }  
}  
```

由大到小：

```
void bubbleSort(int a[], int n)
{  
    for(int i =0 ; i< n-1; i++) //排序轮次，数组包含n个元素，需要n-1轮冒泡才能完成，因此i取值为[0,n-1)
    {  
        for(int j = n-1; j > 0; j--) //两两依次比较，小的靠后
        {  
            if(a[j] > a[j-1])  
            {  
                int tmp = a[j] ; 
                a[j] = a[j-1] ;  
                a[j-1] = tmp;  
            }  
        }  
    }  
}  
```

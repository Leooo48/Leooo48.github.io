---
title: 数据结构与C++基础汇总(一)
date: 2017-08-05 22:39:16
categories:
- C++
---

面试所需的数据结构、C++基础知识与细节问题记录专帖。

## 面向对象编程的三个基本特征与五个设计原则

三个基本特征：
* 封装
* 继承
* 多态
![](http://www.cnitblog.com/images/cnitblog_com/lily/1972/o_OOBase.gif)

五个设计原则：
* 单一职责原则
* 开放封闭原则
* Liskov替换原则
* 依赖倒置原则
* 接口隔离原则

## i++与++i的效率问题

如果使用系统内建数据类型，则两者的效率无差别；如果使用的是用户自定义的数据类型，则++i的效率更高一些，因为在执行过程中，i++需要复制对象，而++i则不需要，因此效率较高。

## inlcude <> 与 "" 的区别

> \#include <\header1> 表明该文件为一个工程或标准头文件，编译程序会先到标准函数库（预定义目录）中查找，目录可设置
> \#include "header2"  表明该文件是用户提供的头文件，编译程序会先从当前目录中查找，然后再在标准位置查找

## char*与std::string类型的区别与联系

参考[wjzz的博客](http://blog.csdn.net/weijj6608/article/details/44681889)。

string可以被看成是以字符为元素的一种容器。字符构成序列（字符串）。标准的string类提供了STL容器接口，具有一些成员函数比如begin()、end()，迭代器可以根据他们进行定位。但是，string与char\*不同的是，string不一定以NULL('\0')结束。string长度可以根据length()得到，string可以根据下标访问。所以，不能将string直接赋值给char\*。

字符串函数参考[tzheng2008的博客](http://blog.csdn.net/tzheng2008/article/details/7342562)。

转化方法：

* string转const char*：

```
string s1 = "abcdeg";
const char *k = s1.c_str();
const char *t = s1.data();
```

* string转char*：

```
string s1 = "abcdefg";
char *data;
int len = s1.length();
data = (char *)malloc((len+1)*sizeof(char));
s1.copy(data,len,0);
```

* char*转string：

可以直接赋值。

```
string s;
char *p = "adghrtyh";
s = p;
printf("%s",s1.c_str());
```

* char[]转string

同上直接赋值，用printf输出时注意格式。

* string转char[]

```
string pp = "dagah";
char p[8];
int i;
for( i=0;i<pp.length();i++)
	p[i] = pp[i];
p[i] = '\0';
```

## 反转字符串的三种方法

* 采用双向遍历进行交换（缺点：交换要新定义变量）

* 双向遍历，采用逻辑异或进行交换，不需定义新变量

```
int r = str.Length - 1;
for (int i = 0; i < r; i++, r--)
{
	str[i] ^= str[r];
	str[r] ^= str[i];
	str[i] ^= str[r];
}
```

* 采用栈的后进先出特性反转字符串（缺点：进出栈开销与两次循环开销使得该方法的运算开销较大）

## C++多态性的理解

**多态性**意味着一个操作在不同的类中可以有不同的实现方式（阿里笔试题）。

以下内容参考[hackbuteer1的博客](http://blog.csdn.net/hackbuteer1/article/details/7475622)

多态性可以简单地概括为“一个接口，多种方法”，程序在运行时才决定调用的函数，它是面向对象编程领域的核心概念。

C++多态性是通过虚函数来实现的，虚函数允许子类重新定义成员函数，而子类重新定义父类的做法称为覆盖(override)，或者称为重写。（这里我觉得要补充，重写的话可以有两种，直接重写成员函数和重写虚函数，只有重写了虚函数的才能算作是体现了C++多态性）而重载则是允许有多个同名的函数，而这些函数的参数列表不同，允许参数个数不同，参数类型不同，或者两者都不同。编译器会根据这些函数的不同列表，将同名的函数的名称做修饰，从而生成一些不同名称的预处理函数，来实现同名函数调用时的重载问题。但这并没有体现多态性。

最常见的用法就是声明基类的指针，利用该指针指向任意一个子类对象，调用相应的虚函数，可以根据指向的子类的不同而实现不同的方法。如果没有使用虚函数的话，即没有利用C++多态性，则利用基类指针调用相应的函数的时候，将总被限制在基类函数本身，而无法调用到子类中被重写过的函数。因为没有多态性，函数调用的地址将是一定的，而固定的地址将始终调用到同一个函数，这就无法实现一个接口，多种方法的目的了。

笔试题目：

```
#include<iostream>
using namespace std;

class A
{
public:
	void foo()
	{
		printf("1\n");
	}
	virtual void fun()
	{
		printf("2\n");
	}
};
class B : public A
{
public:
	void foo()
	{
		printf("3\n");
	}
	void fun()
	{
		printf("4\n");
	}
};
int main(void)
{
	A a;
	B b;
	A *p = &a;
	p->foo();
	p->fun();
	p = &b;
	p->foo();
	p->fun();
	return 0;
}
```

第一个p->foo()和p->fuu()都很好理解，本身是基类指针，指向的又是基类对象，调用的都是基类本身的函数，因此输出结果就是1、2。

第二个输出结果就是1、4。p->foo()和p->fuu()则是基类指针指向子类对象，正式体现多态的用法，p->foo()由于指针是个基类指针，指向是一个固定偏移量的函数，因此此时指向的就只能是基类的foo()函数的代码了，因此输出的结果还是1。而p->fun()指针是基类指针，指向的fun是一个虚函数，由于每个虚函数都有一个虚函数列表，此时p调用fun()并不是直接调用函数，而是通过虚函数列表找到相应的函数的地址，因此根据指向的对象不同，函数地址也将不同，这里将找到对应的子类的fun()函数的地址，因此输出的结果也会是子类的结果4。

练习： B \*ptr = (B \*)&a;  ptr->foo();  ptr->fun();
输出： 3，2

**小结：**
1. 有virtual才可能发生多态现象  
2. 不发生多态（无virtual）调用就按原类型调用  

## 二叉树遍历方法

根据根节点位置的不同分为三种：
先序序列或前序序列：根节点、左子节点、右子节点
中序序列：左子节点、根节点、右子节点
后序序列：左子节点、右子节点、根节点

给定一个二叉树的先序序列与中序序列，可以确定其后序序列。方法是：通过先序序列确定根节点，将中序序列分为左右两支，在递归来划分，直到到达终结点。

## 内存分配new/delete与malloc/free的区别

参考[hackbuteer1的博客](http://blog.csdn.net/hackbuteer1/article/details/6789164)。

二者都可用于申请动态内存和释放内存，不同点是malloc与free是C++/C语言的**标准库函数**，new/delete是C++的**运算符**。对于非内部数据类的对象而言，光用maloc/free 无法满足动态对象的要求，因为对象在创建的同时要自动执行构造函数，对象消亡之前要自动执行析构函数。由于malloc/free是库函数而不是运算符，不在编译器控制权限之内，不能够把执行构造函数和析构函数的任务强加malloc/free。

**用法：**

函数malloc的原型如下：
```
void * malloc(size_t size);
```
用malloc 申请一块长度为length 的整数类型的内存，程序如下：
```
int *p = (int *) malloc(sizeof(int) * length);
```
我们应当把注意力集中在两个要素上：“类型转换”和“sizeof”:
1. malloc 返回值的类型是void \*，所以在调用malloc 时要显式地进行类型转换，将void \* 转换成所需要的指针类型。
2. malloc 函数本身并不识别要申请的内存是什么类型，它只关心内存的总字节数。

函数free的原型如下：
```
void free( void * memblock );
```
如果p 是NULL 指针，那么free对p 无论操作多少次都不会出问题。如果p 不是NULL 指针，那么free 对p连续操作两次就会导致程序运行错误。

new/delete 的使用要点：
运算符new 使用起来要比函数malloc 简单得多，例如：
```
int *p1 = new int[length];
```
这是因为new 内置了sizeof、类型转换和类型安全检查功能。对于非内部数据类型的对象而言，new 在创建动态对象的同时完成了初始化工作。如果对象有多个构造函数，那么new 的语句也可以有多种形式。
如果用new 创建对象数组，那么只能使用对象的无参数构造函数。例如
```
Obj *objects = new Obj[100];       // 创建100 个动态对象
```
释放的时候别忘了前面的[]：
```
delete []objects;
```
如果不加[]，则相当于delete objects[0]，漏掉了另外99 个对象。

**小结：**
* new自动计算需要分配的空间，而malloc需要手工计算字节数
* new是类型安全的，而malloc不是，比如：
	int\* p = new float[2]; // 编译时指出错误
	int\* p = malloc(2\*sizeof(float)); // 编译时无法指出错误
	new operator 由两步构成，分别是 operator new 和 construct
* operator new对应于malloc，但operator new可以重载，可以自定义内存分配策略，甚至不做内存分配，甚至分配到非内存设备上。而malloc无能为力
* new将调用constructor，而malloc不能；delete将调用destructor，而free不能。
* malloc/free要库文件支持，new/delete则不要。
* **new/delete可以调用类的构造函数与析构函数。**
* free之后，要把指针赋值为NULL，否则会变成野指针。

## debug与release的区别

* Debug 通常称为调试版本，它包含调试信息，并且不作任何优化，便于程序员调试程序
* Release 称为发布版本，它往往是进行了各种优化，使得程序在代码大小和运行速度上都是最优的，以便用户很好地使用

Debug和Release没有本质的界限，他们只是一组编译选项的集合，编译器只是按照预定的选项行动。

## const的用法

参考[博客](http://www.cnblogs.com/wintergrass/archive/2011/04/15/2015020.html)。

const是常量限定符，用法包括以下几个方面：
* 修饰基本数据类型，如int、char等，定义常量
* 应用在函数中，修饰函数的形参或者返回值
* 在类中的应用

1、修饰常量
```
const int a=10; //等价与 int const a=10;

const int* a = & [1] 
int const *a = & [2] //const在\*之前，表示指针所指向的变量是常量，而指针不是常量
int* const a = & [3] //const在\*之后，表示指针本身是常量，而指针所指向的变量不是常量
const int* const a = & [4]    //指针及所指向的变量均为常量

int const &a=x;
const int &a=x; //常量引用，即a的值不能修改
```
总结一下就是：
> 如果const位于星号\*的左侧，则const就是用来修饰指针所指向的变量，即指针指向为常量；
> 如果const位于星号\*的右侧，const就是修饰指针本身，即指针本身是常量。

2、应用在函数中
* 作为函数参数的const修饰，通常用于参数为指针或引用的情况，不能对传递进来的指针的内容进行改变，保护了原指针所指向的内容，如形参为 const A& a ，则不能对传递进来的引用对象进行改变。
* 修饰函数返回值

3、在类中的应用
修饰类的成员函数（定义体），任何不会修改数据成员的函数都应该用const修饰，形式为：int GetCount(void) const;

## C++ 虚函数&纯虚函数&抽象类&接口&虚基类

参考[fly1988happy](http://www.cnblogs.com/fly1988happy/archive/2012/09/25/2701237.html)的讲解，十分清晰的对比和分析了每个定义的详细内涵。

## C语言优先级与结合性

![](https://github.com/Leooo48/markdownimages/blob/master/C%E8%AF%AD%E8%A8%80%E8%BF%90%E7%AE%97%E7%AC%A6%E7%9A%84%E4%BC%98%E5%85%88%E7%BA%A7%E5%92%8C%E7%BB%93%E5%90%88%E6%80%A7%E4%B8%80%E8%A7%88%E8%A1%A8.png?raw=true)

## static关键字的作用

* 隐藏：当同时编译多个文件时，所有未加static前缀的全局变量和函数都具有全局可见性，而加static前缀之后就变为当前文件可见的全局变量和函数。
* 保持变量内容的持久：共有两种变量存储在静态存储区：全局变量和static变量。存储在静态数据区的变量会在程序刚开始运行时就完成初始化，也是唯一的一次初始化。
* 默认初始化为0（全局变量也具备这一属性，因为全局变量也存储在静态数据区）：在静态数据区，内存中所有的字节默认值都是0x00。

理解全局变量、静态全局变量、静态局部变量、局部变量的区别，参考[博客](http://www.cnblogs.com/yc_sunniwell/archive/2010/07/14/1777441.html)。


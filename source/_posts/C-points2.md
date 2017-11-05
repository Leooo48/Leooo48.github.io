---
title: 数据结构与C++基础汇总(二)
date: 2017-08-08 22:26:08
categories: 
- C++
---

面试所需的数据结构、C++基础知识与细节问题记录专帖（接上帖）。

## 有符号变量与无符号变量值的转换

当表达式中存在有符号类型和无符号类型时，所有的操作数都自动转换成**无符号类型**。
负数转化为无符号整形数后，会是一个非常大的数，相加后可能会溢出，从而和为0。

## 用#define声明一个常数
如每年多少秒：
`#define SECOND_PER_YEAR (60*60*24*365)UL`
注意：`#define`的表达式中可以计算，并且若结果较大，加上L代表长整型，同时若数据恒为正，则加U代表无符号整型。

## 预处理的应用
```
#include <stdio.h>

#define DEBUG

int main()
{
	...
#ifdef DEBUG
	printf("debug info\n");
#endif
	...
}
```
这里用#ifdef判断DEBUG是否被预定义了。若预定义了，则执行printf语句；否则执行#endif后的语句，而不执行printf语句。

## 宏参数的连接

在宏定义中，用#吧宏参数变成一个字符串，用##把两个宏参数连接起来。
```
#include <stdio.h>

#define STR(s) #s
#define CONS(a,b) (int)(a##e##b)

int main()
{
	...
}
```

## sizeof计算变量所占空间大小

计算结果是**字节**，即所占几个字节。**1字节=8bit=8位，1位=0或1。**字由若干个字节构成，字的位数叫做字长，不同档次的机器有不同的字长。例如一台8位机，它的1个字就等于1个字节，字长为8位。如果是一台16位机，那么，它的1个字就由2个字节构成，字长为16位。

1、普通变量大小（32位编译系统）
```
sizeof(int)=4;
sizeof(char)=1;
sizeof(short)=2;
sizeof(long)=8;
sizeof(double)=8;

char *p;
sizeof(p)=4; //指针大小恒为4

char str[]="Hello"
sizeof(str)=6; //5+1 因为末尾有结束符 ‘\0’
```

2、类/结构体对象大小
```
class E
{
public:
	int i;
	int ii;
	short j;
	char ch;
	int iii;
}

sizeof(E)=12; //4+4+2+1+1(补充)+4=16
```
类或结构体的大小计算要满足对齐原则：
* 首地址要能够被最宽的基本类型大小整除
* 每个成员相对于首地址的偏移量都是成员大小的整数倍，如果不是会补充字节
* 总大小是最宽的基本类型大小的整数倍，如果不是会补充字节

注意：
* 只有类的变量占用大小，类的函数是不占用内存的。空类默认分配一个char用来表示类的对象。
* 类的静态成员储存在静态存储区中，不算在类的大小内。
* 对于包含**虚函数**的类来说，除了本身变量占用的大小外，多加了一个指针大小（4），用来保存虚表指针成员。
* 继承时，子类包含一个指向父类的指针，大小为4。

3、联合体大小

联合体的大小取决于它所有成员中占用空间最大的一个成员大小，但要注意cpu对齐问题。
```
union u
{
	double a;
	int b;
}

union u2
{
	char a[13];
	int b;
}

sizeof(u)=8;
sizeof(u2)=16; //char型数组大小为13，但还有一个int型变量，对齐方式为4，因此13应补齐4的整数倍，即16
```

4、CPU对齐问题

可以用 #pragma pack(x) 改变编译器的对齐方式，C++固有类型对齐方式取编译器对齐方式与自身大小中较小的一个。例如编译器按照2对齐，int型大小为4，则对齐取2与4较小的2作为对齐方式。

如无特殊说明，编译器默认对齐为8，各个成员大小小于8的均采用各自大小的对齐方式。

## 联合体与结构体的区别

* 联合体：几个不同类型的变量占用同一段内存，互相之间不能同时存在
* 结构体：把不同数据类型的变量组合成一个新的整体，从而定义出一个新的数据类型，互相之间是可以同时存在的

**注意：**

1. struct和union都是由多个不同的数据类型成员组成, 但在任何同一时刻, union中只存放了一个被选中的成员, 而struct的所有成员都存在。在struct中，各成员都占有自己的内存空间，它们是同时存在的。一个struct变量的总长度等于所有成员长度之和。在Union中，所有成员不能同时占用它的内存空间，它们不能同时存在。Union变量的长度等于最长的成员的长度。
2. 对于union的不同成员赋值, 将会对其它成员重写, 原来成员的值就不存在了, 而对于struct的不同成员赋值是互不影响的。
```
union myun  
{  
     struct { int x; int y; int z; }u;  
     int k;  
}a;  
int main()  
{  
     a.u.x =4;  
     a.u.y =5;  
     a.u.z =6;  
     a.k = 0;  
     printf("%d %d %d\n",a.u.x,a.u.y,a.u.z);  
     return 0;  
}  
```
输出为  `0 5 6`。

## 数组指针+1的意义
```
main()
{
	int a[5]={1,2,3,4,5};
	int *ptr=(int *)(&a+1); 
	printf("%d,%d",*(a+1),*(ptr-1));
}
```
输出为 `2,5`。因为`*(a+1）`就是a[1]，`*(ptr-1)`就是a[4],执行结果是 `2,5`。
a,&a的地址是一样的，但意思不一样：
* a是数组首地址，也就是a[0]的地址，a+1是数组下一元素的地址，即a[1]；
* &a是对象（数组）首地址，&a+1不是首地址+1，系统会认为加一个a数组的偏移，是偏移了一个数组的大小（本例是5个int），也就是说&a+1是下一个对象的地址，即a[5]。

&a是数组指针，其类型为 `int (*)a[5]`；而指针加1要根据指针类型加上一定的值，不同类型的指针+1之后增加的大小不同，a是长度为5的int数组指针，所以要加 `5*sizeof(int)`，所以ptr实际是a[5]。
但是prt与(&a+1)类型是不一样的(**这点很重要**)。所以，prt-1只会减去`sizeof(int*)`，也就是a[4]。

## assert函数用法总结

assert宏的原型定义在<assert.h>中，其作用是如果它的条件返回错误，则终止程序执行，原型定义：
```
#include <assert.h>
void assert( int expression );
```
assert的作用是现计算表达式 expression ，如果其值为假（即为0），那么它先向stderr打印一条出错信息，然后通过调用 abort 来终止程序运行。

**注意事项**
* 在函数开始处检验传入参数的合法性如：
```
int resetBufferSize(int nNewSize)
{
　　//功能:改变缓冲区大小,
　　//参数:nNewSize 缓冲区新长度
　　//返回值:缓冲区当前长度 
　　//说明:保持原信息内容不变     nNewSize<=0表示清除缓冲区
　　assert(nNewSize >= 0);
　　assert(nNewSize <= MAX_BUFFER_SIZE);
　　...
}
```
* 每个assert只检验一个条件，因为同时检验多个条件时，如果断言失败，无法直观的判断是哪个条件失败
```
assert(nOffset >= 0);
assert(nOffset+nSize <= m_nInfomationSize);
```
* 不能使用改变环境的语句，因为assert只在DEBUG个生效，如果这么做，会使用程序在真正运行时遇到问题
* assert和后面的语句应空一行，以形成逻辑和视觉上的一致感
* 有的地方，assert不能代替条件过滤

## 判断是否为回文字符串
递归法：
```
#include <iostream>  
using namespace std;  

int fun(int low, int high, char *str, int length)  
{  
    if (length == 0 || length == 1)  
        return    1;  
    if (str[low] != str[high])  
        return    0;  
    return fun(low+1, high-1, str, length-2);  
}  

int main()  
{  
    char    str[]="aaabdaaa";  
    int     length = strlen(str);  
    //返回1代表是， 0代表不是  
    cout << fun(0, length-1, str, length) << endl;  
    return    0;  
}  
```

## 字符串操作函数
包含头文件：`#include <string.h> // C语言`
包含头文件：`#include <cstring> // C++`

输入字符串可以是`char str[]="abcdefg";`或者`char str[7]="abcdefg";`或者`char *str=new char[10]; cin>>str;`

操作函数：
`extern char* strcpy(char* dest, const char* src)`：把从src地址开始的、含有`'\0'`结束符的字符串，复制到以dest地址开始的字符串中，并返回指向dest的指针。也就是将src字符数组复制到dest数组中，如果dest数组本身有数据，则dest中数据小于src地址长度的将会被覆盖，而大于src长度的将保留
`extern char* strstr(char* str1, char* str2)`：从字符串str1中查找是否有字符串str2， 如果有，从str1中的str2位置起，返回str1的指针，如果没有，返回NULL
`extern int strcmp(const char* s1, const char* s2)`：比较字符串s1和s2，当s1<s2时，返回值<0；当s1=s2时，返回值=0；当s1>s2时，返回值>0
`extern unsigned int strlen(char* s)`：计算字符串的长度，不包括`'\0'`在内
`extern char* strcat(char* dest, const char* src)`：连接字符串，将`src`连接到`dest`的尾部，覆盖`\0`
`extern unsigned int strrev(char* s)`：反转字符串，只对字符数组有用，对`string`类型是没用的

## stack 栈的使用

包含头文件：`#include <stack>`
定义一个栈：`stack<int> stk1;`
入栈：`stk1.push(obj);`
出栈：`stk1.pop();`
栈顶指针：`stk1.top();`

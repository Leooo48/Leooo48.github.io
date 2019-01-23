---
title: C++中模板template的使用
date: 2019-01-23 13:32:01
categories: 
- C++
---

模板是泛型编程的基础，泛型编程即以一种独立于任何特定类型的方式编写代码。

模板是创建泛型类或函数的蓝图或公式。例如库容器就是泛型编程的例子，每个容器都有一个单一的定义。我们常用的 **向量** 就是一个例子，可以定义许多不同类型的向量，如 `vector <int>` 或 `vector <string>`。

# typename与class的区别

首先，我们先来明确一个最基本的问题：在c++ Template中非常多地方都用到了**typename**与**class**这两个关键字，并且好像能够替换，是不是这两个关键字全然一样呢? 

事实上，**class**常被用于定义类。在模板引入c++后，最初定义模板的方法为：`template <class T>`，这里**class**关键字表明**T**是一个类型。后来，为了避免**class**在这两个地方的使用可能给人带来混淆，所以引入了**typename**这个关键字。它的作用同**class**一样，表明后面的符号为一个类型。这样在定义模板的时候就能够使用以下的方式了： `template <typename T>`。
**在模板定义语法中关键字class与typename的作用全然一样。**

# 类模板与模板类

## 1. 什么是类模板 

一个类模板（也称为类属类或类生成类）允许用户为类定义一种模式，使得类中的某些**数据成员**、成员函数的**參数**、某些成员函数的**返回值**能够取任意类型（包含系统提前定义的和用户自己定义的）。 
假设一个类中数据成员的数据类型不能确定，或者是某个成员函数的參数或返回值的类型不能确定，就必须将此类声明为模板。类模板的存在不是代表一个详细的、实际的类，而是代表着**一类**类。

## 2. 类模板定义 

定义一个类模板，一般有2方面的内容： 
A. 首先要定义类，其格式为：

```
template <class T>
class foo {
……
}
```

foo 为类名，在类定义体中，如采用通用数据类型的成员，函数參数的前面需加上T。当中通用类型T能够作为普通成员变量的类型，还能够作为const和static成员变量以及成员函数的參数和返回类型之用，如：

```
template <class T>
class Test {
 private:
  T n;
  const T i;
  static T cnt;
 public:
  Test():i(0) {}
  Test(T k);
  ~Test(){}
  void print();
  T operator+(T x);
};
```

B. 在类定义体外定义成员函数时，若此成员函数中有模板參数存在，则除了须要和一般类的体外定义成员函数一样的定义外，还需在函数体外进行**模板声明** ，如：

```
template <class T>
void Test<T>::print() {
  std::cout << "n=" << n << std::endl;
  std::cout << "i=" << i << std::endl;
  std::cout << "cnt=" << cnt << std::endl;
}
```

假设函数是以通用类型为返回类型，则要在函数名前的类名后缀上`<T>`，如：

```
template<class T>
Test<T>::Test(T k) : i(k) { 
  n = k;
  cnt++;
}

template<class T>
T Test<T>::operator+(T x){
  return n + x;
}
```

C. 在类定义体外初始化const成员和static成员变量的做法和普通类体外初始化const成员和static成员变量的做法基本上是一样的，唯一的差别是需再对**模板进行声明**，如:

```
template<class T>
int Test<T>::cnt = 0;

template <class T>
Test<T>::Test(T k) : i(k) {
  n  =k;
  cnt++;
}
```

## 3. 类模板的使用

类模板的使用实际上是将类模板**实例化**成一个详细的类，它的格式为：类名<实际的类型>。模板类是类模板实例化后的一个产物。

# 函数模板和模板函数

## 1. 函数模板 

函数模板能够用来创建一个通用的函数，以支持多种不同的形參，避免重载函数的函数体反复设计。
它的最大特点是把函数使用的数据类型作为參数。

函数模板的声明形式为：

```
template <typename（或class) T>
<返回类型> <函数名>(參数表) {
   函数体
}
```

- template是定义模板函数的关键字
- template后面的尖括号不能省略
- typename（或class)是声明数据类型參数标识符的关键字，用以说明它后面的标识符是数据类型标识符。

这样，在以后定义的这个函数中，凡希望依据实參数据类型来确定数据类型的变量，都能够用数据类型參数标识符来说明，从而使这个变量能够适应不同的数据类型。

```
template<typename（或class) T>
T fuc(T x, T y) {
  T x;
  ……
}
```

函数模板仅仅是声明了一个函数的描写叙述，并不是一个能够直接运行的函数。只有在依据实际情况用实參的数据类型取代类型參数标识符之后，才是真正的函数。

## 2. 模板函数 

模板函数的生成就是将函数模板的类型形參实例化的过程。

```
double d;
int a;
fuc(d,a);
```

则系统将用实參d的数据类型double去取代函数模板中的T生成函数：

```
double fuc(double x, int y) {
  double x;
  ……
}
```

# 例程

下面看几个例程。

## 1. 函数模板

```
#include <iostream>
#include <string>

using namespace std;

template <typename T> 
inline T const& Max (T const& a, T const& b) {
  return  a < b ? b : a;
}

int main() {
  int i = 39; 
  int j = 20; 
  cout << "Max(i, j): " << Max(i, j) << endl; 
  
  double f1 = 13.5;
  double f2 = 20.7; 
  cout << "Max(f1, f2): " << Max(f1, f2) << endl; 
  
  string s1 = "Hello";
  string s2 = "World"; 
  cout << "Max(s1, s2): " << Max(s1, s2) << endl; 
  
  return 0; 
}
```

当上面的代码被编译和执行时，它会产生下列结果：

```
Max(i, j): 39
Max(f1, f2): 20.7
Max(s1, s2): World
```

## 2. 类模板

```
#include <iostream>
#include <vector>
#include <cstdlib>
#include <string>
#include <stdexcept>

using namespace std;

template <class T>
class Stack {
 private:
  vector<T> elems; // 元素
 public:
  void push(T const&); // 入栈
  
  void pop(); // 出栈
  
  T top() const; // 返回栈顶元素 
  
  bool empty() const {  // 如果为空则返回真
    return elems.empty(); 
  }
};

template <class T>
void Stack<T>::push(T const& elem) {  // 追加传入元素的副本
  elems.push_back(elem); 
}

template <class T>
void Stack<T>::pop() {
  if (elems.empty()) {
    throw out_of_range("Stack<>::pop(): empty stack");
  }  
  // 删除最后一个元素
  elems.pop_back(); 
}

template <class T>
T Stack<T>::top () const {
  if (elems.empty()) {
    throw out_of_range("Stack<>::top(): empty stack");
  }  
  // 返回最后一个元素的副本 
  return elems.back(); 
}

int main() {
  try {
    Stack<int> intStack; // int 类型的栈
    Stack<string> stringStack; // string 类型的栈 
    
    // 操作 int 类型的栈 
    intStack.push(7); 
    cout << intStack.top() <<endl; 
    
    // 操作 string 类型的栈 
    stringStack.push("hello"); 
    cout << stringStack.top() << std::endl; 
    stringStack.pop(); 
    stringStack.pop(); 
  } catch (exception const& ex) {
    cerr << "Exception: " << ex.what() << endl; 
    return -1; 
  }  
}
```

当上面的代码被编译和执行时，它会产生下列结果：

```
7
hello
Exception: Stack<>::pop(): empty stack
```
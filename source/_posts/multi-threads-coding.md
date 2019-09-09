---
title: C++中多线程的简单使用
date: 2019-09-09 21:26:12
categories:
- C++
---



在编程中，常会需要用多线程的方式去处理并行任务，或进行更巧妙的逻辑设计。庞大的多线程库衍生出很多种用法，需要慢慢探索，不过掌握入门级的使用还是很方便的。



# 线程的初始化



**常用**构造函数有以下两种：

- `thread() noexcept;` 默认构造函数，创建一个空的`std::thread` 执行对象
- `template <class Fn, class... Args>
  explicit thread(Fn&& fn, Args&&... args);`初始化构造函数，创建一个 `std::thread` 对象，该 `std::thread` 对象可被 `joinable`，新产生的线程会调用 `fn` 函数，该函数的参数由 `args` 给出



# 互斥锁mutex



C++11中新增了&lt;mutex&gt;，它是C++标准程序库中的一个头文件，定义了C++11标准中的一些互斥访问的类与方法等。

- `std::mutex`：是C++11中最基本的互斥量，`std::mutex`对象提供了独占所有权的特性，不支持递归地对`std::mutex`对象上锁
- `std::timed_mutex`：该类表示定时互斥锁，不能递归使用
- `std::recursive_mutex`：该类表示递归互斥锁。递归互斥锁可以被同一个线程多次加锁，以获得对互斥锁对象的多层所有权。例如，同一个线程多个函数访问临界区时都可以各自加锁，执行后各自解锁。`std::recursive_mutex`释放互斥量时需要调用与该锁层次深度相同次数的`unlock()`，即`lock()`次数和`unlock()`次数相同。可见，线程申请递归互斥锁时，如果该递归互斥锁已经被当前调用线程锁住，则不会产生死锁。此外，`std::recursive_mutex`的功能与 `std::mutex`大致相同
- `std::recursive_timed_mutex`：带定时的递归互斥锁



# 线程启动

下面以`std::mutex`为例介绍 C++11 中的互斥量用法。



`std::mutex`的成员函数：

1. 构造函数，`std::mutex`不允许拷贝构造，也不允许 move 拷贝，最初产生的 mutex 对象是处于 unlocked 状态的

2. `lock()`，调用线程将锁住该互斥量。线程调用该函数会发生下面 3 种情况：

   a. 如果该互斥量当前没有被锁住，则调用线程将该互斥量锁住，直到调用 unlock之前，该线程一直拥有该锁。

   b. 如果当前互斥量被其他线程锁住，则当前的调用线程被阻塞住。

   c. 如果当前互斥量被当前调用线程锁住，则会产生死锁 (deadlock) 。

3. `unlock()`，解锁，释放对互斥量的所有权。

4. `try_lock()`，尝试锁住互斥量，如果互斥量被其他线程占有，则当前线程也不会被阻塞。线程调用该函数也会出现下面 3 种情况：

   a. 如果当前互斥量没有被其他线程占有，则该线程锁住互斥量，直到该线程调用 unlock 释放互斥量。

   b. 如果当前互斥量被其他线程锁住，则当前调用线程返回 false，而并不会被阻塞掉。

   c. 如果当前互斥量被当前调用线程锁住，则会产生死锁 (deadlock) 。



**Example Code**：

```c++
using namespace std;

mutex g_mtx;

void fun() {
  if (g_mtx.try_lock()) {
    cout << "thread" << endl;
    g_mtx.unlock();
  }
}

int main() {
  thread threads[10];
  for (int i = 0; i < 10; ++i) {
    threads[i] = thread(fun);
  }

  for (auto & th : threads) {
    th.join();
  }
  
  system("pause");
  return 0;
}
```



```c++
using namespace std;

recursive_mutex g_mutex;

void threadfun1() {
  cout << "enter threadfun1" << endl;
  lock_guard lock(g_mutex);
  cout << "execute threadfun1" << endl;
}

void threadfun2() {
  cout << "enter threadfun2" << endl;
  lock_guard lock(g_mutex);
  threadfun1();
  cout << "execute threadfun2" << endl;
}

int main() {
  threadfun2(); //死锁
  return 0;
}
```


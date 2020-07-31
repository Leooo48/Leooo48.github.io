---
title: libstdc++.so.6 version `GLIBCXX3.4.22' not found
date: 2020-07-31 14:33:00
categories: 
- 问题与解决
---

```
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
sudo add-apt-repository ppa:ubuntu-toolchain-r/test 
sudo apt-get update
sudo apt-get install libstdc++6
strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
```

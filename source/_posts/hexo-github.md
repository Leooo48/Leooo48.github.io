---
title: hexo+github搭建个人博客并实现多端更新
date: 2017-07-12 10:46:16
categories: 
- Ubuntu
---

本教程使用Hexo在ubuntu 14.04下创建博客，并实现win10 x64同步更新。

### Step 1. 创建博客

在ubuntu 14.04系统创建博客的Hexo教程很多，号称30分钟搭建个人博客。实际过程中遇到无数的坑，无数次卸载重装。最终的解决方案参考[ZL的博客](https://github.com/MasterIzumi/tricks/blob/master/blog_on_github.md)。

#### 1. 安装Nodejs

```
sudo apt-get install nodejs
sudo apt-get install nodejs-legacy
sudo apt-get install npm
```

以上命令按顺序装，否则会出错。

#### 2. 安装Hexo

```
sudo npm install hexo-cli -g
```

#### 3. 初始化博客文件夹

```
mkdir blog
cd blog
sudo hexo init
```

#### 4. 生成并查看博客。主题是默认主题，若一切正常，浏览器输入127.0.0.1即可显示HelloWorld的博客。

```
sudo hexo g # generate webpage
sudo hexo s # run local server
```

每次修改完之后可以使用` sudo hexo s `指令进行本地查看。

### Step 2. 挑选博客主题

Hexo具有丰富的主题可供选择，如Next主题：

```
sudo hexo clean
cd themes
git clone https://github.com/iissnan/hexo-theme-next.git
```

主题配置通过修改主题文件夹里面_config.yml文件，然后更新博客：

```
sudo hexo g # generate webpage
sudo hexo s # run local server
```

主题配置参考[NexT：http://theme-next.iissnan.com/getting-started.html](http://theme-next.iissnan.com/getting-started.html)。

### Step 3. 与GitHub链接

#### 1. 安装hexo与git的必要文件

```
cd blog
sudo npm install hexo-deployer-git --save
```

#### 2. 在GitHub网站新建空仓库，命名为yourname.github.io，yourname是GitHub帐户名。

#### 3. 编辑博客文件夹目录下的_config.yml文件，注意每个冒号“：”后面均有一个空格：

```
deploy:
  type: git
  repository: https://github.com/yourname/yourname.github.io.git # 此处大小写敏感，需注意
  branch:master
```

#### 4. 重新更新博客：

```
sudo hexo clean
sudo hexo g
sudo hexo d
```

部署完成后，打开网页进行查看，网址是yourname.github.io。


### Step 4. 在windows中配置多端更新


教程参考[Righere的博客：https://righere.github.io/2016/10/10/install-hexo/](https://righere.github.io/2016/10/10/install-hexo/)。

#### 1. 上传博客配置文件到GitHub

如果我们登录GitHub就会发现，hexo默认push的只是必要的网页文件，而我们需要把所有的本地文件都上传，才能在另一台计算机进行同步更博。此部分教程参考[monkey lzl的博客：http://blog.csdn.net/Monkey_LZL/article/details/60870891](http://blog.csdn.net/Monkey_LZL/article/details/60870891)。

在博客的原始文件夹（ubuntu 14.04系统）中**删去git clone下来的主题文件夹里面的.git文件夹！**，然后新建hexo分支，用来储存博客源文件。

```
cd blog
git init
git add -A
git commit -m "all blog source files"
git branch hexo
git checkout hexo
git remote add origin git@github.com:yourname/yourname.github.io.git
git push origin hexo -f
```

#### 2. 在win 10系统，安装Git x64

[点此下载 Git x64](https://github-production-release-asset-2e65be.s3.amazonaws.com/23216272/ed753c7e-5a85-11e7-9106-d484e52df854?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAIWNJYAX4CSVEH53A%2F20170712%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20170712T034345Z&X-Amz-Expires=300&X-Amz-Signature=28dfdad9dcc187544a3221a3ae8e0a6391eaf624c47f9cb49065e5dda567ee0e&X-Amz-SignedHeaders=host&actor_id=16009946&response-content-disposition=attachment%3B%20filename%3DGit-2.13.2-64-bit.exe&response-content-type=application%2Foctet-stream)

一路next即可。

#### 3. 安装Nodejs

[点此下载 Nodejs v6.11.1 x64](https://nodejs.org/dist/v6.11.1/node-v6.11.1-x64.msi)

一路next即可，默认装在C盘。我尝试过装在非系统盘，结果悲剧了一上午，各种报错。

#### 4. 克隆博客源文件

克隆之前，首先要将当前系统的ssh公钥添加到GitHub的列表中。如果当前系统未生成过公钥，则运行`ssh-keygen -t rsa`，然后将`.ssh`文件夹下的`.pub`文件中的公钥添加到GitHub的列表中。

```
git clone -b hexo git@github.com:yourname/yourname.github.io.git
cd yourname.github.io
```

#### 5. 安装Hexo

```
npm install hexo-cli -g
npm install
```

### Step 5. 更新博客的流程

```
cd blog
git pull
hexo new post "new blog name"
git add -A
git commit -m "XXX"
git push origin hexo
hexo g
hexo d
```

> 配置hexo折腾了大概一天半的时间，不过配好之后的使用体验和博客美观程度相当好，比之前博客好太多了，还是很值得的。最重要的是一定要按照步骤一步步来，千万不要自作聪明，否则会踩很多坑。
#Nullmax点云标注工具-使用手册
[TOC]
##I. 配置使用环境及安装
- `配置要求：ubuntu16.04 + ROS Kinetic`
### 1. **安装ROS-Kinetic**
参考[ROS WiKi-安装说明](http://http://wiki.ros.org/kinetic/Installation/Ubuntu)
```添加ROS源：
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
添加ROS源秘钥：
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
更新源
sudo apt-get update```
```
安装ROS完整版：（由于使用Rviz，PCL等模块，请务必安装完整版）
sudo apt-get install ros-kinetic-desktop-full
sudo apt-cache search ros-kinetic
初始化ROS：
sudo rosdep init
rosdep update
添加环境变量
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
更新ROS环境变量
source /opt/ros/kinetic/setup.bash```
```
测试ROS是否成功安装:
开启一个新的Teminnal，输入：
roscore
测试Rviz
开启一个新的Teminnal，输入：
rviz```
成功显示Rviz界面如下：
![1.png](/home/halo/Pictures/nullmax_tool/s1.png "sensor position" "width:720px")
### 2. **安装 Nullmax Tools 标注工具**
```
(1) 解压 nullmax_tool.tar.gz, 进入文件夹nullmax_tool
(2) 在nullmax_tool下开启终端，运行安装命令： sh install.sh
(3) 显示 install successful 后，home文件夹下出现lidar_annotation文件夹，安装成功
```

##II. 导入pcd文件
1. **导入待标注点云pcd文件**
```
Copy 待标注的点云.pcd格式文件到 lidar_annotation/pcd/ 文件夹下
```

2. **开始标注**
```
打开 Teminnal, 运行: sh run.sh
```
显示标注界面如下：
![2.png](/home/halo/Pictures/nullmax_tool/s2.png "sensor position" "width:1080px")

##III. 标注手册正篇
- ==`首次使用请务必仔细阅读`==
### 1. 标注面板详解
`下面就上图中 A, B, C, D, E 5个模块做详细说明：`

 - [ ]**A. 标注菜单栏 **
```
标注菜单栏由 [文件]， [编辑]，[视图]，[标记]，[选择] 5部分组成
文件：(1)切换新文件，(2)清除当前帧标记，(3)保存
编辑：(1)取消，(2)恢复
视图：(1)增加点的尺寸，(2)减小点的尺寸，(3)重置点的尺寸
标记：(1)清除当前物体的标记，(2)切换颜色，(3)设置BBox遮挡系数，(4)调节BBox方位，(5)调节BBox尺寸
选择：(1)跳转至下一物体，(2)跳转至上一物体
```
```
特别说明：
1.切换新文件会自动保存当前文件的标注信息
2.取消/恢复开销较大，尽量避免使用
3.标记完成一个物体后，需要切换到下一个物体进行标注，否则会覆盖当前标记；选择新的颜色会自动切换到下一物体；物体ID显示在面板上
4.标记BBox时，颜色 1～5，6~10，11~15，16~20 分别对应标签： 小车，大车，行人，骑行；标记其他非BBox时，颜色与标签无关；
5.标记BBOX时，需要设置方位角和遮挡系数，请以实际为准标注，0--不遮挡，1--完全遮挡
尽量使用简洁的方式完成标注，熟练使用快捷键可以有效提高标注速度。
```
 - [ ]**B. 快速选择栏 **
 ```
快速选择栏第一行，标注类型： BBox，地面，路沿，车道线
第二行：操作类型：移动，标记，删除，选择(无效)
第三行：选择方式：点，矩形(浅)------矩形框内部一定深度的点被选中，矩形(深)------矩形框内部所有深度的点被选中，多短线------多边形选择，效果和矩形(浅)相同
特别说明：
路沿和车道线标注仅支持点标注。
快速选择栏每个按钮都映射快捷键，例如： 标注地面 -- F2
```
 - [ ]**C. 快捷键对照表**
|快捷键|功能|快捷键|功能|
|:--------:|:------------:|:--------:|:------------:|
|Ctrl+Shift+O|打开新的一帧|Shift+O|增加点云大小|
|Ctrl+Shift+K|保存当前标记|O|减小点云大小|
|U|取消|Ctrl+Shift+C|清除当前标记的物体|
|Shift+U|恢复|Shift+C|下一颜色|
|Shift+N|下一物体|Shift+Z|上一颜色|
|Shift+P|上一物体|T，G|调节遮挡系数|
|R，F|调节方位|W，S，A，D，Q，E|调节BBox大小|
### 2. 标注教程
`首次使用请务必观看标注教程` 
[https://pan.baidu.com/s/1qrHMToZx7mK5linGElsmJQ] 密码: jajm
为保证清晰度，请下载播放 
```
特别说明
1.点云被重复标记为 BBox，路沿，车道线，地面时，标签优先级为 BBOX > 路沿/车道线 > 地面
```
`标注工具使用过程中如果遇见问题，或者代码部分有疑问，编辑需要，联系杜文文(18355180339 / wwdu@nullmax.ai)`



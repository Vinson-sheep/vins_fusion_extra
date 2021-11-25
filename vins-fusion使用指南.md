# vins-fusion使用指南

*Vinson Sheep 2021.11.25*

以**ubuntu 18.04/ros melodic**为例 

[参考地址1](https://blog.csdn.net/qq_42800654/article/details/109393646)

[参考地址2](https://blog.csdn.net/weixin_53660567/article/details/120254512)

## 第一步 安装realsense SDK

1. 使用源码安装

https://github.com/IntelRealSense/librealsense/

```
cd librealsense
```

2. 安装依赖

```
sudo apt-get install libudev-dev pkg-config libgtk-3-dev libusb-1.0-0-dev pkg-config libglfw3-dev libssl-dev
```

3. 编译安装.

```
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
make
sudo make install 
```

4. 测试sdk是否成功安装

```
realsense-viewer
```

![img](https://img-blog.csdnimg.cn/20210913081411948.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBATlVEVOS4gOaemueglOeptueUnw==,size_20,color_FFFFFF,t_70,g_se,x_16)



## 第二步 安装ROS Wrapper

1. 安装

```
cd ~/catkin_ws/src
git clone https://github.com/intel-ros/realsense.git
cd ..
catkin_make
// 记得修改~/.bashrc
```

catkin_make可能报错缺少`ddynamic_reconfigure`

```
sudo apt install ros-melodic-ddynamic-reconfigure
```



## 第三步 使用D435i 运行VINS-FUSION

1. 修改realsense文件

在`~/catkin_ws/src/realsense/realsense2_camera/launch/`新建`rs_camera_vins.launch`，内容[如下](https://github.com/Lu-tju/SLAM_file/blob/main/vins_rs/launch/rs_camera_vins.launch)

2. 修改VINS文件

在[此处](https://github.com/Lu-tju/SLAM_file/tree/main/vins_rs/realsense_d435i)下载d435i内外参文件，替换掉`~/catkin_ws/src/VINS-Fusion/config/realsense_d435i`目录下三个同名文件。

修改显示image-track（默认 0 ，没有发布image-track）

```
show_track = 1
```

3. 运行

```
roslaunch realsense2_camera rs_camera_vins.launch 
roslaunch vins vins_rviz.launch 
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

![img](https://img-blog.csdnimg.cn/20201031115556340.png)



## 第四步 坐标转换

下载[vins_fusion_extra](https://github.com/Vinson-sheep/vins_fusion_extra)

```
cd ~/catkin_ws/src
git clone https://github.com/Vinson-sheep/vins_fusion_extra.git
cd ..
catkin_make
```

启动坐标转换结点

```
roslaunch vins_fusion_extra pose_transform.launch
```

查看话题

```
rostopic list
```

列表中多了两个话题`/RPY`和`/mavros/vision_pose/pose`，移动D435i观察话题信息变化，确保信息正确后即可投入使用。


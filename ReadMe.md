# IMU

目前代码是**第二版**，后期还会在这一版的代码中更新，代码分为两部分。

* <font color=red>**在运行代码前**,查看代码*NewtonParase.cc* 文件，注意第31行</font>

  ~~~C++
  const char *proj4_text = "+proj=utm +zone=50 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
  ~~~

  长春是51  所以默认设为51 如果换了一个地方，<font color=Blue>**必须修改**</font>

* **安装依赖** 

  由于GNSS本身返回的数据是经纬度信息，而autoware信息本身是采用**UTM**坐标的方式，所以需要转换成UTM的坐标，安装如下依赖

  ~~~shell
  sudo apt-get install sqlite3
  ~~~

  还有在proj上的依赖

  ~~~html
  https://proj.org/install.html
  ~~~

  安装方法网站上有，我使用的是源码进行安装，CMake安装方式，也推荐这种安装方式。下载的是[7.1.0](https://proj.org/download.html#current-release)

  

* imu_driver为IMU在ROS环境下的驱动，运行方法如下：

~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
cp <imu_driver_code> .
cd ..
catkin_make
. ./devel/setup.bash
sudo chmod 777 /dev/ttyACM0
rosrun imu_driver IMU
~~~

**<imu_driver_code>** 代表的是文件夹imu_driver里的代码。

* data_record为数据记录的文件，主要是把IMU的数据存储下来，方便离线的分析，目前由于代码是第一版，第二版的代码要根据第一版存储下来的结果进行离线分析来继续第二版。dataRecord.cpp与dataRecord.hpp是数据存储的源代码，真正在数据存储的时候只需要运行如下代码即可

  ~~~
  sudo ./dataRecord
  ~~~

  

## 常见错误

* /dev/ttyACM0: No such file or directory :如果在还没开始运行代码的时候报错，代表IMU的数据线没有插上。 
* 测试发现还是没有best pose信号，可能是没有接入RTK的原因。后续接入RTK的时候需要重新采集一次信息。
* 如果有问题，后续接着补充

## 未来的工作
* 如果要将IMU放置在传感器融合里面，如IMU预积分，就需要得到IMU的方差的值出来，但是我们的iMU本身并不能输出方差的值，但是有开源的方案可以参考：
>> https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
>> https://github.com/rpng/kalibr_allan

* 目前加速度（由重力加速度验证）和UTM坐标位置（由谷歌地图验证）已经确定是没有问题的，对于heading还有角速度还需要想办法验证一下。
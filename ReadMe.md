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
* fail to read the port ttyACM0, please check the port and start the driver again：如果出现了这个错误，这个错误很麻烦，说实话我也没有想明白原因是什么，应该是我ros消息发布的队列消息的size选择不正确导致的。减小队列消息的长度之后我这边测试没有出这个问题了。但如果长久运行出现这个问题，麻烦通知我一下。Newton-M2与工控的连接方式是**ttyACM**连接，ttyUSB**X**/ttyACM**X**中的X代表的意思是电脑**第几个检测**到的设备，0代表第一个检测到，1代表第二个检测到。。。以此类推。在开发套件里面是**ttyACM0**,一般出现这个情况是连接口跳转到了**ttyACM1** 为什么会跳转我也没想明白
* 如果有问题，后续接着补充

## 未来的工作
如果要将IMU放置在传感器融合里面，如IMU预积分，就需要得到IMU的方差的值出来，但是我们的iMU本身并不能输出方差的值，但是有开源的方案可以参考：
>> https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
>> https://github.com/rpng/kalibr_allan


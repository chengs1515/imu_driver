# IMU

目前代码是**第一版**，后期还会在这一版的代码中更新，代码分为两部分。

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
* fail to read the port ttyACM0, please check the port and start the driver again：如果出现了这个错误，这个错误很麻烦，说实话我也没有想明白原因是什么，应该是我ros消息发布的队列消息的size选择不正确导致的。减小队列消息的长度之后我这边测试没有出这个问题了。但如果长久运行出现这个问题，麻烦通知我一下。
* 如果有问题，后续接着补充


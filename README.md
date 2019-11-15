# Robot_SLAM_Navigation
1031 demo version

一、 概述

所有代码均需在ROS环境下运行。运行前需设置地面站和上位机的分布式主从通信（网线直连），注意设置和更改两台处理器的IP地址。

二、 环境配置

**Ubuntu and ROS**
Our software is developed in Ubuntu 16.04. ROS Kinetic. ROS can be installed here: [ROS Installation](http://wiki.ros.org/ROS/Installation)

**Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

**Mosek and OOQP**

We use **Mosek** for conic programming. To use mosek, you should request a free **Personal Academic License** [here](https://www.mosek.com/products/academic-licenses/). Then create a folder named 'mosek' in your home directory and put your license in it. All header and library files are already included in this repo, so you don't need to download mosek again. 

We use **OOQP** for quadratic programming. 

1. Get a copy of **MA27** from the [HSL Archive](http://www.hsl.rl.ac.uk/download/MA27/1.0.0/a/). Just select the **Personal Licence (allows use without redistribution)**, then fill the information table. You can download it from an e-mail sent to you. Then, un-zip **MA27**, and follow the *README* in it, install it to your Ubuntu.

**If you are new to Ubuntu, or too lazy to follow its *README*, see here, just type 3 commands in MA27's folder :**
```
./configure
make 
sudo make install
```

2. Manually un-zip packages *OOQP.zip* in the **installation** folder of this repo and install it follow the document *INSTALL* in **OOQP**, install it to your Ubuntu.

**As above, you can just type 3 commands in OOQP's folder :**
```
./configure
make 
sudo make install
```

**NOTE**: Compile MA27, you will get a static library file named libma27.a in its /src folder. Then when you compile OOQP, the original OOQP would search the libma27.a file in its current top folder. However, in this repo, I modify OOQP's *configure* file to let it search libma27.a in your ubuntu system. So:

**Case1** - If you download OOQP by yourself (from OOQP's website), you have to copy and paste libma27.a into OOQP's folder before you compile OOQP, otherwise you would find a compile error.

**Case2** - If you use OOQP from this repo, just follow the above commands without any other considerations. 

**1.3**   **some tools**

To install the following dependencies, you can run the auto-install script by 
```
  ./install_tools.sh
```

Then run
```
  ./config_gcc.sh
```
to finish the configuration

Or, you can manually install them one by one:
```
  sudo apt-get install ros-kinetic-joy
  sudo apt-get install libnlopt-dev
  sudo apt-get install libf2c2-dev
  sudo apt-get install libarmadillo-dev 
  sudo apt-get install glpk-utils libglpk-dev
  sudo apt-get install libcdd-dev

  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt-get update
  sudo apt-get install gcc-7 g++-7
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-7
```

The simulator requires C++17, which needs **gcc 7** to compile. When you catkin_make, the simulator would automatically select gcc 7 as its compiler, but wouldn't change your default compiler (gcc 4.8/ gcc 5). 

三、 源码模块

源码主要分为四个部分：

建图算法：基于深度图像，可以实时构建3D稠密地图的系统

定位算法：基于摄像头、IMU数据的实时定位系统。

导航算法：能够根据全局地图实现全局规划路径的生成，根据局部地图进行临时障碍物的检测和规避。

小车通信接口：进行里程计与IMU信号的硬同步，接收里程计的输出数据，给小车下发实时速度指令。

四个部分中，建图算法和定位算法的结果是导航算法所需的输入数据。小车发送的里程计数据是定位算法的输入数据。导航算法通过小车通信接口向小车下发轮子速度指令。

四、 源码的部署

1. 地面站：

执行实时建图算法和可视化。

需拷贝建图算法和导航算法两个文件夹。

通信设置脚本为：net_setup_gs.sh

启动脚本为：ground_station.sh

2. 上位机：

执行定位算法、导航算法，以及和小车的数据及命令通信。

需拷贝定位算法、导航算法、小车通信三个文件夹。

通信设置脚本为：net_setup.sh

控制器启动脚本为:controller.sh

启动定位导航程序：start.sh

启动传感器（相机、IMU、轮子）：launch_sensors.sh

五、 程序操作

实时建图默认定位导航程序启动点为导航起点，

在控制器界面，按下键盘‘m’ 键，设定导航终点，

按下手柄上的‘start’ 键，进行地图优化和路径规划

上一步完成后，按下键盘‘n’键，小车开始自动运行。

六、 其他提示

1. 如果脚本运行出错，可能是部分指令运行需要较长时间，尚未完成即开始执行下一指令。建议尝试将指令分开运行观察运行效果。

2. 为保证定位算法稳定运行，请务必先启动定位程序，再启动传感器。

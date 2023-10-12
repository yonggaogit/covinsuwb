# Multi-UAV Visual Inertial Odometry with UWB
## 1 Introduction
- The construction of this project aims to integrate the ranging information of UWB with centralized multi-UAV collaborative visual positioning to alleviate the cumulative drift of visual positioning
- This project builds on the following projects, thanks to [ETH Zurich V4RL Lab](https://asl.ethz.ch/v4rl.html) and [Zhejiang University's Fast-Lab](http://zju-fast.com/):
    - [COVINS](https://github.com/VIS4ROB-lab/covins)
    - [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
    - [Fast-Drone-250](https://github.com/yonggaogit/Fast-Drone-250)
    - [Nlink-Parser](https://github.com/nooploop-dev/nlink_parser)


##  2 Environment Prepare
### 2.1 ROS environment
- ros noetic
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
- catkin tools
```shell
sudo apt-get install python3-catkin-tools
sudo apt-get install python3-wstool
```
### 2.2 code clone
```shell
mkdir -p ~/project/covinsuwb_ws/src
cd ~/project/covinsuwb_ws
catkin init
catkin config --extend /opt/ros/noetic/
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
cd ~/project/covinsuwb_ws/src
git clone https://github.com/yonggaogit/covinsuwb.git
```
### 2.3 Third-party toolkits
- complile serial
```shell
cd ~/project/covinsuwb_ws/src/covinsuwb/3rd_party/serial_module
make
make test
sudo make install
```

- compile glog
```shell
cd ~/project/covinsuwb_ws/src/covinsuwb/3rd_party/glog_module
sudo chmod 777 autogen.sh
sudo chmod 777 configure
./autogen.sh && ./configure && make && sudo make install
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
```

- compile ceres
```shell
mkdir build
cd build
cmake ..
sudo make -j4
sudo make install
sudo apt-get install ros-noetic-ddynamic-reconfigure
```

### 2.3 Real-Flight Environment 
- If you want to run the real machine, please refer to the configuration of the [Fast-Drone-250](https://github.com/yonggaogit/Fast-Drone-250), the hardware configuration is basically the same, and the software needs to install realsense drivers and mavros

## 3 Compile & Build
- First Time you download
```shell
cd ~/project/covinsuwb_ws
./src/covinsuwb/install_file.sh 8
```

- When you modify the code of some module
```shell
# if you modify the covins_backend module
catkin build covins_backend

# if you modify the covins_frontend module
catkin build covins_frontend

# if you modify the vins module
catkin build vins
```

## 4 Usage
### 4.1 Dataset Experiment
- change the covins_comm/config/config_comm.yaml(sys.server_ip:127.0.0.1)
```shell
# open a new terminator
roscore

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/simulate_shfiles/rviz.sh
./src/covinsuwb/simulate_shfiles/rviz.sh

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/simulate_shfiles/backend.sh
./src/covinsuwb/simulate_shfiles/backend.sh

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/simulate_shfiles/frontend.sh
./src/covinsuwb/simulate_shfiles/frontend.sh 0

# open a new terminator, ..., determite by how many agent you want to run
cd ~/project/covinsuwb_ws
./src/covinsuwb/simulate_shfiles/frontend.sh 1
```
- the vins configuration wait to publish, please wait....
### 4.2 Real Experiment
- change the covins_comm/config/config_comm.yaml(sys.server_ip:the ip of your server agent)
```shell
# open a new terminator
roscore

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/real_shfiles/rviz.sh
./src/covinsuwb/real_shfiles/rviz.sh

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/real_shfiles/backend.sh
./src/covinsuwb/real_shfiles/backend.sh

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/real_shfiles/frontend.sh
./src/covinsuwb/real_shfiles/frontend.sh 0

# open a new terminator
cd ~/project/covinsuwb_ws
sudo chmod 777 ./src/covinsuwb/real_shfiles/uwb.sh
./src/covinsuwb/real_shfiles/uwb.sh 0
```
- the vins configuration wait to publish, please wait....
## 5 Dataset
### 5.1 EuROC
- The is the original [EuROC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset, but the dataset is single UAV dataset.You need merge some data and simulate uwb range data, contact me.
- usage to be continue
### 5.2 S3E
- [S3E](https://github.com/PengYu-Team/S3E) 
- usage to be continue

## 6 Evaluation
- install [evo tools](https://github.com/MichaelGrupp/evo)
- the output localization result file in the directory (covinsuwb/covins_backend/output/KF_x_xxx.csv)
    - you can change the option `sys.trajectory_format` in the file named `config_backend.yaml`(covinsuwb/covins_backend/config) to change the output file type(ftum or euroc).
- for multi uav localization result, you need use cat to merge all the csv file, and use the evo instruction
    ```shell
    evo_ape euroc KF_0_ftum.csv gt_data.csv -vas
    ```
## 7 Result
- to be continue
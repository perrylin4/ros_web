# ROS网页遥控器系统使用指南

## 项目概述
通过rosbridge实现Web与ROS2通信的三指触控遥控系统，支持多点触控操作，适配移动设备。

## 核心功能
- **/cmd_vel** (geometry_msgs/Twist)：控制机器人线速度和角速度
- **/height** (std_msgs/Float32)：控制机器人高度（0.0-1.0）

## 安装配置
### 1. 安装依赖
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
sudo apt install ros-humble-rosbridge-server
```

### 2.修改连接配置
在index.html中约590行处修改ROS bridge地址：
```javascript
const ip = 'ws://YOUR_HOST_IP:' // 改为主机IP
const port = '9090';             // Rosbridge端口
```

### 3.启动
```bash
chmod +x launch.sh
bash ./launch.sh
```

### 4.操作说明
支持3指同时操作，左摇杆控制相对于机器方向的移动方向，右摇杆仅左右有用，控制转向速度，高度表控制机器人身体高度
> 测试环境：ROS humble + python3.10
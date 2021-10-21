# 目录

- [目录](#目录)
- [项目介绍](#项目介绍)
  - [使用方法](#使用方法)
    - [获取配置文件](#获取配置文件)
    - [修改节点](#修改节点)
    - [订阅图像topic](#订阅图像topic)

# 项目介绍

运行环境：Nvidia axvier + Ubuntu 18.04(arm64) + ROS1 melodic

基于LBAS库修改的GigE相机节点

## 使用方法

### 获取配置文件

运行节点前需运行```ParametrizeCamera_LoadAndSave.cpp```获取配置文件```FeatureFile.ini```，修改```/src/node.cpp```内的地址信息

```cpp
    nRet = MV_CC_FeatureLoad(handle, "/home/bit/ugv_camera/src/lbas_camera_driver/FeatureFile.ini");
```

### 修改节点

获取相机和本机的IP地址（注意要设置为同一网段），修改```/src/node.cpp```内的节点信息

```cpp
    unsigned char camera_IP[4] = {192, 168, 1, 201};
    unsigned char local_IP[4] = {192, 168, 1, 100};
```

### 订阅图像topic

topic名称为```/camera/image_color```

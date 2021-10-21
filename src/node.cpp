/*
 *@author:Cloude Remnant
 *@date:2021-10-09
 *@description:
*/

/* Include**********************************************/
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include "lbas_camera_driver/MvCameraControl.h"

/* Macro Definition*************************************/

/* Global Variable**************************************/
int nRet = MV_OK;
void *handle = NULL;
/* Function*********************************************/
void cameraLoadSettings(void)
{
    unsigned int nIp;
    unsigned char camera_IP[4] = {192, 168, 1, 201};
    unsigned char local_IP[4] = {192, 168, 1, 100};
    MV_CC_DEVICE_INFO stDevInfo = {0};
    MV_GIGE_DEVICE_INFO stGigEDev = {0};

    nIp = (camera_IP[0] << 24) | (camera_IP[1] << 16) | (camera_IP[2] << 8) | camera_IP[3];
    stGigEDev.nCurrentIp = nIp;

    nIp = (local_IP[0] << 24) | (local_IP[1] << 16) | (local_IP[2] << 8) | local_IP[3];
    stGigEDev.nNetExport = nIp;

    stDevInfo.nTLayerType = MV_GIGE_DEVICE; // ch:仅支持GigE相机 | en:Only support GigE camera
    stDevInfo.SpecialInfo.stGigEInfo = stGigEDev;

    while (1)
    {
        sleep(1);
        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, &stDevInfo);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Create Handle fail! nRet [0x%x]\n", nRet);
            continue;
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Open Device fail! nRet [0x%x]\n", nRet);
            continue;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
            if (nRet != MV_OK)
                ROS_WARN("Set Packet Size fail nRet [0x%x]!\n", nRet);
        }
        else
            ROS_WARN("Get Packet Size fail nRet [0x%x]!\n", nPacketSize);

        // ch:从文件中导入相机属性 | en:Import the camera properties from the file
        nRet = MV_CC_FeatureLoad(handle, "/home/bit/ugv_camera/src/lbas_camera_driver/FeatureFile.ini");
        if (MV_OK != nRet)
        {
            ROS_ERROR("Load Feature fail! nRet [0x%x]\n", nRet);
            continue;
        }
        else
        {
            ROS_INFO("Finish import the camera properties from the file\n");
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lbas_camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_color", 1);
    ros::Rate loop_rate(5);

    unsigned char *pData = NULL;
    unsigned char *pDataForBGR = NULL;

    sleep(5); // 防止摄像头未连接

    cameraLoadSettings();

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
        ROS_ERROR("Get PayloadSize fail! nRet [0x%x]\n", nRet);

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
        ROS_ERROR("MV_CC_StartGrabbing fail! nRet [0x%x]\n", nRet);

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);

    unsigned int nDataSize = stParam.nCurValue;

    while (ros::ok())
    {
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 200);
        // 处理图像
        // image processing
        pDataForBGR = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);

        // 像素格式转换
        // convert pixel format
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
        // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
        // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
        // destination pixel format, output data buffer, provided output buffer size
        stConvertParam.nWidth = stImageInfo.nWidth;
        stConvertParam.nHeight = stImageInfo.nHeight;
        stConvertParam.pSrcData = pData;
        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = pDataForBGR;
        stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;

        if (nRet == MV_OK)
        {
            nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
            if (MV_OK != nRet)
                ROS_ERROR("MV_CC_ConvertPixelType fail! nRet [0x%x]\n", nRet);
            cv::Mat img(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

            pub.publish(msg);
        }
        loop_rate.sleep();
    }
    // 停止取流
    // stop grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
        ROS_ERROR("MV_CC_StopGrabbing fail! nRet [0x%x]\n", nRet);

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
        ROS_ERROR("MV_CC_CloseDevice fail! nRet [0x%x]\n", nRet);

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
        ROS_ERROR("MV_CC_DestroyHandle fail! nRet [0x%x]\n", nRet);

    return 0;
}

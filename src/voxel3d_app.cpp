/**
 @file      voxel3d_app.cpp
 @brief     5HiRab ToF camera example
 @author    Yushan Chen
 @copyright Copyright (c) 2025 5Voxel Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>             /* getopt_long() */
#include <errno.h>
#include <iostream>
#ifdef PLAT_WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#endif /* PLAT_WINDOWS */

#include "opencv2/core/version.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "voxel3d.h"
#include "voxel3d_vd55h1_info.h"

#define TOOLS_VER_MAJOR         (1)
#define TOOLS_VER_MINOR         (13)

#ifdef PLAT_WINDOWS
#define M_PI                    (3.141592653589793f)
#endif /* PLAT_WINDOWS */

#ifdef PLAT_WINDOWS
#define SleepSeconds(x)        Sleep(x * 1000)
#else /* PLAT_LINUX */
#define SleepSeconds(x)        sleep(x)
#endif /* PLAT_WINDOWS */

CamDevInfo      camInfo;
static int      var1 = 0, var2 = 0;
static int      conf_threshold = 5;
static bool     found_tof_device = false, found_rgb_device = false, found_flir_device = false;
static int      iWaitKey = 0;
static int      display_imu = 0;
static int      display_temperature = 0;
static int      display_rgb = 1;

static int      auto_exposure_mode = 1;
static int      confidence_threshold = 1;
static int      max_confidence_threshold = 1;
static int      min_confidence_threshold = 1;
static int      median_filter_enable = 1;
static int      bilateral_filter_enable = 1;
static int      kalman_filter_enable = 1;
static int      adaptive_conf_filter_enable = 1;

static int      m_doRectify = RectifyType::NONE;

static int      track_mouse_position = 1;
static int      mouse_x = 0, mouse_y = 0;

static bool     m_undistort = false;

using namespace std;
using namespace cv;

static unsigned char fw_upgrade_cb(int state, unsigned int percent_complete)
{
    switch (state) {
    case 0:
        printf("\n\rFW upgrade - state: Initial             ");
        break;
    case 1:
        printf("\n\rFW upgrade - state: Downloading (%d%%)", percent_complete);
        break;
    case 2:
        printf("\n\rFW upgrade - state: Complete            ");
        break;
    default:
        printf("\n\rFW upgrade - state: Error               ");
        break;
    }

    return (1);
}

void ToFCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        track_mouse_position = 0;
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        track_mouse_position = 1;
    }
    else if (event == EVENT_MBUTTONDOWN)
    {
    }
    else if (event == EVENT_MOUSEMOVE)
    {
        if (track_mouse_position) {
            mouse_x = x;
            mouse_y = y;
        }
    }
}

static void mainloop(char* dev_sn)
{
    int ret = 0;
    int tof_width = 0, tof_height = 0;
    int rgb_width = 0, rgb_height = 0;
    int flir_width = 0, flir_height = 0;
    unsigned int tof_frame_count = 0;

    Mat depth, depth_tmp, conf, pointCloudXYZ;
    Mat colorMap, conf8u, depth8U;
    Mat rgb, rectify_rgb;
    Mat rectify_flir, flir, flir8U;
    IMU_DATA imu_data = { 0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

    cv::Mat operateMat = cv::Mat::zeros(340, 400, CV_8U);
    cv::putText(operateMat, "Press", cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'Esc' to exit program", cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " '+/-' to change confidence threshold", cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'a' to switch AE mode", cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'A' to switch Adaptive Conf mode", cv::Point(10, 120), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'b' to switch Bilateral filter mode", cv::Point(10, 140), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'e' to show current exposure time", cv::Point(10, 160), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'E' to change exposure time", cv::Point(10, 180), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'i' to switch IMU data display", cv::Point(10, 200), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'k' to switch Kalman filter mode", cv::Point(10, 220), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'm' to switch Median filter mode", cv::Point(10, 240), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'r' to switch RGB display", cv::Point(10, 260), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 't' to switch temperature display", cv::Point(10, 280), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    cv::putText(operateMat, " 'u' to switch RGB-D fusion", cv::Point(10, 300), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    if (found_flir_device) {
        cv::putText(operateMat, " 'v' to switch Thermal-D fusion", cv::Point(10, 320), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    }
    cv::imshow("operate", operateMat);

    if (found_tof_device) {
        tof_width = voxel3d_tof_get_width(dev_sn);
        tof_height = voxel3d_tof_get_height(dev_sn);
        depth = cv::Mat(tof_height, tof_width, CV_16UC1, cv::Scalar(0));
        depth_tmp = cv::Mat(tof_height, tof_width, CV_16UC1, cv::Scalar(0));
        conf = cv::Mat(tof_height, tof_width, CV_16UC1, cv::Scalar(0));
        rectify_rgb = cv::Mat(tof_height, tof_width, CV_8UC3, cv::Scalar(0, 0, 0));
        pointCloudXYZ = cv::Mat(tof_height, tof_width, CV_32FC3, cv::Scalar(0, 0, 0));

        ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_AUTO_EXPOSURE, &var1, &var2);
        if (ret > 0) {
            auto_exposure_mode = var1;
        }
        ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_CONF_THRESHOLD, &var1, &var2);
        if (ret > 0) {
            confidence_threshold = var1;
        }
        ret = voxel3d_tof_get_caps_max(dev_sn, TOF_CAPS_CONF_THRESHOLD, &var1, &var2);
        if (ret > 0) {
            max_confidence_threshold = var1;
        }
        ret = voxel3d_tof_get_caps_min(dev_sn, TOF_CAPS_CONF_THRESHOLD, &var1, &var2);
        if (ret > 0) {
            min_confidence_threshold = var1;
        }
        ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_MEDIAN, &var1, &var2);
        if (ret > 0) {
            median_filter_enable = var1;
        }
        ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_BILATERAL, &var1, &var2);
        if (ret > 0) {
            bilateral_filter_enable = var1;
        }
        ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_KALMAN, &var1, &var2);
        if (ret > 0) {
            kalman_filter_enable = var1;
        }
        ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_ADAPTIVE_CONF, &var1, &var2);
        if (ret > 0) {
            adaptive_conf_filter_enable = var1;
        }
    }
    if (found_rgb_device) {
        rgb_width = voxel3d_rgb_get_width(dev_sn);
        rgb_height = voxel3d_rgb_get_height(dev_sn);
#ifdef PLAT_LINUX /* for imdecode from mjpg to RGB888 */
       rgb = cv::Mat(1, rgb_height * rgb_width * 3, CV_8UC1, cv::Scalar(0));
#else /* PLAT_WINDOWS */
       rgb = cv::Mat(rgb_height, rgb_width, CV_8UC3, cv::Scalar(0, 0, 0));
#endif
    }
    if (found_flir_device) {
        flir_width = voxel3d_lepton3_get_width(dev_sn);
        flir_height = voxel3d_lepton3_get_height(dev_sn);
        if (found_tof_device) {
            rectify_flir = cv::Mat(tof_height, tof_width, CV_32FC1, cv::Scalar(0));

        } else {
            rectify_flir = cv::Mat(flir_height, flir_width, CV_32FC1, cv::Scalar(0));
        }
        flir = cv::Mat(flir_height, flir_width, CV_32FC1, cv::Scalar(0));
    }

    iWaitKey = 0;

    while (iWaitKey != 27) {
        switch (iWaitKey) {
            case '+':
            {
                int result;
                conf_threshold = (conf_threshold >= max_confidence_threshold) ? max_confidence_threshold : conf_threshold + 1;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_CONF_THRESHOLD, conf_threshold, 0);
                if (result > 0) {
                    printf("Set confidence threshold to %d (maximum = %d)\n", conf_threshold, max_confidence_threshold);
                }
                else {
                    printf("Failed to set confidence threshold (%d)\n", result);
                    conf_threshold--;
                }
                break;
            }
            case '-':
            {
                int result;
                conf_threshold = (conf_threshold <= min_confidence_threshold) ? min_confidence_threshold : conf_threshold - 1;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_CONF_THRESHOLD, conf_threshold, 0);
                if (result > 0) {
                    printf("Set confidence threshold to %d (minimum = %d)\n", conf_threshold, min_confidence_threshold);
                }
                else {
                    printf("Failed to set confidence threshold (%d)\n", result);
                    conf_threshold++;
                }
                break;
            }
            case 'a':
            {
                int result;
                auto_exposure_mode ^= 1;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_AUTO_EXPOSURE, auto_exposure_mode, 0);
                if (result > 0) {
                    printf("Set Auto Exposure Mode : %d\n", auto_exposure_mode);
                }
                break;
            }
            case 'A':
            {
                int result;
                adaptive_conf_filter_enable ^= 1;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_ADAPTIVE_CONF, adaptive_conf_filter_enable, 0);
                if (result > 0) {
                    printf("Set Adaptive Conf mode : %d\n", adaptive_conf_filter_enable);
                }
                break;
            }
            {
                int result;
                auto_exposure_mode ^= 1;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_AUTO_EXPOSURE, auto_exposure_mode, 0);
                if (result > 0) {
                    printf("Set Auto Exposure Mode : %d\n", auto_exposure_mode);
                }
                break;
            }
            case 'b':
            {
                int var1, var2;

                ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_BILATERAL, &var1, &var2);
                if (ret > 0) {
                    var1 ^= 1;
                    ret = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_BILATERAL, var1, var2);
                    if (ret > 0) {
                        bilateral_filter_enable = var1;
                        printf("Bilateral filter is %s\n", bilateral_filter_enable ? "enabled" : "disabled");
                    }
                    else {
                        printf("Failed to set Bilateral filter\n");
                    }
                }
                else {
                    printf("Failed to get current value of Bilateral filter variables\n");
                }
                break;
            }
            case 'c':
            {
                m_undistort ^= 1;
                break;
            }
            case 'e':
            {
                int result;
                result = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_EXPOSURE_TIME, &var1, &var2);
                if (result > 0) {
                    printf("Current Exposure Time (Freq1: %d, Freq2: %d)\n", var1, var2);
                }
                else {
                    printf("Failed to get current exposure time values (%d)\n", result);
                }
                break;
            }
            case 'E':
            {
                int result;
                int target_expo = 0;
                cout << "\nEnter exposure time (us): ";
                cin >> target_expo;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_EXPOSURE_TIME, target_expo, target_expo);
                if (result > 0) {
                    printf("Set Exposure Time to %d\n", target_expo);
                }
                else {
                    printf("Failed to set exposure time values (%d)\n", result);
                }
                break;
            }
            case 'f':
            {
                int result;
                printf("Enter 'f'\n");
                result = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_FRAME_RATE, &var1, &var2);
                if (result > 0) {
                    printf("Current Frame Rate: %d\n", var1);
                }
                else {
                    printf("Failed to get current frame rate (%d)\n", result);
                }
                break;
            }
            case 'F':
            {
                int result;
                int target_fps = 0;
                cout << "\nEnter frame rate: ";
                cin >> target_fps;
                result = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_FRAME_RATE, target_fps, 0);
                if (result > 0) {
                    printf("Set Frame Rate to %d\n", target_fps);
                }
                else {
                    printf("Failed to set frame rate (%d)\n", result);
                }
                break;
            }
            case 'i':
                display_imu ^= 1;
                break;
            case 'I':
            {
                unsigned short reg_addr;
                unsigned char rw_type;
                int data_len = 1, ret = 0;
                unsigned char rw_data[8];

                memset(rw_data, 0x0, sizeof(rw_data));
                cout << "Enter 1 (read) or 2 (write): ";
                cin >> rw_type;
                cout << "\nEnter register address (Hex): ";
                cin >> hex >> reg_addr;
                cout << "\nEnter length: ";
                cin >> data_len;
                switch (rw_type) {
                case '1':
                    ret = voxel3d_tof_sensor_i2c_read(dev_sn, reg_addr, data_len, (char *)rw_data);
                    if (ret > 0) {
                        printf("\n");
                        for (int ix = data_len - 1; ix >= 0; ix--) {
                            printf("0x%02X ", rw_data[ix]);
                        }
                        printf("\n");
                    }
                    else {
                        printf("sensor i2c read error (%d)\n", ret);
                    }
                    break;
                case '2':
                    printf("\nLSB first");
                    for (int ix = 0; ix < data_len; ix++) {
                        printf("\nEnter byte-%d (Hex): ", ix);
                        scanf("%02hhX", &rw_data[ix]);
                    }
                    ret = voxel3d_tof_sensor_i2c_write(dev_sn, reg_addr, data_len, (char *)rw_data);
                    if (ret > 0) {
                        printf("sensor i2c write successfully\n");
                    }
                    else {
                        printf("sensor i2c write error (%d)\n", ret);
                    }
                    break;
                default:
                    printf("wrong input %c (0x%02X)\n", rw_type, rw_type);
                }
                break;
            }
            case 'k':
            {
                int var1, var2;

                ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_KALMAN, &var1, &var2);
                if (ret > 0) {
                    var1 ^= 1;
                    ret = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_KALMAN, var1, var2);
                    if (ret > 0) {
                        kalman_filter_enable = var1;
                        printf("Kalman filter is %s\n", kalman_filter_enable ? "enabled" : "disabled");
                    }
                    else {
                        printf("Failed to set Kalman filter\n");
                    }
                }
                else {
                    printf("Failed to get current value of Kalman filter variables (err: %d)\n", ret);
                }
                break;
            }
            case 'm':
            {
                int var1, var2;

                ret = voxel3d_tof_get_caps_curr(dev_sn, TOF_CAPS_MEDIAN, &var1, &var2);
                if (ret > 0) {
                    var1 ^= 1;
                    ret = voxel3d_tof_set_caps(dev_sn, TOF_CAPS_MEDIAN, var1, var2);
                    if (ret > 0) {
                        median_filter_enable = var1;
                        printf("Median filter is %s\n", median_filter_enable ? "enabled" : "disabled");
                    }
                    else {
                        printf("Failed to set Median filter\n");
                    }
                }
                else {
                    printf("Failed to get current value of Median filter variables (err: %d)\n", ret);
                }
                break;
            }
            case 't':
                display_temperature ^= 1;
                break;
            case 'r':
                display_rgb ^= 1;
                break;
            case 'u':
            {
                if (m_doRectify == RectifyType::TOF2RGB) {
                    m_doRectify = RectifyType::NONE;
                    std::cout << "Disable TOF-RGB fusion" << std::endl;
                }
                else {
                    m_doRectify = RectifyType::TOF2RGB;
                    std::cout << "Enable TOF-RGB fusion" << std::endl;
                }
                voxel3d_set_rectifyType(dev_sn, m_doRectify);
                break;
            }
            case 'v':
            {
                if (m_doRectify == RectifyType::FLIR2TOF) {
                    m_doRectify = RectifyType::NONE;
                    std::cout << "Disable Thermal-D fusion" << std::endl;
                }
                else {
                    m_doRectify = RectifyType::FLIR2TOF;
                    std::cout << "Enable Thermal-D fusion" << std::endl;
                }
                voxel3d_set_rectifyType(dev_sn, m_doRectify);

                break;
            }
        }

        if (found_tof_device) {
            unsigned int ret = voxel3d_tof_queryframe(dev_sn, depth.ptr<unsigned short>(0), conf.ptr<unsigned short>(0), m_undistort);
            if (ret) {
                tof_frame_count = ret;
                voxel3d_tof_generatePointCloud(
                    dev_sn,
                    depth.ptr<unsigned short>(0),
                    pointCloudXYZ.ptr<float>(0));

                conf.convertTo(conf8u, CV_8UC1, 255.0 / 1024.f);
                putText(conf8u, format("%d", conf.at<unsigned short>(mouse_y, mouse_x)), Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));
                imshow("IR", conf8u);

                for (int ix = 0; ix < tof_width * tof_height; ix++) {
                    depth_tmp.at<unsigned short>(ix) = depth.at<unsigned short>(ix) % 1024;
                }
                depth_tmp.convertTo(depth8U, CV_8UC1, 255.0 / 1024);
                applyColorMap(depth8U, colorMap, COLORMAP_JET);
                putText(colorMap, format("(%.3f, %.3f, %.3f)", pointCloudXYZ.at<Vec3f>(mouse_y, mouse_x)[0],
                    pointCloudXYZ.at<Vec3f>(mouse_y, mouse_x)[1], pointCloudXYZ.at<Vec3f>(mouse_y, mouse_x)[2]),
                    Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));
                imshow("Depth", colorMap);
                if (found_rgb_device)
                {
#ifdef PLAT_LINUX
                    Mat decodedRGB;
                    if (m_doRectify == RectifyType::RGB2TOF)
                    {
                        /*
                         * Not supported. Suggest to use TOF2RGB for RGB-D fusion
                         */
                    }
                    else {
                        ret = voxel3d_rgb_queryframe(dev_sn, rgb.ptr<uchar>(0));
                        if (ret > 0) {
                            decodedRGB = cv::imdecode(rgb, cv::IMREAD_COLOR);
                            if (!decodedRGB.empty()) {
                                cv::resize(decodedRGB, rectify_rgb, rectify_rgb.size());
                                if (display_rgb) {
                                    putText(rectify_rgb, format("%d, %d, %d", rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(0),
                                            rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(1), rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(2)),
                                            Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));
                                    imshow("RGB", rectify_rgb);
                                }
                            }
                        }
                    }
#else /* PLAT_WINDOWS */
                    if (m_doRectify == RectifyType::RGB2TOF)
                    {
                        ret = voxel3d_rgb_queryframe(dev_sn, rectify_rgb.ptr<uchar>(0));
                        
                    }
                    else {
                        ret = voxel3d_rgb_queryframe(dev_sn, rgb.ptr<uchar>(0));
                        cv::resize(rgb, rectify_rgb, rectify_rgb.size());
                    }
                    putText(rectify_rgb, format("%d, %d, %d", rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(0), rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(1), rectify_rgb.at<cv::Vec3b>(mouse_y, mouse_x)(2)), Point(mouse_x, mouse_y), 1, 1, Scalar(255, 255, 255));

                    if (display_rgb) {
                        imshow("RGB", rectify_rgb);
                    }
#endif
                }

                if (found_flir_device) {
                    if (m_doRectify == RectifyType::FLIR2TOF)
                    {
                        ret = voxel3d_lepton3_queryframe(dev_sn, rectify_flir.ptr<float>(0));                        
                    }
                    else
                    {
                        ret = voxel3d_lepton3_queryframe(dev_sn, flir.ptr<float>(0));
                        if (ret > 0) {
                            cv::resize(flir, rectify_flir, rectify_flir.size());
                        }
                    }

                    if (ret > 0) {
                        rectify_flir.convertTo(flir8U, CV_8UC1, 255.0 / 40.0);
                        putText(flir8U, format("%f", rectify_flir.at<float>(mouse_y, mouse_x)), Point(mouse_x, mouse_y), 1, 1, Scalar(0, 0, 0));
                        imshow("Thermal", flir8U);
                    }
                }

                if (display_imu) {
                    if (voxel3d_read_imu_data(dev_sn, &imu_data)) {
                        printf("frame #%d: IMU TS = %d, ACC (%.4f, %.4f, %.4f), GYRO (%.4f, %.4f, %.4f)\n",
                            tof_frame_count, 
                            imu_data.imu_ts, imu_data.imu_accel[0], imu_data.imu_accel[1], imu_data.imu_accel[2],
                            imu_data.imu_gyro[0], imu_data.imu_gyro[1], imu_data.imu_gyro[2]);
                    }
                }

                if (display_temperature) {
                    if (tof_frame_count % 30 == 0) {
                        printf("Sensor temperature: %.1f, Laser temperature: %.1f\n",
                            voxel3d_tof_get_sensor_temperature(dev_sn),
                            voxel3d_tof_get_illum_temperature(dev_sn));
                    }
                }
            }
        }

        //set the callback function for any mouse event
        setMouseCallback("Depth", ToFCallBackFunc, NULL);

        iWaitKey = waitKey(5);
    }
    return;
}

static void usage(FILE *fp, int argc, char **argv)
{
    fprintf(fp,
         "Usage: %s [options]\n\n"
         "Version %d.%d\n"
         "Options:\n"
         "-h | --help             Print this message\n"
         "-C | --calib_save       Save calibration data to bin file [5VSTDON only]\n"
         "-c | --camera_info      Show camera info\n"
         "-i | --show_info        Show device info\n"
         "-S | --scan_dev         Scan devices and list device S/N\n"
         "-s | --dev_sn           Specify device S/N to access\n"
         "-t | --temperature      Display camera temperature\n"
         "-u | --fw_upgrade       Device firmware upgrade\n"
         "-U | --use_case         Switch use cases [5VSTDON only]\n"
         "-v | --version          Show lib & firmware version\n"
         "\n",
         argv[0], TOOLS_VER_MAJOR, TOOLS_VER_MINOR);
}

static const char short_options[] = "hCciSs:tu:U:v";

static const struct option
long_options[] = {
    { "help",              no_argument,       NULL, 'h' },
    { "calib_save",        no_argument,       NULL, 'C' },
    { "camera_info",       no_argument,       NULL, 'c' },
    { "show_info",         no_argument,       NULL, 'i' },
    { "scan_dev",          no_argument,       NULL, 'S' },
    { "dev_sn",            required_argument, NULL, 's' },
    { "temperature",       no_argument,       NULL, 't' },
    { "fw_upgrade",        required_argument, NULL, 'u' },
    { "use_case",          required_argument, NULL, 'U' },
    { "version",           no_argument,       NULL, 'v' },
    { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    char data[64];
    int  result;
    float fResult;
    char dev_sn[MAX_PRODUCT_SN_LEN] = {'\0'};
    CamInitSettings tof_default_setting = { 0, 0, VIDEO_MODE_YUY2 }; //use default format

    
    for (;;) {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                        short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
        case 0:
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

        case 'C':
        {
            int result;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                int read_len = 8192; /* EEPROM size */
                FILE* pFile;

                pFile = fopen("EEPROM.bin", "wb");
                if (pFile == NULL) {
                    printf("Failed to open file\n");
                    exit(-1);
                }

                unsigned char* p_raw = (unsigned char*)malloc(read_len);
                if (!p_raw) {
                    printf("no memory for calibration RAW\n");
                    fclose(pFile);
                    exit(-1);
                }

                result = voxel3d_tof_read_calibration_raw(dev_sn, (char*)p_raw, read_len);
                if (result > 0) {
                    fwrite(p_raw, 1, read_len, pFile);
                    printf("File saved!\n");
                }
                fclose(pFile);
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);
        }
        case 'c':
        {
            int result;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                CameraInfo caminfo;
                memset(&caminfo, 0x0, sizeof(caminfo));
                voxel3d_tof_read_camera_info(dev_sn, &caminfo);
                printf("Camera Info (ToF) : \n");
                printf("  Fx = %.5f\n", caminfo.focalLengthFx);
                printf("  Fy = %.5f\n", caminfo.focalLengthFy);
                printf("  Cx = %.5f\n", caminfo.principalPointCx);
                printf("  Cy = %.5f\n", caminfo.principalPointCy);
                printf("  K1 = %.25f\n", caminfo.K1);
                printf("  K2 = %.25f\n", caminfo.K2);
                printf("  P1 = %.25f\n", caminfo.P1);
                printf("  P2 = %.25f\n", caminfo.P2);
                printf("  K3 = %.25f\n", caminfo.K3);
                printf("  K4 = %.25f\n", caminfo.K4);
                printf("  K5 = %.25f\n", caminfo.K5);
                printf("  K6 = %.25f\n\n", caminfo.K6);
            }

            result = voxel3d_rgb_init(dev_sn);
            if (result > 0) {
                CameraInfo caminfo;
                memset(&caminfo, 0x0, sizeof(caminfo));
                voxel3d_rgb_read_camera_info(dev_sn, &caminfo);
                printf("Camera Info (RGB) : \n");
                printf("  Fx = %.5f\n", caminfo.focalLengthFx);
                printf("  Fy = %.5f\n", caminfo.focalLengthFy);
                printf("  Cx = %.5f\n", caminfo.principalPointCx);
                printf("  Cy = %.5f\n", caminfo.principalPointCy);
                printf("  K1 = %.25f\n", caminfo.K1);
                printf("  K2 = %.25f\n", caminfo.K2);
                printf("  P1 = %.25f\n", caminfo.P1);
                printf("  P2 = %.25f\n", caminfo.P2);
                printf("  K3 = %.25f\n", caminfo.K3);
                printf("  K4 = %.25f\n", caminfo.K4);
                printf("  K5 = %.25f\n", caminfo.K5);
                printf("  K6 = %.25f\n\n", caminfo.K6);
            }
            voxel3d_tof_release(dev_sn);
            voxel3d_rgb_release(dev_sn);
            exit(EXIT_SUCCESS);
        }
        case 'i':
        {
            int result;
            float vfov = 0, hfov = 0;
            int width = 0, height = 0;
            unsigned int mod_freq0 = 0, mod_freq1 = 0;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                unsigned char sensor_info[250];
                unsigned char* p_sensor_info = sensor_info;

                width = voxel3d_tof_get_width(dev_sn);
                printf("\nToF:\n");
                printf("\tDefault Frame width : %d\n", width);
                height = voxel3d_tof_get_height(dev_sn);
                printf("\tDefault frame height : %d\n", height);
                hfov = voxel3d_tof_get_depth_hfov(dev_sn);
                printf("\tHFoV : %.2f (%.1f degree)\n", hfov, hfov * 180 / M_PI);
                vfov = voxel3d_tof_get_depth_vfov(dev_sn);
                printf("\tVFoV : %.2f (%.1f degree)\n", vfov, vfov * 180 / M_PI);
                voxel3d_get_mod_freq(dev_sn, &mod_freq0, &mod_freq1);
                printf("\tMod Freq 0 : %d (MHz)\n", mod_freq0 / 1000000);
                printf("\tMod Freq 1 : %d (MHz)\n\n", mod_freq1 / 1000000);

                result = voxel3d_tof_get_debug_info(dev_sn, (char *)sensor_info, sizeof(sensor_info));

                if (result > 0) {
                    printf("ToF debug info (5VSTDON Only):\n");
                    while ((p_sensor_info - sensor_info) < (int)sizeof(sensor_info)) {
                        unsigned short reg_addr;
                        unsigned char reg_len;
                        string reg_name;
                        reg_addr = (p_sensor_info[0]) | ((p_sensor_info[1]) << 8);
                        reg_len = p_sensor_info[2];
                        p_sensor_info += 3;

                        if (reg_addr == 0 && reg_len == 0) {
                            break;
                        }

                        reg_name = get_name_by_reg_addr(reg_addr);
                        printf("\tReg 0x%04X (%s) = ", reg_addr, reg_name.c_str());
                        for (int ix = reg_len - 1; ix >= 0; ix--) {
                            printf("%02X ", p_sensor_info[ix]);
                        }

                        switch (reg_addr) {
                        case VD55H1_REG_ADDR_SYSTEM_FSM:
                        {
                            string sys_fsm_str = get_name_by_system_fsm_code(*p_sensor_info);
                            printf("(%s)", sys_fsm_str.c_str());
                            break;
                        }
                        case VD55H1_REG_ADDR_BOOT_FSM:
                        {
                            string boot_fsm_str = get_name_by_boot_fsm_code(*p_sensor_info);
                            printf("(%s)", boot_fsm_str.c_str());
                            break;
                        }
                        case VD55H1_REG_ADDR_ERROR_CODE:
                        case VD55H1_REG_ADDR_WARNING_CODE:
                        {
                            unsigned short err_code = *p_sensor_info | (*(p_sensor_info + 1) << 8);
                            string err_str = get_name_by_err_code(err_code);
                            printf("(%s)", err_str.c_str());
                            break;
                        }
                        }
                        printf("\n");
                        p_sensor_info += reg_len;
                    }
                }
                printf("\n");
            }

            result = voxel3d_rgb_init(dev_sn);
            if (result > 0) {
                printf("RGB:\n");
                width = voxel3d_rgb_get_width(dev_sn);
                printf("\tDefault frame width : %d\n", width);
                height = voxel3d_rgb_get_height(dev_sn);
                printf("\tDefault frame height : %d\n\n", height);
            }

            result = voxel3d_lepton3_init(dev_sn);
            if (result > 0) {
                char part_num[32];
                int shutter_mode;
                printf("Thermal:\n");
                voxel3d_lepton3_get_pn_shutter_mode(dev_sn, part_num, &shutter_mode);
                printf("\tP/N : %s\n", part_num);
                printf("\tShutter : %s\n", shutter_mode == 0 ? "manual" :
                    shutter_mode == 1 ? "auto" :
                    shutter_mode == 2 ? "external" : "unknown");
            }
            voxel3d_tof_release(dev_sn);
            voxel3d_rgb_release(dev_sn);
            voxel3d_lepton3_release(dev_sn);
            exit(EXIT_SUCCESS);

            break;
        }

        case 's':
            strncpy(dev_sn, optarg, MAX_PRODUCT_SN_LEN);
            break;

        case 'S':
            voxel3d_scan(&camInfo);
            if (camInfo.num_of_devices > 0) {
                char dev_name[MAX_DEV_NAME_LEN];

                printf("\nFound %d devices\n", camInfo.num_of_devices);
                for (int ix = 0; ix < camInfo.num_of_devices; ix++) {

                    printf("\n%d: Name = %s, SN = %s\n", ix, camInfo.dev_info[ix].dev_name, camInfo.dev_info[ix].product_sn);
                    printf("    Available frame format(s): %d\n", camInfo.dev_info[ix].frame_fmts.avail_frame_num);
                    for (int iy = 0; iy < camInfo.dev_info[ix].frame_fmts.avail_frame_num; iy++) {
                        printf("        %d: width = %d, height = %d, fps = %.1f, format = %s %s\n",
                            iy,
                            camInfo.dev_info[ix].frame_fmts.fmt[iy].width,
                            camInfo.dev_info[ix].frame_fmts.fmt[iy].height,
                            camInfo.dev_info[ix].frame_fmts.fmt[iy].fps,
                            camInfo.dev_info[ix].frame_fmts.fmt[iy].fmt == VIDEO_MODE_YUY2 ? "YUY2" : "MJPG",
                            camInfo.dev_info[ix].frame_fmts.default_frame_index == iy ? "[Default]" : "");

                    }

                    if (camInfo.dev_info[ix].caps.total_caps_num > 0 &&
                        camInfo.dev_info[ix].caps.total_caps_num <= MAX_SUPPORTED_CAPABILITIES) {
                        printf("    Available capabilities: %d\n", camInfo.dev_info[ix].caps.total_caps_num);
                        result = voxel3d_tof_init(dev_sn, tof_default_setting);
                        if (result > 0) {
                            int caps_num = voxel3d_tof_get_total_caps_num(dev_sn);
                            for (int iy = 0; iy < caps_num; iy++) {
                                int max1 = 0, max2 = 0, min1 = 0, min2 = 0, curr1 = 0, curr2 = 0;

                                int cap_code = voxel3d_tof_get_caps_code(dev_sn, iy);
                                int ret = voxel3d_tof_get_caps_name(dev_sn, cap_code, dev_name, sizeof(dev_name));
                                if (ret >= 0) {
                                    voxel3d_tof_get_caps_max(dev_sn, cap_code, &max1, &max2);
                                    voxel3d_tof_get_caps_min(dev_sn, cap_code, &min1, &min2);
                                    voxel3d_tof_get_caps_curr(dev_sn, cap_code, &curr1, &curr2);
                                    printf("        %d: cap code %d (%s)\n",
                                        iy,
                                        cap_code,
                                        dev_name);
                                    printf("            var1 (max: %d, curr: %d, min: %d), var2 (max: %d, curr: %d, min: %d)\n",
                                        max1, curr1, min1, max2, curr2, min2);
                                }
                                else {
                                    printf("        caps[%d]: ret = %d\n", iy, ret);
                                }
                            }
                        }
                        voxel3d_tof_release(dev_sn);
                    }
                }

                

                printf("\nEmbeded IMU: %s\n", camInfo.imu_info.is_exist ? "Exist" : "Absent");
                if (camInfo.imu_info.is_exist) {
                    printf("    Coordinate: %s\n", 
                        camInfo.imu_info.imu_coordinate == IMU_COORDINATE_RIGHT_HAND ? "Right-hand" :
                        camInfo.imu_info.imu_coordinate == IMU_COORDINATE_LEFT_HAND ? "Left-hand" : "Unknown");
                    printf("    Axis with Gravity: %s\n",
                        camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_X ? "X" :
                        camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_NEGATIVE_X ? "-X" :
                        camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_Y ? "Y" :
                        camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_NEGATIVE_Y ? "-Y" :
                        camInfo.imu_info.imu_axis_with_gravity == IMU_GRAVITY_ON_Z ? "Z" : "-Z"
                        );
                }
            }
            else {
                printf("Can't find any 5Voxel device\n");
            }
            exit(EXIT_SUCCESS);

        case 't':
        {
            int result;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                fResult = voxel3d_tof_get_sensor_temperature(dev_sn);
                if (fResult > -273.15f) {
                    printf("\nSensor temperature :       %f\n", fResult);
                }
                fResult = voxel3d_tof_get_illum_temperature(dev_sn);
                if (fResult > -273.15f) {
                    printf("Illumination temperature : %f\n", fResult);
                }
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);
        }
        case 'u':
        {
            int result;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                memset(data, 0x0, sizeof(data));
                voxel3d_read_fw_version(dev_sn, data, sizeof(data));
                printf("--------------------------------------------------------\n");
                printf("Before F/W upgrade\n");
                printf("F/W file     : %s\n", optarg);
                printf("F/W version  : %s\n", data);
                printf("--------------------------------------------------------\n\n");
                result = voxel3d_dev_fw_upgrade(dev_sn, optarg, fw_upgrade_cb);
                voxel3d_release(dev_sn);
                if (result < 0) {
                    printf("5HiRab FW upgrade failed (err: %d)\n", result);
                    exit(EXIT_SUCCESS);
                }

                printf("Firmware upgrade completed and camera reboots.\n");
                printf("Close opened cameras and rescan to continue...\n");
                voxel3d_release(dev_sn);
                exit(EXIT_SUCCESS);
            }
            break;
        }
        case 'U':
        {
            int result;
            int target_usecase = 3;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {

                target_usecase = atoi(optarg);

                printf("Use case #1 - 200MHz\n");
                printf("Use case #2 - 80MHz\n");
                printf("Use case #3 - 80MHz  + 60MHz\n");
                printf("Use case #4 - 80MHz  + 50MHz\n");
                printf("Use case #5 - 200MHz + 166MHz\n\n");
                printf("Current: %d\n", voxel3d_get_usecase(dev_sn));
                printf("Target : %d\n", target_usecase);
                result = voxel3d_set_usecase(dev_sn, target_usecase);
                if (result > 0) {
                    for (int ix = 0; ix < 15; ix++) {
                        printf(".");
                        SleepSeconds(1);
                    }
                    printf("\nSuccessfully set use case to %d\n", target_usecase);
                }
                else {
                    printf("\nFailed to set use case (err: %d)\n", result);
                }
            }
            voxel3d_tof_release(dev_sn);
            break;
        }
        case 'v':
        {
            int result;

            result = voxel3d_tof_init(dev_sn, tof_default_setting);
            if (result > 0) {
                int ret;
                SleepSeconds(1);
                memset(data, 0x0, sizeof(data));
                ret = voxel3d_read_lib_version(data, sizeof(data));
                if (ret < 0) {
                    printf("Share library version read failed (err: %d)\n", ret);
                }
                else {
                    printf("Share library version  : %s\n", data);
                }

                memset(data, 0x0, sizeof(data));
                ret = voxel3d_read_fw_version(dev_sn, data, sizeof(data));
                if (ret < 0) {
                    printf("Device F/W version read failed (err: %d)\n", ret);
                }
                else {
                    printf("Device F/W version    : %s\n", data);
                }

                memset(data, 0x0, sizeof(data));
                ret = voxel3d_read_fw_build_date(dev_sn, data, sizeof(data));
                if (ret < 0) {
                    printf("Device F/W build date read failed (err: %d)\n", ret);
                }
                else {
                    printf("Device F/W build date : %s\n", data);
                }
            }
            voxel3d_tof_release(dev_sn);
            exit(EXIT_SUCCESS);
        }
        default:
            usage(stderr, argc, argv);
            exit(EXIT_SUCCESS);
        }
    }

    /*
     * Start device
     */
    result = voxel3d_tof_init(dev_sn, tof_default_setting);
    if (result > 0) {
        found_tof_device = true;
    }

    result = voxel3d_rgb_init(dev_sn);
    if (result > 0) {
        found_rgb_device = true;
    }

    result = voxel3d_lepton3_init(dev_sn);
    if (result > 0) {
        found_flir_device = true;
    }

    m_doRectify = RectifyType::NONE;
    voxel3d_set_rectifyType(dev_sn, m_doRectify);

    /*
     * main loop function
     */
    if (found_tof_device || found_flir_device || found_rgb_device) {
        mainloop(dev_sn);
    }

    /*
     * Stop device
     */
    voxel3d_tof_release(dev_sn);
    voxel3d_rgb_release(dev_sn);
    voxel3d_lepton3_release(dev_sn);

    return (EXIT_SUCCESS);
}


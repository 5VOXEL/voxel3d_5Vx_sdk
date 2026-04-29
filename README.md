# voxel3d_5Vx_sdk
Library and utilities for working with 5Voxel 5VSTDON & 5VHiRab 3D-ToF Camera  
* For usage of voxel3d library
  1. Open voxel3d_tools/html/index.html in browser
  2. Click "Files", then "voxel3d.h" for detail info

-------------------------------------------------------------------------------
# Windows
Usage: voxel3d_tools.exe [options]  
  
Version 1.13  
Options:  
&emsp;-h | --help&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Print this message  
&emsp;-C | --calib_save&emsp;&emsp;&emsp;Save calibration data to bin file [5VSTDON only]  
&emsp;-c | --camera_info&emsp;&emsp;&nbsp;Show camera info  
&emsp;-i | --show_info&emsp;&emsp;&emsp;&emsp;Show device info  
&emsp;-S | --scan_dev&emsp;&emsp;&emsp;&nbsp;&nbsp;Scan devices and list device S/N  
&emsp;-s | --dev_sn&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;Specify device S/N to access  
&emsp;-t | --display&emsp;&emsp;&emsp;&emsp;&emsp;Display camera temperature  
&emsp;-u | --fw_upgrade&emsp;&emsp;&nbsp;Device firmware upgrade  
&emsp;-U | --use_case&emsp;&emsp;&emsp;&nbsp;&nbsp;Switch use cases [5VSTDON only]  
&emsp;-v | --version&emsp;&emsp;&emsp;&emsp;&nbsp;Show lib & firmware version  
  
  
Example:  
voxel3d_tools.exe
  
  
Supported Deivce(s)
-------------------------------------------------------------------------------
5Voxel 5VHiRab series  
1. 5VSTDON03  
2. 5VSTDON04  
3. 5VHiRab887CF60  
4. 5VHiRab887CF80  
5. 5VHiRab976F60  
6. 5VHiRab976F80  
7. 5VHiRabv1976F80  

Supported OS/Platform
-------------------------------------------------------------------------------
$ Windows 11 (Visual Studio Community 2022)  
$ Ubuntu 18.04 (Jetson Nano)  
  
  
Kernel module needed
-------------------------------------------------------------------------------
N/A	

Build steps (Windows 11)
-------------------------------------------------------------------------------
1. Install Vistual Studion Community 2022  
2. Go to platform/win and click on 'voxel3d_tools.sln' (Visual Studio Solution File)  
3. Build x64 release project  
4. Open a command window and go to following directory  
        platform/win/Bin/x64-Release/voxel3d_tools  
5. Execute 'voxel3d_tools.exe -h' to show menu  
6. Execute 'voxel3d_toosl.exe' to start camera streaming  

Build steps (Linux)
-------------------------------------------------------------------------------
Preparation  
- To rebuild example, libopencv-dev is required. If host doesn't have, do
  'sudo apt install libopencv-dev'  
  
Build steps  
1. Go to platform/linux  
2. Execute 'make clean;make'  
3. Find 'voxel3d_tools' executable file at same location (platform/linux)   
4. Execute 'voxel3d_tools -h' to show menu  
5. Execute 'voxel3d_toosl' to start camera streaming  

# UdCap 宇叠动作捕捉手套社区驱动
## 当前功能支持状态

 - [x] 枚举 USB 设备
 - [x] 设备探测
 - [x] 接收器状态管理
 - [x] 连接状态管理
 - [x] 松散架构，允许混合手套
 - [x] 数据标定
 - [x] 手指骨骼四元数自动计算
 - [x] 控制器震动
 - [x] 控制器按钮(A/B/Menu/Joy/Power)支持和虚拟按钮(Trigger/Grip/Trackpad)支持
 - [x] 配对
 - [x] 调节信道
 - [x] 关节微调
 - [x] 存取手套用户设置
 - [x] 接收器热插拔检测
 - [x] 拔出并重新插入接收器后自动恢复连接
 - [ ] 兼容其他不由默认 ASIO 驱动兼容的串口模式(如 Android)
 - [ ] OTA (暂时不打算支持)
 
## 介绍
第三方社区实现的宇叠动作捕捉手套驱动，理论上以原生方式支持 Windows macOS Linux 平台。以 C++ 17 实现，提供静态链接。

正常工作时，不会在探测后继续占用与其无关的串口。

核心内存占用约 `1.2M + 50k * N`，N 为连接的手套数量

## 编译和运行
### 依赖
 * CMake
 * Eigen
 * Boost
 * libusb
 * libusbp
 * hidapi

 注: 所有依赖会自动使用 [CPM](https://github.com/cpm-cmake/CPM.cmake) 下载，无需系统中存在。

### 编译
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

如果需要编译测试程序，请在 `cmake` 时添加 `-DBUILD_TEST_TOOLS=ON -DCOPY_HIDAPI_DLL=ON` 选项。

如果想要输出更详细的原始协议数据包，请在 `-DBUILD_TEST_TOOLS=ON` 的同时添加 `-DTEST_TOOLS_PRINT_RAW=ON` 选项。

### 测试
启用测试后会生成 `UdCapCommunityDriverCoreTest` 应用程序，该程序会自动枚举连接的接收器，并获取其序列号。连接手套后会自动开始标定过程，请跟随输出做出动作。
 
## 说明
宇叠为上海宇叠智能科技有限公司，UdCap 是其动作捕捉手套产品的名称。

其余库内容以 MIT 协议开源。
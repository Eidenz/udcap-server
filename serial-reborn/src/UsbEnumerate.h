//
// Created by max_3 on 2025/5/8.
//

#ifndef SERIALREBORN_USBENUMERATE_H
#define SERIALREBORN_USBENUMERATE_H

#include "SerialCommon.h"
#include <vector>
#include <functional>
#include <mutex>
#include <memory>
#include <utility>
#include <map>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>


enum UsbEnumerateRefreshType {
    USB_ENUMERATE_REFRESH_ALL = 0,
    USB_ENUMERATE_REFRESH_SERIAL = 1,
    USB_ENUMERATE_REFRESH_HID = 2
};

enum UsbHotPlugEventType {
    USB_HOT_PLUG_EVENT_ADD = 0,
    USB_HOT_PLUG_EVENT_REMOVE = 1
};

#ifdef WIN32

#endif
class UsbEnumerate {
private:
    void initHotPlug();
    std::vector<SerialDevice> devices;
    static std::shared_ptr<UsbEnumerate> _instance;
    static std::mutex _mtx;
    std::thread hotPlugThread;
    bool isHotPlugRunning = true;
    std::map<uint32_t, std::function<void(UsbHotPlugEventType)>> listenHotPlugCallbacks;
    std::recursive_mutex hotPlugMutex;
    std::atomic_uint32_t hotPlugFd = 0;
    std::thread eventLoopThread;
    std::atomic_bool eventLoopRunning = true;
    std::queue<UsbHotPlugEventType> packetQueue;
    std::mutex eventLoopMutex;
    std::condition_variable eventCondition;
public:
    UsbEnumerate(const UsbEnumerate&) = delete;
    UsbEnumerate& operator=(const UsbEnumerate&) = delete;
    void callHotPlugCallback(UsbHotPlugEventType);
    std::function<void()> listenHotPlug(const std::function<void(UsbHotPlugEventType)> &callback);
    static std::shared_ptr<UsbEnumerate> getInstance();
    void refresh(UsbEnumerateRefreshType type);
    void printDevices();
    std::vector<SerialDevice> findPorts(std::function<bool(const SerialDevice&)>);
    ~UsbEnumerate();
    UsbEnumerate();
};


#endif //SERIALREBORN_USBENUMERATE_H

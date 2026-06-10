//
// Created by max_3 on 2025/5/8.
//

#include <iostream>
#include <iomanip>
#include <hidapi.h>
#include <libusb.h>
#include <libusbp.hpp>
#include "UsbEnumerate.h"

#ifdef WIN32
#include <windows.h>
#undef min
#undef max
#include <dbt.h>
#endif
std::shared_ptr<UsbEnumerate> UsbEnumerate::_instance;
std::mutex UsbEnumerate::_mtx;
std::shared_ptr<UsbEnumerate> UsbEnumerate::getInstance() {
    if (!_instance) {
        std::lock_guard<std::mutex> lock(_mtx);
        if (_instance)
            return _instance;
        _instance = std::make_shared<UsbEnumerate>();
    }
    return _instance;
}

UsbEnumerate::UsbEnumerate() {
    libusb_init_context(nullptr, nullptr, 0);
    initHotPlug();
    std::thread t([this]() {
        while (eventLoopRunning.load()) {
            if (packetQueue.empty()) {
                std::unique_lock lk(eventLoopMutex);
                eventCondition.wait_for(lk, std::chrono::milliseconds(1000));
                continue;
            }
            std::lock_guard guard(hotPlugMutex);
            if (packetQueue.empty()) {
                continue;
            }
            UsbHotPlugEventType packet = packetQueue.front();
            packetQueue.pop();
            for (const auto &pair: listenHotPlugCallbacks) {
                pair.second(packet);
            }
        }
    });
    eventLoopThread = std::move(t);
}

UsbEnumerate::~UsbEnumerate() {
#ifdef WIN32
    isHotPlugRunning = false;
    if (hotPlugThread.joinable()) {
        hotPlugThread.join();
    }
#else
    libusb_hotplug_deregister_callback(nullptr, 0);
#endif
    libusb_exit(nullptr);
    this->eventLoopRunning.store(false);
    this->eventLoopThread.join();
}

#ifdef WIN32
LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
        case WM_DEVICECHANGE:
            if (wParam == DBT_DEVICEARRIVAL)
            {
                auto pHdr = (PDEV_BROADCAST_HDR)lParam;
                if (pHdr->dbch_devicetype == DBT_DEVTYP_PORT || pHdr->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
                {
                    UsbEnumerate::getInstance()->callHotPlugCallback(UsbHotPlugEventType::USB_HOT_PLUG_EVENT_ADD);
                }
            }
            else if (wParam == DBT_DEVICEREMOVECOMPLETE)
            {
                auto pHdr = (PDEV_BROADCAST_HDR)lParam;
                if (pHdr->dbch_devicetype == DBT_DEVTYP_PORT || pHdr->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
                {
                    UsbEnumerate::getInstance()->callHotPlugCallback(UsbHotPlugEventType::USB_HOT_PLUG_EVENT_REMOVE);
                }
            }
            break;
        case WM_QUIT:
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, message, wParam, lParam);
}
#else
int hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
                     libusb_hotplug_event event, void *user_data) {
  static libusb_device_handle *dev_handle = NULL;
  struct libusb_device_descriptor desc;
  int rc;

  (void)libusb_get_device_descriptor(dev, &desc);

  if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event) {
    rc = libusb_open(dev, &dev_handle);
    if (LIBUSB_SUCCESS != rc) {
        std::cerr << "Error opening device: " << libusb_error_name(rc) << std::endl;
    }
    UsbEnumerate::getInstance()->callHotPlugCallback(UsbHotPlugEventType::USB_HOT_PLUG_EVENT_ADD);
  } else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
    if (dev_handle) {
      libusb_close(dev_handle);
      dev_handle = NULL;
    }
    UsbEnumerate::getInstance()->callHotPlugCallback(UsbHotPlugEventType::USB_HOT_PLUG_EVENT_REMOVE);
  }
  return 0;
}
#endif

void UsbEnumerate::initHotPlug() {
#ifdef WIN32
    hotPlugThread = std::thread([this]() {
        HINSTANCE hInstance = GetModuleHandle(nullptr);
        const char CLASS_NAME[] = "USBMonitorClass";
        WNDCLASS wc = { };
        wc.lpfnWndProc = WndProc;
        wc.hInstance = hInstance;
        wc.lpszClassName = CLASS_NAME;
        RegisterClass(&wc);
        HWND hwnd = CreateWindowEx(
                0,
                CLASS_NAME,
                "USB Monitor",
                0,
                CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
                nullptr, nullptr, hInstance, nullptr
        );
        if (!hwnd)
        {
            std::cerr << "Failed to create window for USB monitoring." << std::endl;
            return;
        }
        MSG msg = { };
        while (isHotPlugRunning)
        {
            if (GetMessage(&msg, nullptr, 0, 0)) {
                if (msg.message == WM_QUIT || msg.message == WM_DESTROY) {
                    break;
                }
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        }
        DestroyWindow(hwnd);
        UnregisterClass(CLASS_NAME, hInstance);
    });
#else
    libusb_hotplug_callback_handle callback_handle;
    int rc;
    rc = libusb_hotplug_register_callback(nullptr, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
                                            LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, 0, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
                                            LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback, nullptr,
                                            &callback_handle);
    if (LIBUSB_SUCCESS != rc) {
        std::cerr << "Error registering hotplug callback: " << libusb_error_name(rc) << std::endl;
        return;
    }

    hotPlugThread = std::thread([this]() {
        while (isHotPlugRunning) {
            int rc = libusb_handle_events_completed(nullptr, nullptr);
            if (rc != LIBUSB_SUCCESS) {
                break;
            }
        }
    });
#endif
}

std::function<void()> UsbEnumerate::listenHotPlug(const std::function<void(UsbHotPlugEventType)> &callback) {
    std::lock_guard guard(hotPlugMutex);
    uint32_t fd = hotPlugFd.fetch_add(1);
    listenHotPlugCallbacks[fd] = callback;
    return [this, fd]() {
        std::lock_guard guard(hotPlugMutex);
        listenHotPlugCallbacks.erase(fd);
    };
}

void UsbEnumerate::callHotPlugCallback(UsbHotPlugEventType eventType) {
    std::lock_guard guard(hotPlugMutex);
    packetQueue.push(eventType);
    eventCondition.notify_all();
}

void UsbEnumerate::refresh(UsbEnumerateRefreshType type) {
    this->devices.clear();
    if (type == UsbEnumerateRefreshType::USB_ENUMERATE_REFRESH_ALL || type ==
        UsbEnumerateRefreshType::USB_ENUMERATE_REFRESH_SERIAL) {
        try {
            auto devices = libusbp::list_connected_devices();
            for (const libusbp::device &device: devices) {
                // Print the USB device info.
                uint16_t vendorId = device.get_vendor_id();
                uint16_t productId = device.get_product_id();
                try {
                    std::string serialNumber = "";
                    try {
                        serialNumber = device.get_serial_number();
                    } catch (const libusbp::error &) {
                        serialNumber = "";
                    }

                    bool success = false;
                    for (uint32_t i = 0; i < 255; i++) {
                        try {
                            libusbp::serial_port port(device, i, true);
                            std::string port_name = port.get_name();
                            SerialDevice compSerialDevice {};
                            compSerialDevice.isHid = false;
                            compSerialDevice.vid = vendorId;
                            compSerialDevice.pid = productId;
                            compSerialDevice.serialNumber = serialNumber;
                            compSerialDevice.interfaceNumber = i;
                            compSerialDevice.composite = true;
                            compSerialDevice.portName = port_name;
                            success = true;
                            this->devices.push_back(compSerialDevice);
                        } catch (const libusbp::error &error) {

                        }
                    }
                    if (!success) {
                        try {
                            libusbp::serial_port port(device, 0, false);
                            SerialDevice serialDevice {};
                            serialDevice.isHid = false;
                            serialDevice.vid = vendorId;
                            serialDevice.pid = productId;
                            serialDevice.serialNumber = serialNumber;
                            serialDevice.interfaceNumber = 0;
                            serialDevice.composite = false;
                            std::string port_name = port.get_name();
                            serialDevice.portName = port_name;
                            this->devices.push_back(serialDevice);
                        } catch (const libusbp::error &error) {
                        }
                    }
                } catch (const libusbp::error &error) {
                }
            }
        } catch (const libusbp::error &error) {
            std::cerr << "Error listing serial devices: " << error.what() << std::endl;
        }
    }
    if (type == UsbEnumerateRefreshType::USB_ENUMERATE_REFRESH_ALL || type ==
        UsbEnumerateRefreshType::USB_ENUMERATE_REFRESH_HID) {
        auto err = hid_init();
        if (err > 0) {
            std::cerr << "Error listing HID devices" << std::endl;
            return;
        }
        auto hids = hid_enumerate(0, 0);
        struct hid_device_info *curDev;
        curDev = hids;
        while (curDev) {
            SerialDevice serialDevice{};
            serialDevice.isHid = true;
            serialDevice.vid = curDev->vendor_id;
            serialDevice.pid = curDev->product_id;
            std::wstring wsSerialNumber(curDev->serial_number);
            serialDevice.serialNumber = std::string(wsSerialNumber.begin(), wsSerialNumber.end());
            serialDevice.path = curDev->path;
            serialDevice.interfaceNumber = curDev->interface_number;
            serialDevice.busType = static_cast<HidBusType>(curDev->bus_type);
            std::wstring wsManufacturerString(curDev->manufacturer_string);
            serialDevice.manufacturerString = std::string(wsManufacturerString.begin(), wsManufacturerString.end());
            std::wstring wsProductString(curDev->product_string);
            serialDevice.productString = std::string(wsProductString.begin(), wsProductString.end());
            serialDevice.releaseNumber = curDev->release_number;
            serialDevice.usage = curDev->usage;
            serialDevice.usagePage = curDev->usage_page;
            this->devices.push_back(std::move(serialDevice));
            curDev = curDev->next;
        }
    }
}

void UsbEnumerate::printDevices() {
    std::cout << "IsHID\tVID\t\tPID\t\tComposite\tSerial Number\tPort Name" << std::endl;
    for (const auto &device: devices) {
        std::string comp = "";
        if (device.composite) {
            comp = "YES(" + std::to_string(device.interfaceNumber) + ")";
        } else {
            comp = "NO";
        }
        std::cout << device.isHid << "\t"
                << std::hex << std::setw(4) << std::setfill('0') << device.vid << "\t\t"
                << std::hex << std::setw(4) << std::setfill('0') << device.pid << "\t\t"
                << comp << "\t\t"
                << device.serialNumber << "\t\t"
                << device.portName << std::endl;
    }
}

std::vector<SerialDevice> UsbEnumerate::findPorts(std::function<bool(const SerialDevice &)> filter) {
    std::vector<SerialDevice> filteredDevices;
    for (const auto &device: devices) {
        if (filter(device)) {
            filteredDevices.push_back(device);
        }
    }
    return filteredDevices;
}

//
// Created by max_3 on 2025/5/8.
//

#ifndef SERIALREBORN_COMMON_H
#define SERIALREBORN_COMMON_H
#include <cstdint>
#include <string>


#ifdef _WIN32
#define __export __declspec(dllexport) extern "C"
#else
#define __export
#endif

enum HidBusType {
	HID_BUS_UNKNOWN = 0x00,
	HID_BUS_USB = 0x01,
	HID_BUS_BLUETOOTH = 0x02,
	HID_BUS_I2C = 0x03,
	HID_BUS_SPI = 0x04,
};

struct SerialDevice {
    bool isHid;
    // Common
    uint16_t vid;
    uint16_t pid;
    uint8_t interfaceNumber;
    std::string serialNumber;
    // Serial
    std::string portName;
    bool composite;
    // HID
    HidBusType busType;
    std::string path;
    std::string manufacturerString;
    std::string productString;
    uint8_t releaseNumber;
    uint8_t usage;
    uint8_t usagePage;
};

#endif //SERIALREBORN_COMMON_H

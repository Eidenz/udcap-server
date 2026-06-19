//
// Created by max_3 on 2025/5/11.
//

#ifndef SERIALREBORN_PORTACCESSOR_H
#define SERIALREBORN_PORTACCESSOR_H

#include "SerialCommon.h"
#include <boost/asio.hpp>
#include "BoostHelper.hpp"
#include <vector>
#include <functional>
#include <thread>
#include <hidapi.h>
#include <map>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <string>

enum StopBit {
    STOPBIT_1,
    STOPBIT_1_5,
    STOPBIT_2
};

enum Parity {
    P_NONE, P_EVEN, P_ODD
};

enum FlowControl {
    FC_NONE, FC_SOFTWARE, FC_HARDWARE
};

struct SendDataItem {
    bool isString;
    std::string strData;
    std::vector<uint8_t> byteData;
};

struct ReceiveDataItem {
    bool isOnceRaw;
    std::vector<uint8_t> byteData;
};

class PacketRealignmentHelper {
public:
    PacketRealignmentHelper() = default;
    virtual ~PacketRealignmentHelper() = default;
    PacketRealignmentHelper(const PacketRealignmentHelper&) = delete;
    virtual std::vector<std::vector<uint8_t>> processPacket(const std::vector<uint8_t>& data) = 0;
};

enum PortAccessorConnectionState {
    PORT_ACCESSOR_CONNECTION_STATE_DISCONNECTED,
    PORT_ACCESSOR_CONNECTION_STATE_CONNECTED
};

class PortAccessor {
public:
    explicit PortAccessor(const SerialDevice& port);
    ~PortAccessor();
    PortAccessor(const PortAccessor&) = delete;
    PortAccessor& operator=(const PortAccessor&) = delete;
    static std::shared_ptr<PortAccessor> create(const SerialDevice& port);
    void openPort();
    void closePort();
    bool isOpen() const;
    void setBaudRate(int baudRate);
    void setStopBit(StopBit stopBit);
    void setParity(Parity parity);
    void setFlowControl(FlowControl flowControl);
    void setDataBits(int dataBits);
    void setReadSize(size_t size);
    void setTimeout(size_t timeout);
    std::vector<uint8_t> readData();
    void writeData(const std::vector<uint8_t>&);
    void writeData(const std::string&);

    void setPacketRealignmentHelper(std::unique_ptr<PacketRealignmentHelper> helper);
    bool hasPacketRealignmentHelper() const;
    std::unique_ptr<PacketRealignmentHelper> popPacketRealignmentHelper();
    std::function<void()> addDataCallback(std::function<void(std::shared_ptr<std::vector<uint8_t>>)>);
    std::function<void()> addConnectionCallback(std::function<void(PortAccessorConnectionState)>);
    void startContinuousRead();
    void stopContinuousRead();
    // Ask the read thread to close and reopen the port on its next loop. That
    // drops then re-raises DTR/RTS, which hardware-resets a CH340-based UDCAP
    // receiver -- the same thing a full app restart does -- so a receiver left
    // wedged after its glove power-cycled re-acquires the glove without a
    // restart. The work runs on the read thread (sole owner of the socket).
    void requestReset();
    void setWriteDelay(const uint64_t delay);
    std::function<void()> addOnceRawDataCallback(std::function<bool(std::shared_ptr<std::vector<uint8_t>>)>);
    void setPrintRxTxToStdOut(bool enable);
private:
    bool isDead = false;
    std::atomic_bool resetRequested{false};
    std::function<void()> unlistenHotPlugCallback;
    bool printRxTxToStdOut = false;
    std::atomic_uint32_t callbackFd = 0;
    void writeDataToDevice(const boost::asio::const_buffer &buffer);
    void callConnectionStateCallback(PortAccessorConnectionState state);
    SerialDevice serialDevice;
    boost::asio::io_context io;
    std::unique_ptr<DeadlineSocket<boost::asio::serial_port>> serialPort;
    hid_device* hidDevice = nullptr; // hid_device
    size_t readSize = 64;
    size_t timeout = 700;
    std::mutex ioMutex;
    bool isOpenFlag = false;
    std::mutex queueMutex;
    std::thread continuousReadThread;
    std::mutex callbackMutex;
    bool continuousReadRunning = false;
    std::atomic_int32_t continuousReadStartCount = 0;
    std::unique_ptr<PacketRealignmentHelper> packetRealignmentHelper;
    std::map<uint32_t, std::function<void(std::shared_ptr<std::vector<uint8_t>>)>> dataCallbacks;
    std::map<uint32_t, std::function<bool(std::shared_ptr<std::vector<uint8_t>>)>> onceRawDataCallbacks;
    std::map<uint32_t, std::function<void(PortAccessorConnectionState)>> connectionStateCallbacks;

    // EventLoop
    std::atomic_bool eventLoopRunning = true;
    // RX EventLoop
    std::thread rxEventThread;
    std::queue<ReceiveDataItem> rxQueue;
    std::mutex rxQueueMutex;
    std::condition_variable rxQueueCondition;
    // TX EventLoop
    std::atomic_uint16_t txEventLoopSendDelay = 0;
    std::thread txEventThread;
    std::queue<SendDataItem> txQueue;
    std::mutex txQueueMutex;
    std::condition_variable txQueueCondition;
};



#endif //SERIALREBORN_PORTACCESSOR_H

//
// Created by max_3 on 2025/5/11.
//

#include <utility>
#include "UdCapProbe.h"

std::vector<std::vector<uint8_t>> UdCapProbePacketRealignmentHelper::processPacket(const std::vector<uint8_t> &data) {
    std::vector<std::vector<uint8_t> > packets;
    for (auto &byte: data) {
        switch (this->stateMachine) {
            case UDCAP_PROBE_STATE_NONE: {
                if (byte == 'U') {
                    this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_U;
                }
                break;
            }
            case UDCAP_PROBE_STATE_GET_HEADER_U: {
                if (byte == 'D') {
                    this->stateMachine = UDCAP_PROBE_STATE_GET_HEADER_UD;
                    packetBuffer.clear();
                    packetBuffer.push_back('U');
                    packetBuffer.push_back('D');
                } else {
                    this->stateMachine = UDCAP_PROBE_STATE_NONE;
                }
                break;
            }
            case UDCAP_PROBE_STATE_GET_HEADER_UD: {
                if (byte != '\r') {
                    packetBuffer.push_back(byte);
                } else {
                    packets.push_back(packetBuffer);
                    this->stateMachine = UDCAP_PROBE_STATE_NONE;
                }
                break;
            }
        }
    }
    return packets;
}

UdCapProbe::UdCapProbe(std::shared_ptr<PortAccessor> portAccessor): portAccessor(std::move(portAccessor)) {

}

UdCapProbe::~UdCapProbe() = default;

UdCapProbeType UdCapProbe::probe() {
    std::function<void()> unlisten = [](){};
    try {
        this->udCapSerial = "";
        portAccessor->openPort();
        portAccessor->setBaudRate(115200);
        portAccessor->setReadSize(8);
        std::mutex mutex;
        std::condition_variable condition;
        std::string uds = "";
        std::unique_ptr<PacketRealignmentHelper> pPacketRealignmentHelper = portAccessor->popPacketRealignmentHelper();
        portAccessor->setPacketRealignmentHelper(std::make_unique<UdCapProbePacketRealignmentHelper>());
        unlisten = portAccessor->addDataCallback([this, &uds, &condition](std::shared_ptr<std::vector<uint8_t>> data) {
            uds = std::string(data->begin(), data->end());
            condition.notify_all();
        });
        portAccessor->addOnceRawDataCallback([&condition](std::shared_ptr<std::vector<uint8_t>> data){
            condition.notify_all();
            return true;
        });
        portAccessor->startContinuousRead();
        std::string s = "AT+NAME?\r\n";
        portAccessor->writeData(s);
        std::unique_lock lk(mutex);
        auto state = condition.wait_for(lk, std::chrono::seconds(1));
        if (state == std::cv_status::timeout) {
            unlisten();
            portAccessor->stopContinuousRead();
            portAccessor->setPacketRealignmentHelper(std::move(pPacketRealignmentHelper));
            this->udCapSerial = "";
            return UDCAP_PROBE_FAILURE;
        }
        state = condition.wait_for(lk, std::chrono::seconds(3));
        unlisten();
        portAccessor->stopContinuousRead();
        portAccessor->setPacketRealignmentHelper(std::move(pPacketRealignmentHelper));
        if (state == std::cv_status::timeout) {
            this->udCapSerial = "";
            return UDCAP_PROBE_FAILURE;
        }
        this->udCapSerial = uds;
        return UDCAP_PROBE_HAND_V1;
    } catch (std::runtime_error&) {
        this->udCapSerial = "";
        if (portAccessor->isOpen()) {
            this->portAccessor->stopContinuousRead();
        }
        return UDCAP_PROBE_FAILURE;
    }
    this->udCapSerial = "";
    if (portAccessor->isOpen()) {
        this->portAccessor->stopContinuousRead();
    }
    return UDCAP_PROBE_FAILURE;
}

std::string UdCapProbe::getUDCapSerial() {
    return this->udCapSerial;
}
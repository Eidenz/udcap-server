#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <SDKDDKVer.h>
#endif

#include <iostream>
#include <functional>
#include <vector>
#include <UsbEnumerate.h>
#include <PortAccessor.h>
#include <UdCapProbe.h>
#include <UdCapV1Core.h>
#if WIN32
#include <windows.h>
#else
#include <csignal>
#endif

std::vector<std::function<void()>> unlistenFunc;
std::vector<std::shared_ptr<UdCapV1Core>> cores;
std::mutex mtx;
std::condition_variable cv;

#if WIN32
BOOL WINAPI CtrlHandler(DWORD fdwCtrlType) {
    switch (fdwCtrlType) {
        case CTRL_C_EVENT: {
            std::cout << "Capture Ctrl-C: Stopping..." << std::endl;
            cv.notify_all();
            return TRUE;
        }
        default: {
            return FALSE;
        }
    }
}
#else
void signal_handler(int signal)
{
    cv.notify_all();
}
#endif

int main() {
#if WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
#endif
    try {
        UsbEnumerate::getInstance()->refresh(USB_ENUMERATE_REFRESH_SERIAL);
        UsbEnumerate::getInstance()->printDevices();
        std::vector<SerialDevice> devices = UsbEnumerate::getInstance()->findPorts([](const SerialDevice &device) {
            return device.vid == 0x1A86 && device.pid == 0x7523;
        });
        for (const auto &device: devices) {
            std::cout << "Found device: " << device.portName << std::endl;
            std::shared_ptr<PortAccessor> portAccessor = std::make_shared<PortAccessor>(device);
#ifdef TEST_TOOLS_PRINT_RAW
            portAccessor->setPrintRxTxToStdOut(true);
#endif
            UdCapProbe prober(portAccessor);
            switch (prober.probe()) {
                case UDCAP_PROBE_FAILURE: {
                    std::cout << "  Probe failed." << std::endl;
                    break;
                }
                case UDCAP_PROBE_HAND_V1: {
                    std::cout << "  Probe successful. This is a UdCap Receiver with SN: " << prober.getUDCapSerial() <<
                            std::endl;
                    std::shared_ptr<UdCapV1Core> core = std::make_shared<UdCapV1Core>(portAccessor);
                    auto unlisten = core->listen([&core](std::shared_ptr<UdCapV1MCUPacket> data) {
                        if (data->commandType == CMD_LINK_STATE) {
                            std::cout << "  Link State: " << UdCapV1Core::fromUdStateToString(data->udState) <<
                                    std::endl;
                            if (data->udState == UdState::UD_INIT_STATE_LINKED) {
                                std::thread t([&core]() {
                                    try {
                                        std::cout << "Start calibration. " << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(1));
                                        core->runCalibration(UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND);
                                        std::cout << "Start calibration. Fist!" << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(5));
                                        std::cout << "Getting Value Fist!" << std::endl;
                                        core->captureCalibrationData(UdCapV1HandCaliType::UDCAP_V1_HAND_CALI_TYPE_FIST);
                                        std::cout << "Start calibration. Adduction!" << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(5));
                                        std::cout << "Getting Value Adduction!" << std::endl;
                                        core->captureCalibrationData(
                                            UdCapV1HandCaliType::UDCAP_V1_HAND_CALI_TYPE_ADDUCTION);
                                        std::cout << "Start calibration. Protract!" << std::endl;
                                        std::this_thread::sleep_for(std::chrono::seconds(5));
                                        std::cout << "Getting Value Protract!" << std::endl;
                                        core->captureCalibrationData(
                                            UdCapV1HandCaliType::UDCAP_V1_HAND_CALI_TYPE_PROTRACT);
                                        std::cout << "Done!" << std::endl;
                                        core->completeCalibration(UdCapV1DeviceCaliType::UDCAP_V1_DEVICE_CALI_TYPE_HAND);
                                    } catch (std::exception &e) {
                                        std::cerr << e.what() << std::endl;
                                    }
                                });
                                t.detach();
                            }
                        } else if (data->commandType == CMD_SERIAL) {
                            std::cout << "  Serial Num: " << data->deviceSerialNum << std::endl;
                            std::cout << "    Hand: " << (core->getTarget() == UD_TARGET_LEFT_HAND
                                                              ? "Left"
                                                              : (core->getTarget() == UD_TARGET_RIGHT_HAND
                                                                     ? "Right"
                                                                     : "Unknown")) << std::endl;
                            std::cout << "    Type: " << (data->isEnterprise ? "Enterprise" : "Client") << std::endl;
                        } else if (data->commandType == CMD_DATA) {
                             std::cout << "  Angle: ";
                             for (const auto &angle: data->angle) {
                                 std::cout << std::dec << angle << " ";
                             }
                             std::cout << std::endl;
                        } else if (data->commandType == CMD_GET_CHANNEL) {
                            std::cout << "    Channel: " << static_cast<unsigned>(data->channel) << std::endl;
                        } else if (data->commandType == CMD_READY) {
                            if (data->isReady) {
                                std::cout << "Ready." << std::endl;
                                // core.mcuSendVibration(1, 2, 10);
                            }
                        } else if (data->commandType == CMD_ANGLE) {
                            std::array<double, 28> angles = data->result;
                            std::cout << "Angles: ";
                            for (int i = 0; i < angles.size(); i++) {
                                std::cout << angles[i] << ", ";
                            }
                            std::cout << std::endl;
                        } else if (data->commandType == CMD_INPUT_JOYSTICK) {
                                std::cout << "Input joystick. X: " << data->joystickData.joyX << " Y: " << data->
                                        joystickData.joyY << std::endl;
                        } else if (data->commandType == CMD_INPUT_BUTTON) {
                                std::cout << "Input Button. A: " << data->button.btnA << " B: " << data->button.btnB <<
                                        " Menu: " << data->button.btnMenu << " JoyStick: " << data->button.btnJoyStick <<
                                        " PWR: " << data->button.btnPower <<
                                        " Trigger: " << data->button.btnTrigger << "(" << data->button.trigger << ")" <<
                                        " Grip: " << data->button.btnGrip << "(" << data->button.grip << ")" <<
                                        " Trackpad: " << data->button.btnTrackpad << "(" << data->button.trackpad << ")" <<
                                        std::endl;
                        } else if (data->commandType == CMD_SKELETON_QUATERNION) {
                            std::cout << "Skeleton Quaternion: " << std::endl;

                            std::cout << " Thumb Distal: " << data->skeletonQuaternion.thumbFinger.distal.x << ", "
                                      << data->skeletonQuaternion.thumbFinger.distal.y << ", "
                                      << data->skeletonQuaternion.thumbFinger.distal.z << ", "
                                      << data->skeletonQuaternion.thumbFinger.distal.w << std::endl;
                            std::cout << " Thumb Intermediate: " << data->skeletonQuaternion.thumbFinger.intermediate.x << ", "
                                        << data->skeletonQuaternion.thumbFinger.intermediate.y << ", "
                                        << data->skeletonQuaternion.thumbFinger.intermediate.z << ", "
                                        << data->skeletonQuaternion.thumbFinger.intermediate.w << std::endl;
                            std::cout << " Thumb Proximal: " << data->skeletonQuaternion.thumbFinger.proximal.x << ", "
                                        << data->skeletonQuaternion.thumbFinger.proximal.y << ", "
                                        << data->skeletonQuaternion.thumbFinger.proximal.z << ", "
                                        << data->skeletonQuaternion.thumbFinger.proximal.w << std::endl;

                            std::cout << " Index Distal: " << data->skeletonQuaternion.indexFinger.distal.x << ", "
                                      << data->skeletonQuaternion.indexFinger.distal.y << ", "
                                      << data->skeletonQuaternion.indexFinger.distal.z << ", "
                                      << data->skeletonQuaternion.indexFinger.distal.w << std::endl;
                            std::cout << " Index Intermediate: " << data->skeletonQuaternion.indexFinger.intermediate.x << ", "
                                      << data->skeletonQuaternion.indexFinger.intermediate.y << ", "
                                      << data->skeletonQuaternion.indexFinger.intermediate.z << ", "
                                      << data->skeletonQuaternion.indexFinger.intermediate.w << std::endl;
                            std::cout << " Index Proximal: " << data->skeletonQuaternion.indexFinger.proximal.x << ", "
                                      << data->skeletonQuaternion.indexFinger.proximal.y << ", "
                                      << data->skeletonQuaternion.indexFinger.proximal.z << ", "
                                      << data->skeletonQuaternion.indexFinger.proximal.w << std::endl;

                            std::cout << " Middle Distal: " << data->skeletonQuaternion.middleFinger.distal.x << ", "
                                      << data->skeletonQuaternion.middleFinger.distal.y << ", "
                                      << data->skeletonQuaternion.middleFinger.distal.z << ", "
                                      << data->skeletonQuaternion.middleFinger.distal.w << std::endl;
                            std::cout << " Middle Intermediate: " << data->skeletonQuaternion.middleFinger.intermediate.x << ", "
                                      << data->skeletonQuaternion.middleFinger.intermediate.y << ", "
                                      << data->skeletonQuaternion.middleFinger.intermediate.z << ", "
                                      << data->skeletonQuaternion.middleFinger.intermediate.w << std::endl;
                            std::cout << " Middle Proximal: " << data->skeletonQuaternion.middleFinger.proximal.x << ", "
                                      << data->skeletonQuaternion.middleFinger.proximal.y << ", "
                                      << data->skeletonQuaternion.middleFinger.proximal.z << ", "
                                      << data->skeletonQuaternion.middleFinger.proximal.w << std::endl;

                            std::cout << " Ring Distal: " << data->skeletonQuaternion.ringFinger.distal.x << ", "
                                      << data->skeletonQuaternion.ringFinger.distal.y << ", "
                                      << data->skeletonQuaternion.ringFinger.distal.z << ", "
                                      << data->skeletonQuaternion.ringFinger.distal.w << std::endl;
                            std::cout << " Ring Intermediate: " << data->skeletonQuaternion.ringFinger.intermediate.x << ", "
                                      << data->skeletonQuaternion.ringFinger.intermediate.y << ", "
                                      << data->skeletonQuaternion.ringFinger.intermediate.z << ", "
                                      << data->skeletonQuaternion.ringFinger.intermediate.w << std::endl;
                            std::cout << " Ring Proximal: " << data->skeletonQuaternion.ringFinger.proximal.x << ", "
                                      << data->skeletonQuaternion.ringFinger.proximal.y << ", "
                                      << data->skeletonQuaternion.ringFinger.proximal.z << ", "
                                      << data->skeletonQuaternion.ringFinger.proximal.w << std::endl;

                            std::cout << " Little Distal: " << data->skeletonQuaternion.littleFinger.distal.x << ", "
                                      << data->skeletonQuaternion.littleFinger.distal.y << ", "
                                      << data->skeletonQuaternion.littleFinger.distal.z << ", "
                                      << data->skeletonQuaternion.littleFinger.distal.w << std::endl;
                            std::cout << " Little Intermediate: " << data->skeletonQuaternion.littleFinger.intermediate.x << ", "
                                      << data->skeletonQuaternion.littleFinger.intermediate.y << ", "
                                      << data->skeletonQuaternion.littleFinger.intermediate.z << ", "
                                      << data->skeletonQuaternion.littleFinger.intermediate.w << std::endl;
                            std::cout << " Little Proximal: " << data->skeletonQuaternion.littleFinger.proximal.x << ", "
                                      << data->skeletonQuaternion.littleFinger.proximal.y << ", "
                                      << data->skeletonQuaternion.littleFinger.proximal.z << ", "
                                      << data->skeletonQuaternion.littleFinger.proximal.w << std::endl;

                            std::cout << std::endl;
                        } else if (data->commandType == CMD_PORT_CONNECTION_STATE) {
                            std::cout << "Port Connection State: " << (data->portConnectionState ? "true" : "false") << std::endl;
                        }
                    });
                    cores.push_back(core);
                    unlistenFunc.push_back(unlisten);
                    break;
                }
            }
        }
    } catch (const std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    if (!cores.empty()) {
        std::unique_lock lk(mtx);
        cv.wait(lk);
        for (auto unlisten: unlistenFunc) {
            unlisten();
        }
        cores.clear();
    }
    return 0;
}

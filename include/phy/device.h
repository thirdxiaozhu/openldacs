//
// Created by jiaxv on 2026/1/8.
//

#ifndef OPENLDACS_DEVICE_H
#define OPENLDACS_DEVICE_H

#include <uhd/usrp/multi_usrp.hpp>

#include "openldacs.h"

namespace openldacs::phy::device {

    enum class DeviceType : int {
        USRP = 0,
        HACKRF,
    };

    enum DeviceDirection : uint8_t {
        TX = 0x01,
        RX = 0x10,
    };

    class Device {
    public:
        virtual ~Device() = default;

    protected:
        explicit Device(const uint8_t dir): direction_(dir) {

        };
        virtual void setupDevice() = 0;
        uint8_t direction_;
        const double rate_ = 625e3;             // 例如 LDACS 1.6 Msps（你也可设 625k 等）
        const double tx_gain_ = 30.0;
        const double rx_gain_ = 30.0;

        const double fl_freq = 1110e6;
        const double rl_freq = 964e6;
    private:
    };

    class USRP final: public Device {
    public:
        explicit USRP(const uint8_t dir) : Device(dir) {
            setupDevice();
        }
        void setupDevice() override;
    private:
        const std::string args_ = "type=b200,serial=192113";   // 也可以加 serial=xxxx
        std::shared_ptr<uhd::usrp::multi_usrp> usrp_;
    };


    class DeviceFactory {
    public:
        static std::unique_ptr<Device> createDevice(DeviceType type, uint8_t direction) {
            switch (type) {
                case DeviceType::USRP:
                    return std::make_unique<USRP>(direction);
                case DeviceType::HACKRF:
                    // return std::make_unique<HackRF>();
                    throw std::runtime_error("HACKRF device not implemented yet");
                default:
                    throw std::invalid_argument("Unknown device type");
            }
        }
    };


}

#endif //OPENLDACS_DEVICE_H
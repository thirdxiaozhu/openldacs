//
// Created by jiaxv on 2026/1/8.
//

#ifndef OPENLDACS_DEVICE_H
#define OPENLDACS_DEVICE_H

#include <uhd/usrp/multi_usrp.hpp>

#include "openldacs.h"
#include "util/bounded_priority_queue.h"
#include "util/worker.h"

namespace openldacs::phy::device {

    enum class DeviceType : int {
        USRP = 0,
        HACKRF,
    };

    enum DeviceDirection : uint8_t {
        TX = 0x01,
        RX = 0x10,
    };

    constexpr int CAP_HIGH = 512;
    constexpr int CAP_NORM = 2048;


    class Device {
    public:
        virtual ~Device() = default;
        void sendPDU(const itpp::cvec &pdu, const util::Priority pri) {
            pdu_out_.push(pdu, pri);
        }

    protected:
        explicit Device(const uint8_t dir) : direction_(dir), pdu_out_(CAP_HIGH, CAP_NORM) {
        };
        virtual void setupDevice() = 0;
        virtual void transThread() = 0;
        uint8_t direction_;
        const double rate_ = 625e3;             // 例如 LDACS 1.6 Msps（你也可设 625k 等）
        const double tx_gain_ = 30.0;
        const double rx_gain_ = 30.0;

        const double fl_freq = 1110e6;
        const double rl_freq = 964e6;

        util::BoundedPriorityQueue<itpp::cvec> pdu_out_;
        util::Worker trans_worker_;
    private:
    };

    class USRP final: public Device {
    public:
        explicit USRP(const uint8_t dir) : Device(dir) {
            setupDevice();

            trans_worker_.start([&] {
                std::unique_lock<std::mutex> lk(trans_worker_.mutex());

                while (!trans_worker_.stop_requested()) {

                }
            });
        }
    private:
        const std::string args_ = "type=b200,serial=192113";   // 也可以加 serial=xxxx
        std::shared_ptr<uhd::usrp::multi_usrp> usrp_;
        void setupDevice() override;
        void transThread() override;
    };


    class DeviceFactory {
    public:
        static std::unique_ptr<Device> createDevice(const DeviceType type, uint8_t direction) {
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
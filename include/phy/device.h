//
// Created by jiaxv on 2026/1/8.
//

#ifndef OPENLDACS_DEVICE_H
#define OPENLDACS_DEVICE_H

#include <uhd/usrp/multi_usrp.hpp>

#include "openldacs.h"
#include "util/bounded_priority_queue.h"
#include "util/util.h"
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
        void sendData(const itpp::cvec &data, const util::Priority pri) {
            fl_to_trans_.push(util::cvecToComplexDoubleVec(data), pri);
        }

    protected:
        explicit Device(const uint8_t role) : role_(role), fl_to_trans_(CAP_HIGH, CAP_NORM) {
        };
        virtual void setupDevice() = 0;
        virtual void transThread() = 0;
        // 逻辑不对！！！ 注意这里的应该是AS或者gs,但是b210不支持 两个射频
        uint8_t role_;
        const double rate_ = 625e3;             // 例如 LDACS 1.6 Msps（你也可设 625k 等）
        const double tx_gain_ = 30.0;
        const double rx_gain_ = 30.0;

        const double fl_freq = 1110e6;
        const double rl_freq = 964e6;

        util::BoundedPriorityQueue<VecCD> fl_to_trans_;
        util::Worker trans_worker_;
    private:
    };

    class USRP final: public Device {
    public:
        explicit USRP(const uint8_t role) : Device(role),
            tx_args_("fc32"),
            rx_args_("fc32")
        {
            setupDevice();

            trans_worker_.start([&] {
                std::unique_lock<std::mutex> lk(trans_worker_.mutex());
                if (!tx_stream_ || !rx_stream_) {
                        SPDLOG_ERROR("get_tx_stream failed! tx_stream_ / rx_stream_ is null");
                }
                bool first_trans = true;
                while (!trans_worker_.stop_requested()) {
                    VecCD fl_vec;
                    VecCD rl_vec;

                    if (role_ & AS) {
                    }
                    if (role_ & GS) {
                        fl_vec = fl_to_trans_.pop();
                    }

                    std::vector<std::complex<double> *> buffs = {
                        fl_vec.data(),
                        fl_vec.data()
                    };

                    uhd::tx_metadata_t md;
                    md.start_of_burst = first_trans;
                    md.end_of_burst = false;  // 如果没数据的话，会UUUUUU

                    tx_stream_->send(buffs, fl_vec.size(), md);
                    first_trans = false;
                }

                // 退出循环后，发送结束标记
                uhd::tx_metadata_t md_end;
                md_end.end_of_burst = true;
                tx_stream_->send("", 0, md_end);
            });
        }
    private:
        const std::string args_ = "type=b200,serial=192113";   // 也可以加 serial=xxxx
        std::shared_ptr<uhd::usrp::multi_usrp> usrp_;
        uhd::stream_args_t tx_args_;
        uhd::stream_args_t rx_args_;

        std::shared_ptr<uhd::tx_streamer> tx_stream_;
        std::shared_ptr<uhd::rx_streamer> rx_stream_;
        void setupDevice() override;
        void transThread() override;
    };


    class DeviceFactory {
    public:
        static std::unique_ptr<Device> createDevice(const DeviceType type, uint8_t role) {
            switch (type) {
                case DeviceType::USRP:
                    return std::make_unique<USRP>(role);
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
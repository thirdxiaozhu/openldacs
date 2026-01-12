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

    enum RoleChannel : uint8_t {
        AS_CHANNEL = 0x00,
        GS_CHANNEL = 0x01,
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
        // 逻辑不对！！！ 注意这里的应该是AS或者gs,但是b210不支持 两个射频
        uint8_t role_;
        const double rate_ = 625e3;             // 例如 LDACS 1.6 Msps（你也可设 625k 等）
        const double tx_gain_ = 30.0;
        const double rx_gain_ = 30.0;

        const double fl_freq = 1110e6;
        const double rl_freq = 964e6;

        util::BoundedPriorityQueue<VecCD> fl_to_trans_;
        util::Worker trans_worker_;
        util::Worker recv_worker_;
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
                        while (!fl_to_trans_.try_pop(fl_vec)) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            if (trans_worker_.stop_requested()) {
                                break;
                            }
                        }

                        if (trans_worker_.stop_requested()) {
                            break;
                        }
                        // fl_vec = fl_to_trans_.pop();
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

            recv_worker_.start([&] {
                // 接收缓冲区
                VecCD buff(recv_samples_);
                uhd::rx_metadata_t md;

                // 启动连续流
                uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                stream_cmd.stream_now = true;
                rx_stream_->issue_stream_cmd(stream_cmd);

                while (!trans_worker_.stop_requested()) {
                    size_t num_rx = rx_stream_->recv(&buff[0], buff.size(), md, 1.0);

                    // 错误处理
                    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                        std::cerr << "[RX] 超时" << std::endl;
                        continue;
                    }
                    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                        std::cerr << "[RX] 溢出 (O)" << std::endl;
                        continue;
                    }
                    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                        std::cerr << "[RX] 错误: " << md.strerror() << std::endl;
                        break;
                    }

                    // 复制数据到队列
                    VecCD data(buff.begin(), buff.begin() + num_rx);
                    if (!data_queue.push(std::move(data))) {
                        std::cerr << "[RX] 队列满，丢弃数据" << std::endl;
                    }
                }
                const uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
                rx_stream_->issue_stream_cmd(stop_cmd);
            });
        }

        ~USRP() override {
            trans_worker_.request_stop();
            recv_worker_.request_stop();
            trans_worker_.join_and_rethrow();
            recv_worker_.join_and_rethrow();
        }
    private:
        static constexpr int recv_samples_ = 16384;
        const std::string args_ = "type=b200,serial=3459F45";   // 也可以加 serial=xxxx 3459f45
        std::shared_ptr<uhd::usrp::multi_usrp> usrp_;
        uhd::stream_args_t tx_args_;
        uhd::stream_args_t rx_args_;

        std::shared_ptr<uhd::tx_streamer> tx_stream_;
        std::shared_ptr<uhd::rx_streamer> rx_stream_;
        void setupDevice() override;
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
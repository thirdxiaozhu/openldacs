//
// Created by jiaxv on 2026/1/8.
//

#ifndef OPENLDACS_DEVICE_H
#define OPENLDACS_DEVICE_H

#include <itpp/base/specmat.h>
#include <uhd/usrp/multi_usrp.hpp>

#include "openldacs.h"
#include "util/queue.h"
#include "util/util.h"
#include "util/worker.h"

#include <zmq.hpp>

namespace openldacs::phy::device {
    using namespace openldacs::util;
    using cd = std::complex<double>;

    enum class DeviceType : int {
        USRP = 0,
        HACKRF,
    };

    enum RoleChannel : uint8_t {
        FL_CHANNEL = 0x00,
        RL_CHANNEL = 0x01,
    };

    constexpr int CAP_HIGH = 512;
    constexpr int CAP_NORM = 2048;


    class Device {
    public:
        using RxCallbackType = std::function<void(VecCD)>;

        void sendData(const itpp::cvec &data, const util::Priority pri) {
            fl_to_trans_.push(util::cvecToCdVecNormalize(data), pri);
        }

        void registerRxCallback(const std::function<void(VecCD)> &cb) {
            rx_callback_ = cb;
        }

        void setSigmaN(const double new_sigma) {
            sigma_n_ = new_sigma;
        }

        double getSigmaN() const {
            return sigma_n_;
        }

        virtual ~Device() {
            fl_to_trans_.close();
            trans_worker_.requestStop();
            recv_worker_.requestStop();
            tx_async_worker_.requestStop();
            trans_worker_.joinAndRethrow();
            recv_worker_.joinAndRethrow();
            tx_async_worker_.joinAndRethrow();
        }
    protected:
        explicit Device(const uint8_t role) : role_(role), fl_to_trans_(CAP_HIGH, CAP_NORM) {
        };
        // 逻辑不对！！！ 注意这里的应该是AS或者gs,但是b210不支持 两个射频
        uint8_t role_;
        const double rate_ = 625e3;             // 例如 LDACS 1.6 Msps（你也可设 625k 等）
        // Loopback debug defaults: keep TX low to avoid front-end saturation.
        const double tx_gain_ = 0.0;
        const double rx_gain_ = 15.0;

        const double fl_freq_ = 1110e6;
        const double rl_freq_ = 964e6;

        double sigma_n_ = 0.1;

        util::BoundedPriorityQueue<VecCD> fl_to_trans_;
        util::Worker trans_worker_;
        util::Worker recv_worker_;
        util::Worker tx_async_worker_;

        RxCallbackType rx_callback_;

        virtual void setupDevice() = 0;

    private:

    };
    using DevPtr = std::unique_ptr<Device> ;

    class USRP final: public Device {
    public:
        explicit USRP(const uint8_t role) : Device(role),
            tx_args_("fc32"),
            rx_args_("fc32")
        {
            setupDevice();

            trans_worker_.start([&] {

                bool first_trans = true;
                VecCF idle_silence(recv_samples_, std::complex<float>(0.0f, 0.0f));
                if (!tx_stream_ || !rx_stream_) {
                        SPDLOG_ERROR("get_tx_stream failed! tx_stream_ / rx_stream_ is null");
                        return;
                }
                while (!trans_worker_.stop_requested()) {
                    std::optional<VecCD> fl_vec;
                    VecCD fl_try_pop;

                    if (role_ & AS) {
                    }
                    if (role_ & GS) {
                        if (fl_to_trans_.try_pop(fl_try_pop)) {
                            fl_vec = std::move(fl_try_pop);
                        }

                        // if (fl_vec.has_value()) {
                        //     VecCD to_sync_frame = fl_vec.value();
                        //
                        //     // 生成高斯白噪声
                        //     std::random_device rd;
                        //     std::mt19937 gen(rd());
                        //     std::normal_distribution<> dis(0.0, 1.0); // 均值为0，标准差为1的正态分布
                        //
                        //     for (int i = 0; i < fl_vec->size(); i++) {
                        //         to_sync_frame[i] += std::complex<double>(dis(gen), dis(gen)) * 0.1; // 缩放因子可根据需要调整
                        //     }
                        //
                        //     if (rx_callback_) {
                        //         rx_callback_(to_sync_frame);
                        //     }
                        // }
                    }

                    if (!fl_vec.has_value() || fl_vec->empty()) {
                        uhd::tx_metadata_t md;
                        md.start_of_burst = first_trans;
                        md.end_of_burst = false;
                        const size_t sent_idle = tx_stream_->send(
                            idle_silence.data(), idle_silence.size(), md, 0.2);
                        if (sent_idle == 0) {
                            SPDLOG_WARN("USRP TX idle send returned 0");
                        } else {
                            first_trans = false;
                        }
                        continue;
                    }

                    // 降级为float，以满足fc32
                    VecCF fl_vec_cf(fl_vec.value().begin(), fl_vec.value().end());

                    size_t sent_total = 0;
                    bool start_of_burst = first_trans;
                    while (sent_total < fl_vec_cf.size() && !trans_worker_.stop_requested()) {
                        uhd::tx_metadata_t md;
                        md.start_of_burst = start_of_burst;
                        md.end_of_burst = false; // 如果没数据的话，会UUUUUU

                        const size_t sent_now = tx_stream_->send(
                            &fl_vec_cf[sent_total], fl_vec_cf.size() - sent_total, md, 1.0);
                        if (sent_now == 0) {
                            SPDLOG_WARN(
                                "USRP TX send returned 0, drop remaining {} samples",
                                fl_vec_cf.size() - sent_total);
                            break;
                        }

                        sent_total += sent_now;
                        start_of_burst = false;
                    }

                    if (sent_total < fl_vec_cf.size()) {
                        SPDLOG_WARN("USRP TX short send: {}/{}", sent_total, fl_vec_cf.size());
                    }
                    if (sent_total > 0) {
                        first_trans = false;
                    }
                }

                // 退出循环后，发送结束标记
                uhd::tx_metadata_t md_end;
                md_end.end_of_burst = true;
                tx_stream_->send("", 0, md_end);
            });

            tx_async_worker_.start([&] {
                if (!tx_stream_) {
                    SPDLOG_ERROR("tx_async_worker cannot start: tx_stream_ is null");
                    return;
                }
                while (!tx_async_worker_.stop_requested()) {
                    uhd::async_metadata_t async_md;
                    if (!tx_stream_->recv_async_msg(async_md, 0.1)) {
                        continue;
                    }

                    if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW ||
                        async_md.event_code == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET) {
                        SPDLOG_WARN("USRP TX underflow event: {}", static_cast<int>(async_md.event_code));
                        continue;
                    }

                    if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR ||
                        async_md.event_code == uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST) {
                        SPDLOG_WARN("USRP TX seq error event: {}", static_cast<int>(async_md.event_code));
                        continue;
                    }

                    if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_TIME_ERROR) {
                        SPDLOG_WARN("USRP TX time error event");
                        continue;
                    }
                }
            });

            recv_worker_.start([&] {
                // 接收缓冲区
                VecCF buff(recv_samples_);
                uhd::rx_metadata_t md;

                // 启动连续流
                uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                stream_cmd.stream_now = true;
                rx_stream_->issue_stream_cmd(stream_cmd);

                while (!recv_worker_.stop_requested()) {
                    // const auto t0 = std::chrono::high_resolution_clock::now();
                    const size_t num_rx = rx_stream_->recv(&buff[0], buff.size(), md, 1.0); //测试阶段单fl通道
                    // const auto t1 = std::chrono::high_resolution_clock::now();
                    // const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).
                    //         count();
                    // SPDLOG_INFO("recv {} ns", ns);

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

                    if (rx_callback_) {
                        VecCD data(buff.begin(), buff.begin() + num_rx);
                        rx_callback_(std::move(data));
                    }
                }
                const uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
                rx_stream_->issue_stream_cmd(stop_cmd);
            });
        }

        ~USRP() override {
        }
    private:
        static constexpr int recv_samples_ = 4096;
        const std::string args_ = "type=b200,serial=192113";   // 也可以加 serial=xxxx 3459f45
        std::shared_ptr<uhd::usrp::multi_usrp> usrp_;
        uhd::stream_args_t tx_args_;
        uhd::stream_args_t rx_args_;

        std::shared_ptr<uhd::tx_streamer> tx_stream_;
        std::shared_ptr<uhd::rx_streamer> rx_stream_;
        void setupDevice() override;
    };


    class DeviceFactory {
    public:
        static DevPtr createDevice(const DeviceType type, uint8_t role) {
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

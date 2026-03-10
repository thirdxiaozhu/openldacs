//
// Created by jiaxv on 2026/1/8.
//

#ifndef OPENLDACS_DEVICE_H
#define OPENLDACS_DEVICE_H

#include <itpp/base/specmat.h>
#include <uhd/usrp/multi_usrp.hpp>

#include <atomic>

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
        using RxCallbackType = std::function<void(VecCF)>;

        void sendData(const itpp::cvec &data, const util::Priority pri) {
            fl_to_trans_.push(util::cvecToCdVecNormalize(data), pri);
        }

        void registerRxCallback(const std::function<void(VecCF)> &cb) {
            rx_callback_ = cb;
        }

        void setSnrDb(const double new_snr_db) {
            snr_db_.store(new_snr_db);
        }

        double getSnrDb() const {
            return snr_db_.load();
        }

        double getSigmaN() const {
            return std::sqrt(getNoisePowerLinear() / 2.0);
        }

        double getNoisePowerLinear() const {
            return last_noise_power_linear_.load();
        }

        virtual ~Device() noexcept {
            fl_to_trans_.close();
            trans_worker_.requestStop();
            recv_worker_.requestStop();
            tx_async_worker_.requestStop();
            trans_worker_.joinNoexcept("Device::trans_worker");
            recv_worker_.joinNoexcept("Device::recv_worker");
            tx_async_worker_.joinNoexcept("Device::tx_async_worker");
        }
    protected:
        explicit Device(const uint8_t role) : role_(role), fl_to_trans_(CAP_HIGH, CAP_NORM) {
        };
        // 逻辑不对！！！ 注意这里的应该是AS或者gs,但是b210不支持 两个射频
        uint8_t role_;
        const double rate_ = 625e3;             // 例如 LDACS 1.6 Msps（你也可设 625k 等）
        // Loopback debug defaults: keep TX low to avoid front-end saturation.
        const double tx_gain_ = 80.0;
        const double rx_gain_ = 40.0;

        const double fl_freq_ = 1110e6;
        const double rl_freq_ = 964e6;

        const double lo_offset = 1e6;

        std::atomic<double> snr_db_ = 30.0;
        std::atomic<double> last_noise_power_linear_ = 0.0;

        BoundedPriorityQueue<VecCD> fl_to_trans_;
        Worker trans_worker_;
        Worker recv_worker_;
        Worker tx_async_worker_;

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
            // setupDevice();
            trans_worker_.start([&] {
                // sleep(1);
                bool first_trans = true;
                while (!trans_worker_.stop_requested()) {
                    std::optional<VecCD> fl_vec;
                    VecCD fl_try_pop;

                    if (role_ & AS) {
                    }
                    if (role_ & GS) {
                        std::random_device rd;
                        std::mt19937 gen(rd());
                        if (fl_to_trans_.try_pop(fl_try_pop)) {
                            fl_vec = std::move(fl_try_pop);
                        }

                        if (fl_vec.has_value()) {
                            VecCD to_sync_frame = fl_vec.value();

                            double signal_power = 0.0;
                            for (const auto &sample : to_sync_frame) {
                                signal_power += std::norm(sample);
                            }
                            if (!to_sync_frame.empty()) {
                                signal_power /= static_cast<double>(to_sync_frame.size());
                            }

                            const double snr_linear = std::pow(10.0, getSnrDb() / 10.0);
                            const double noise_power = snr_linear > 0.0 ? signal_power / snr_linear : 0.0;
                            last_noise_power_linear_.store(noise_power);

                            const double sigma_n = std::sqrt(noise_power / 2.0);
                            std::normal_distribution<double> dis(0.0, sigma_n);

                            for (size_t i = 0; i < fl_vec->size(); ++i) {
                                to_sync_frame[i] += std::complex<double>(dis(gen), dis(gen));
                            }

                            VecCF vf(to_sync_frame.begin(), to_sync_frame.end());

                            if (rx_callback_) {
                                rx_callback_(vf);
                            }
                        }
                    }

                    // if (!fl_vec.has_value() || fl_vec->empty()) {
                    //     SPDLOG_WARN("No Data avialiable");
                    //     continue;
                    // }
                    //
                    // // 降级为float，以满足fc32
                    // VecCF fl_vec_cf(fl_vec.value().begin(), fl_vec.value().end());
                    //
                    // size_t sent_total = 0;
                    // bool start_of_burst = first_trans;
                    // while (sent_total < fl_vec_cf.size() && !trans_worker_.stop_requested()) {
                    //     uhd::tx_metadata_t md;
                    //     md.start_of_burst = start_of_burst;
                    //     md.end_of_burst = false; // 如果没数据的话，会UUUUUU
                    //
                    //     const size_t sent_now = tx_stream_->send(
                    //         &fl_vec_cf[sent_total], fl_vec_cf.size() - sent_total, md, 1.0);
                    //     if (sent_now == 0) {
                    //         SPDLOG_WARN(
                    //             "USRP TX send returned 0, drop remaining {} samples",
                    //             fl_vec_cf.size() - sent_total);
                    //         break;
                    //     }
                    //
                    //     sent_total += sent_now;
                    //     start_of_burst = false;
                    // }
                    //
                    // if (sent_total < fl_vec_cf.size()) {
                    //     SPDLOG_WARN("USRP TX short send: {}/{}", sent_total, fl_vec_cf.size());
                    // }
                    // if (sent_total > 0) {
                    //     first_trans = false;
                    // }
                }
                // // 退出循环后，发送结束标记
                // uhd::tx_metadata_t md_end;
                // md_end.end_of_burst = true;
                // tx_stream_->send("", 0, md_end);
            });

            // recv_worker_.start([&] {
            //     // 接收缓冲区
            //     VecCF buff(recv_samples_);
            //     uhd::rx_metadata_t md;
            //
            //     // 启动连续流
            //     uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
            //     stream_cmd.stream_now = true;
            //     rx_stream_->issue_stream_cmd(stream_cmd);
            //
            //     while (!recv_worker_.stop_requested()) {
            //         const size_t num_rx = rx_stream_->recv(&buff[0], buff.size(), md, 1.0); //测试阶段单fl通道
            //
            //         // 错误处理
            //         if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            //             std::cerr << "[RX] 超时" << std::endl;
            //             continue;
            //         }
            //         if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            //             std::cerr << "[RX] 溢出 (O)" << std::endl;
            //             continue;
            //         }
            //         if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            //             std::cerr << "[RX] 错误: " << md.strerror() << std::endl;
            //             break;
            //         }
            //
            //         if (rx_callback_) {
            //             VecCF data(buff.begin(), buff.begin() + num_rx);
            //             rx_callback_(std::move(data));
            //         }
            //     }
            //     const uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
            //     rx_stream_->issue_stream_cmd(stop_cmd);
            // });
        }

        ~USRP() override {
        }
    private:
        static constexpr int recv_samples_ = 8192;
        // const std::string args_ = "type=b200,serial=192113";   // 也可以加 serial=xxxx 3459f45
        const std::string args_ = "";
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

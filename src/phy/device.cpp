//
// Created by jiaxv on 2026/1/8.
//

#include "phy/device.h"

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>

#include "phy/phy.h"

namespace openldacs::phy::device {
        void USRP::setupDevice() {
                const std::vector channels = {AS, GS}; // FL: 通道0  RL: 通道1

                // ========== 2) 创建 USRP ==========
                uhd::set_thread_priority_safe();
                usrp_ = uhd::usrp::multi_usrp::make(args_);

                // ========== 3) 内部时钟/时间源 ==========
                usrp_->set_clock_source("internal");
                usrp_->set_time_source("internal");

                // ========== 4) 配置采样率、频率、增益（对两个通道分别设置） ==========
                if (direction_ & TX) {
                        usrp_->set_tx_rate(rate_);
                }
                if (direction_ & RX) {
                        usrp_->set_rx_rate(rate_);
                }

                for (const auto ch : channels) {
                        usrp_->set_tx_gain(tx_gain_, ch);
                        usrp_->set_rx_gain(rx_gain_, ch);
                        usrp_->set_tx_antenna("TX/RX", ch);
                        usrp_->set_rx_antenna("RX2", ch);

                        if (ch == AS) {
                                std::cout << "!!!!!!!!1" << std::endl;
                                usrp_->set_tx_freq(uhd::tune_request_t(rl_freq), ch);
                                usrp_->set_rx_freq(uhd::tune_request_t(fl_freq), ch);

                                rl_tx_args.channels = {static_cast<uint32_t>(ch)};
                                fl_rx_args.channels = {static_cast<uint32_t>(ch)};

                                rl_tx_stream = usrp_->get_tx_stream(rl_tx_args);
                                fl_rx_stream = usrp_->get_rx_stream(fl_rx_args);

                                if (!rl_tx_stream || !fl_rx_stream) {
                                        std::cerr << "Failed to get streams for channel " << ch << std::endl;
                                }
                        }else {
                                std::cout << "!!!!!!!!2" << std::endl;
                                usrp_->set_tx_freq(uhd::tune_request_t(fl_freq), ch);
                                usrp_->set_rx_freq(uhd::tune_request_t(rl_freq), ch);

                                fl_tx_args.channels = {static_cast<uint32_t>(ch)};
                                rl_rx_args.channels = {static_cast<uint32_t>(ch)};

                                fl_tx_stream = usrp_->get_tx_stream(fl_tx_args);
                                rl_rx_stream = usrp_->get_rx_stream(rl_rx_args);

                                // if (!fl_tx_stream || !rl_rx_stream) {
                                //         std::cerr << "Failed to get streams for channel " << ch << std::endl;
                                // }
                                //
                                // std::cout << fl_tx_stream->get_max_num_samps() << " " << rl_rx_stream->
                                //                 get_max_num_samps() << std::endl;
                        }

                }

                // usrp_->get_tx_stream()
        }

        void USRP::transThread() {

        }
}

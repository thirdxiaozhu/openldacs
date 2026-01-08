//
// Created by jiaxv on 2026/1/8.
//

#include "phy/device.h"

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>

#include "phy/phy.h"

namespace openldacs::phy::device {
        void USRP::setupDevice() {
                const std::vector channels = {FL, RL}; // FL: 通道0  RL: 通道1

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

                for (auto ch : channels) {
                        usrp_->set_tx_gain(tx_gain_, ch);
                        usrp_->set_rx_gain(rx_gain_, ch);

                        if (ch == FL) {
                                usrp_->set_tx_freq(uhd::tune_request_t(fl_freq), ch);
                                usrp_->set_rx_freq(uhd::tune_request_t(fl_freq), ch);
                        }else {
                                usrp_->set_tx_freq(uhd::tune_request_t(rl_freq), ch);
                                usrp_->set_rx_freq(uhd::tune_request_t(rl_freq), ch);
                        }

                        // 天线口：B210 常见 RX: "RX2" 或 "TX/RX"；TX: "TX/RX"
                        // 具体你接哪个口就设哪个（不确定先用默认也能跑）
                        usrp_->set_tx_antenna("TX/RX", ch);
                        usrp_->set_rx_antenna("RX2", ch);
                }

                usrp_->get_tx_stream()
        }
}

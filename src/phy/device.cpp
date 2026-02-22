//
// Created by jiaxv on 2026/1/8.
//

#include "phy/device.h"

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <algorithm>

#include "phy/phy.h"

namespace openldacs::phy::device {

        void USRP::setupDevice() {
                // uhd::set_thread_priority_safe();
                usrp_ = uhd::usrp::multi_usrp::make(args_);

                usrp_->set_clock_source("internal");
                usrp_->set_time_source("internal");
                usrp_->set_tx_rate(rate_);
                usrp_->set_rx_rate(rate_);

                // std::vector<size_t> channels = {FL_CHANNEL, RL_CHANNEL};
                std::vector<size_t> channels = {FL_CHANNEL};
                const size_t tx_num_channels = usrp_->get_tx_num_channels();
                const size_t rx_num_channels = usrp_->get_rx_num_channels();
                const size_t max_pair_channels = std::min(tx_num_channels, rx_num_channels);
                if (channels.size() > max_pair_channels) {
                        channels.resize(max_pair_channels);
                        SPDLOG_WARN(
                            "USRP has only {} usable TX/RX channel(s), stream channels are reduced.",
                            max_pair_channels);
                }
                if (channels.empty()) {
                        throw std::runtime_error("No usable USRP TX/RX channels found");
                }

                tx_args_.channels = channels;
                rx_args_.channels = channels;
                tx_stream_ = usrp_->get_tx_stream(tx_args_);
                rx_stream_ = usrp_->get_rx_stream(rx_args_);
                for (const auto ch : channels) {
                        const bool use_rl_mapping = ch == RL_CHANNEL;
                        usrp_->set_tx_freq(uhd::tune_request_t(use_rl_mapping ? rl_freq_ : fl_freq_), ch);
                        usrp_->set_rx_freq(uhd::tune_request_t(use_rl_mapping ? rl_freq_ : fl_freq_), ch);
                        usrp_->set_tx_gain(tx_gain_, ch);
                        usrp_->set_rx_gain(rx_gain_, ch);
                        usrp_->set_tx_antenna("TX/RX", ch);
                        usrp_->set_rx_antenna("RX2", ch);
                }
        }
}

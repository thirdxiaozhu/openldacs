//
// Created by jiaxv on 2026/1/8.
//

#include "phy/device.h"

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>

#include "phy/phy.h"

namespace openldacs::phy::device {

        void Device::synchronisation(const itpp::cvec &input) {
                sync_param_.coarseSync(input);
                sync_param_.fineSync(input);
        }

        void USRP::setupDevice() {
                const std::vector<size_t> channels = {AS_CHANNEL, GS_CHANNEL};

                // uhd::set_thread_priority_safe();
                usrp_ = uhd::usrp::multi_usrp::make(args_);

                usrp_->set_clock_source("internal");
                usrp_->set_time_source("internal");
                usrp_->set_tx_rate(rate_);
                usrp_->set_rx_rate(rate_);

                tx_args_.channels = channels;
                rx_args_.channels = channels;
                tx_stream_ = usrp_->get_tx_stream(tx_args_);
                rx_stream_ = usrp_->get_rx_stream(rx_args_);
                usrp_->set_tx_freq(uhd::tune_request_t(rl_freq), AS_CHANNEL);
                usrp_->set_rx_freq(uhd::tune_request_t(fl_freq), AS_CHANNEL);
                usrp_->set_tx_freq(uhd::tune_request_t(fl_freq), GS_CHANNEL);
                usrp_->set_rx_freq(uhd::tune_request_t(rl_freq), GS_CHANNEL);
                for (const auto ch : channels) {
                        usrp_->set_tx_gain(tx_gain_, ch);
                        usrp_->set_rx_gain(rx_gain_, ch);
                        usrp_->set_tx_antenna("TX/RX", ch);
                        usrp_->set_rx_antenna("RX2", ch);
                }
        }
}

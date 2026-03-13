//
// Created by jiaxv on 2026/1/8.
//

#include "phy/device.h"

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <algorithm>
#include <cstdlib>

#include "phy/phy.h"

namespace openldacs::phy::device {

        void USRP::setupDevice() {
                usrp_ = uhd::usrp::multi_usrp::make(args_);

                usrp_->set_clock_source("internal");
                usrp_->set_time_source("internal");
                usrp_->set_tx_rate(rate_);
                usrp_->set_rx_rate(rate_);

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

                auto read_env_double = [](const char* key, const double fallback) {
                        const char* value = std::getenv(key);
                        if (value == nullptr || *value == '\0') {
                                return fallback;
                        }

                        try {
                                return std::stod(value);
                        } catch (const std::exception&) {
                                SPDLOG_WARN("Invalid env {}='{}', fallback to {}", key, value, fallback);
                                return fallback;
                        }
                };

                auto read_env_string = [](const char* key, const char* fallback) {
                        const char* value = std::getenv(key);
                        if (value == nullptr || *value == '\0') {
                                return std::string(fallback);
                        }
                        return std::string(value);
                };

                double rf_analog_bw = 5.0e6; // 5 MHz
                const double tx_gain = read_env_double("OPENLDACS_USRP_TX_GAIN", tx_gain_);
                const double rx_gain = read_env_double("OPENLDACS_USRP_RX_GAIN", rx_gain_);
                const double tx_bw = read_env_double("OPENLDACS_USRP_TX_BW", rf_analog_bw);
                const double rx_bw = read_env_double("OPENLDACS_USRP_RX_BW", rf_analog_bw);
                const std::string tx_ant = read_env_string("OPENLDACS_USRP_TX_ANT", "TX/RX");
                const std::string rx_ant = read_env_string("OPENLDACS_USRP_RX_ANT", "RX2");

                SPDLOG_INFO(
                    "USRP config: rate={} Hz, tx_gain={} dB, rx_gain={} dB, tx_bw={} Hz, rx_bw={} Hz, tx_ant={}, rx_ant={}",
                    rate_,
                    tx_gain,
                    rx_gain,
                    tx_bw,
                    rx_bw,
                    tx_ant,
                    rx_ant);

                for (auto ch : channels) {
                        const bool use_rl_mapping = false;

                        uhd::tune_request_t tune_req(use_rl_mapping ? rl_freq_ : fl_freq_, lo_offset);
                        usrp_->set_tx_freq(tune_req, ch);
                        usrp_->set_rx_freq(tune_req, ch);

                        usrp_->set_rx_dc_offset(true, ch);
                        usrp_->set_rx_iq_balance(true, ch);

                        usrp_->set_tx_gain(tx_gain, ch);
                        usrp_->set_rx_gain(rx_gain, ch);
                        usrp_->set_tx_bandwidth(tx_bw, ch);
                        usrp_->set_rx_bandwidth(rx_bw, ch);
                        usrp_->set_tx_antenna(tx_ant, ch);
                        usrp_->set_rx_antenna(rx_ant, ch);
                }


                SPDLOG_INFO("--- USRP 硬件状态自检 ---");
                // 1. 检查天线
                SPDLOG_INFO("实际 TX 天线: {}", usrp_->get_tx_antenna(0));
                SPDLOG_INFO("实际 RX 天线: {}", usrp_->get_rx_antenna(0));

                // 2. 检查增益
                SPDLOG_INFO("实际 TX 增益: {} dB", usrp_->get_tx_gain(0));
                SPDLOG_INFO("实际 RX 增益: {} dB", usrp_->get_rx_gain(0));

                // 3. 检查频率
                SPDLOG_INFO("实际 TX 频率: {} Hz", usrp_->get_tx_freq(0));
                SPDLOG_INFO("实际 RX 频率: {} Hz", usrp_->get_rx_freq(0));

                // 4. 检查本振锁定状态 (等待一小会儿让PLL稳定)
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                auto tx_sensor = usrp_->get_tx_sensor("lo_locked", 0);
                auto rx_sensor = usrp_->get_rx_sensor("lo_locked", 0);
                SPDLOG_INFO("TX 本振锁定: {}", tx_sensor.to_bool() ? "YES" : "NO");
                SPDLOG_INFO("RX 本振锁定: {}", rx_sensor.to_bool() ? "YES" : "NO");
                SPDLOG_INFO("--------------------------");
        }
}

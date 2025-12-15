//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"
#include "util/util.h"


namespace openldacs::phy::link::fl {
    using namespace openldacs::util;

    void FLChannelHandler::set_data_pilot_index() {

    }

    void BC1_3Handler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }


    void BC1_3Handler::init_frame_pattern()  {
    }

    void BC2Handler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }

    void BC2Handler::init_frame_pattern()  {
    }

    void FLDataHandler::handle(const std::vector<uint8_t> &input) const {
        spdlog::info(n_fl_ofdm_symb_);
        std::cout << input;
    }

    void FLDataHandler::init_frame_pattern()  {
        frame_pattern_ = Eigen::MatrixXi::Ones(n_fft, n_fl_ofdm_symb_);
        frame_pattern_.col(pos_sync1).setZero();
        frame_pattern_.col(pos_sync2).setZero();

        // guards
        frame_pattern_.block(0, pos_sync1, guard_left, n_fl_ofdm_symb_).setZero();
        frame_pattern_.block(n_fft - guard_right, 0, guard_right, n_fl_ofdm_symb_).setZero();

        // middle
        frame_pattern_.row(n_fft/2).setZero();

        // pilots 1
        for (const int i : pilot_set0) {
            frame_pattern_(i, 2) = 2;
        }

        // pilots 2-53
        for (int i = 0 ; i < n_fl_ofdm_symb_ - 4; i++) {
            for (const int s : config_.pilot_sets[i % 5]) {
                frame_pattern_(s, i+3) = 2;
            }
        }

        // pilots 54
        for (const int i : pilot_set6) {
            frame_pattern_(i, n_fl_ofdm_symb_ - 1) = 2;
        }

        std::cout << frame_pattern_ << std::endl;
    }

    PhyFl::PhyFl() {
    }

    std::unique_ptr<FLChannelHandler> PhyFl::make_handler(const ChannelType type) const {
        switch (type) {
            case ChannelType::BC1_3:   return std::make_unique<BC1_3Handler>(config_);
            case ChannelType::BC2:     return std::make_unique<BC2Handler>(config_);
            case ChannelType::FL_DATA: return std::make_unique<FLDataHandler>(config_);
            default: throw std::runtime_error("Unknown FLType");
        }
    }

    void PhyFl::process_packet(ChannelType type, const std::vector<uint8_t> &input) const {
        auto handler = make_handler(type);
        handler->handle(input);

    }

}
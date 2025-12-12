//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"
#include "util/util.h"


namespace openldacs::phy::fl {
    using namespace openldacs::util;


    void BC1_3Handler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }

    void BC2Handler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }

    void FLDataHandler::handle(const std::vector<uint8_t> &input) const {
        spdlog::info(n_fl_ofdm_symb_);
        std::cout << input;
    }

    Eigen::MatrixXi FLDataHandler::init_frame_pattern() {
        Eigen::MatrixXi frame_pattern = Eigen::MatrixXi::Ones(n_fft, n_fl_ofdm_symb_);
        std::cout << frame_pattern;
        return frame_pattern;
    }

    PhyFl::PhyFl() {
    }

    void PhyFl::process_fl_pkt(FLType type, const std::vector<uint8_t> &input) const {

        auto handler = make_handler(type);
        handler->handle(input);

    }

}
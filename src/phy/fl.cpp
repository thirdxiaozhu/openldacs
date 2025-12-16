//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"
#include "phy/fl.h"
#include "util/util.h"


namespace openldacs::phy::link::fl {
    using namespace openldacs::util;

    void BC1_3Handler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }


    void BC2Handler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }

    void FLDataHandler::handle(const std::vector<uint8_t> &input) const {
        std::cout << input;
    }

    void FLDataHandler::compose_frame(const PhyFl::FLConfig &config, ParamStruct::FrameInfo& frame_info) {
        Eigen::MatrixXi& pattern = frame_info.frame_pattern;

        pattern = Eigen::MatrixXi::Ones(n_fft, n_fl_ofdm_symb_);
        pattern.col(pos_sync1).setZero();
        pattern.col(pos_sync2).setZero();

        // guards
        pattern.block(0, pos_sync1, guard_left, n_fl_ofdm_symb_).setZero();
        pattern.block(n_fft - guard_right, 0, guard_right, n_fl_ofdm_symb_).setZero();

        // middle
        pattern.row(n_fft/2).setZero();

        // pilots 1
        for (const int i : pilot_set0) {
            pattern(i, 2) = static_cast<int>(SymbolValue::PILOT);
        }

        // pilots 2-53
        for (int i = 0 ; i < n_fl_ofdm_symb_ - 4; i++) {
            for (const int s : config.pilot_sets[i % 5]) {
                pattern(s, i+3) = static_cast<int>(SymbolValue::PILOT);
            }
        }

        // pilots 54
        for (const int i : pilot_set6) {
            pattern(i, n_fl_ofdm_symb_ - 1) = static_cast<int>(SymbolValue::PILOT);
        }

        find_Xi(frame_info.data_ind, pattern, static_cast<int>(SymbolValue::DATA));
        find_Xi(frame_info.pilot_ind, pattern, static_cast<int>(SymbolValue::PILOT));
        frame_info.n_data = frame_info.data_ind.size();
        frame_info.n_pilot = frame_info.pilot_ind.size();

        //  indices for whole packages
        // for data
        Eigen::Map<const Eigen::VectorXi> data_ind_eigen(frame_info.data_ind.data(), static_cast<int>(frame_info.n_data));
        Eigen::RowVectorXi frame_offset = n_fft * n_fl_ofdm_symb_ * Eigen::RowVectorXi::LinSpaced(n_frames_, 0, n_frames_ - 1);
        frame_info.data_ind_packet = data_ind_eigen.replicate(1, n_frames_).rowwise() + frame_offset;

        // for pilot
        Eigen::Map<const Eigen::VectorXi> pilot_ind_eigen(frame_info.pilot_ind.data(), static_cast<int>(frame_info.n_pilot));
        frame_info.pilot_ind_packet = pilot_ind_eigen.replicate(1, n_frames_).rowwise() + frame_offset;

        // for sync
        const int n_sync = sync_ind1.size() + sync_ind2.size();
        std::vector<int> sync_combined;
        sync_combined.insert(sync_combined.end(), sync_ind1.begin(), sync_ind1.end());
        sync_combined.insert(sync_combined.end(), sync_ind2.begin(), sync_ind2.end());
        Eigen::Map<const Eigen::VectorXi> sync_ind_eigen(sync_combined.data(), n_sync);
        frame_info.sync_ind_packet = sync_ind_eigen.replicate(1, n_frames_).rowwise() + frame_offset;
    }

    void FLDataHandler::set_pilots_sync_symbol(const PhyFl::FLConfig &config, ParamStruct::FrameInfo& frame_info) {

    }

    void FLDataHandler::build_frame_info(const PhyFl::FLConfig &config, ParamStruct::FrameInfo& frame_info) {
        compose_frame(config, frame_info);
        set_pilots_sync_symbol(config, frame_info);
    }

    FLChannelHandler::ParamStruct FLDataHandler::build_params()  {
        ParamStruct params;
        build_frame_info(config, params.frame_info_);

        return params;
    }

    FLChannelHandler &PhyFl::get_handler(const ChannelType type) const {
        switch (type) {
            case ChannelType::BC1_3:   return *bc13_;
            case ChannelType::BC2:     return *bc2_;
            case ChannelType::FL_DATA: return *data_;
            default: throw std::runtime_error("Unknown FLType");
        }
    }

    void PhyFl::process_packet(const ChannelType type, const std::vector<uint8_t> &input) const {
        const auto& handler = get_handler(type);
        handler.handle(input);
    }

}
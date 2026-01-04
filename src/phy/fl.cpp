//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"
#include "util/util.h"

#include <itpp/itcomm.h>
#include <correct.h>

namespace openldacs::phy::link::fl {
    using namespace openldacs::util;


    void FLChannelHandler::buildFrameInfo() {
        composeFrame();
        setPilotsSyncSymbol();
    }

    void FLChannelHandler::buildParams()  {
        buildFrameInfo();
    }

    FLChannelHandler& PhyFl::getHandler(const CHANNEL type) const {
        switch (type) {
            case BCCH1_3:   return *bc13_;
            case BCCH2:     return *bc2_;
            case CCCH:
            case FL_DCH: return *data_;
            default: throw std::runtime_error("Unknown FLType");
        }
    }

    void FLChannelHandler::randomizer(MVecU8 &to_process, const CodingParams &coding_params) {
        for (int i = 0; i < to_process.size(); ++i) {
            for (int j = 0; j < to_process[i].size(); ++j) {
                to_process[i][j] ^= coding_params.randomize_mvec[i][j];
            }
        }
    }

    void FLChannelHandler::rsEncoder(MVecU8 &to_process, const CodingParams &coding_params) {
        for (int i = 0; i < to_process.size(); ++i) {
            std::vector<uint8_t> output(coding_params.rs_params.n);
            coding_params.rs_params.rs.rsEncode(to_process[i], output);
            to_process[i].assign(output.begin(), output.end());

            // for (int j = 0; j < to_process[i].size(); ++j) {
            //     std::cout << to_process[i][j] << " ";
            // }
            // std::cout << std::endl;
        }
    }


    void BC1_3Handler::submit(const std::vector<uint8_t> &input, CHANNEL ch, CMS cms) const {
        std::cout << input;
    }

    void BC1_3Handler::submit(const std::vector<uint8_t> &input, CHANNEL ch) const {
        std::cout << input;
    }


    void BC2Handler::submit(const std::vector<uint8_t> &input, CHANNEL ch, CMS cms) const {
        std::cout << input;
    }

    void BC2Handler::submit(const std::vector<uint8_t> &input, CHANNEL ch) const {
        std::cout << input;
    }

    void FLDataHandler::submit(const std::vector<uint8_t> &input, CHANNEL ch, CMS cms) const {
        const CodingParams &coding_params = coding_table_.getCodingParams({cms, 3});
        if (input.size() != coding_params.bytes_per_pdu * coding_params.joint_frame) {
            throw std::runtime_error("Input size does not match coding params");
        }

        MVecU8 mvec_input;
        mvec_input.resize(coding_params.joint_frame);
        for (int i = 0; i < coding_params.joint_frame; i++) {
            mvec_input[i].assign(input.begin() + i * coding_params.bytes_per_pdu, input.begin() + (i + 1) * coding_params.bytes_per_pdu);
        }


        // for (const auto& row : vec_input2) {
        //     for (const auto& val : row) {
        //         std::cout << static_cast<int>(val) << " ";
        //     }
        //     std::cout << std::endl;
        // }

        randomizer(mvec_input, coding_params);
        rsEncoder(mvec_input, coding_params);
    }

    void FLDataHandler::submit(const std::vector<uint8_t> &input, const CHANNEL ch) const {
        switch (ch) {
            case CCCH:
                submit(input, FL_DCH, CMS::QPSK_R12);
            case FL_DCH:
                submit(input, FL_DCH, default_cms_);
                break;
            default:
                throw std::runtime_error("Unsupported channel type in FLDATAHandlr");
        }
    }

    void FLDataHandler::composeFrame() {
        FrameInfo &frame_info = params_.frame_info_;

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
            for (const int s : pilot_sets[i % 5]) {
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

    void FLDataHandler::setPilotsSyncSymbol() {
        FrameInfo &frame_info = params_.frame_info_;

        std::vector<cd>& seed = frame_info.pilot_seeds;
        std::vector<std::vector<cd>>& sync_symbols = frame_info.sync_symbols;
        // 估个上限容量，尽量减少扩容次数
        // 最粗略的上界：seed1 + (N_ofdm_symb-4)*4 + seed7
        seed.clear();
        seed.reserve(4 + (std::max(0, static_cast<int>(n_fl_ofdm_symb_) - 4) * 4) + 14);

        // seed1
        seed.insert(seed.end(),
                                 pilot_seed0.begin(), pilot_seed0.end());

        for (int i = 0; i < n_fl_ofdm_symb_ - 4; i++) {
            seed.insert(seed.end(), pilot_seeds[i % 5].begin(), pilot_seeds[i % 5].end());
        }

        seed.insert(seed.end(),
                                 pilot_seed6.begin(), pilot_seed6.end());

        {
            constexpr int N2 = 12;
            constexpr int M2 = 5;
            const double boost_factor2 = std::sqrt(4);
            const std::complex<double> W_N2 = std::exp(std::complex<double>(0, 2 * M2 * M_PI / N2));

            std::vector<double>pow2;
            for (int i = 0; i < N2; i++) {
                pow2.push_back(std::pow(i, 2) / 2.0);
            }

            // Initialize s2 as the boost_factor times W_N raised to the power of 'pow'
            std::vector<cd> s2;
            for (int i = 0; i < N2; ++i) {
                s2.push_back(boost_factor2 * std::pow(W_N2, pow2.at(i)));
            }

            sync_symbols.push_back(s2);
        }

        {
            constexpr int N1 = 24;
            constexpr int M1 = 1;
            const double boost_factor1 = std::sqrt(2);
            const std::complex<double> W_N1 = std::exp(std::complex<double>(0, 2 * M1 * M_PI / N1));
            std::vector<double>pow1;
            for (int i = 0; i < N1; i++) {
                pow1.push_back(std::pow(i, 2) / 2.0);
            }

            std::vector<cd> s1;
            for (int i = 0; i < N1; ++i) {
                s1.push_back(boost_factor1 * std::pow(W_N1, pow1.at(i)));
            }

            sync_symbols.push_back(s1);
        }

    }


    void PhyFl::processPacket(const CHANNEL ch, const std::vector<uint8_t> &input) const {
        FLChannelHandler& handler = getHandler(ch);

        handler.submit(input, ch);
    }

}
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

    void FLChannelHandler::randomizer(VecU8 &to_process, const CodingParams &coding_params) {

        if (to_process.size() != coding_params.bytes_per_pdu) {
            throw std::runtime_error("Input size does not match coding params");
        }

        for (int i = 0; i < to_process.size(); ++i) {
            to_process[i] ^= coding_params.randomize_vec[i];
        }
    }

    RsEncodedUnit FLChannelHandler::rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params) {

        RsEncodedUnit unit;
        unit.sdu_index = index;

        if (to_process.size() != coding_params.rs_params.k) {
            throw std::runtime_error("Input size does not match reed-solomon params");
        }

        std::vector<uint8_t> output(coding_params.rs_params.n);
        coding_params.rs_params.rs.rsEncode(to_process, output);
        unit.rs_bytes.assign(output.begin(), output.end());

        return unit;
    }

    VecU8 FLChannelHandler::blockInterleaver(const std::vector<RsEncodedUnit> &units,
                                             const CodingParams &coding_params) {
        const size_t rows = coding_params.joint_frame * coding_params.pdu_per_frame;
        const size_t cols = units[0].rs_bytes.size();

        std::vector<uint8_t> out;
        out.reserve(rows * cols);

        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                out.push_back(units[j].rs_bytes[i]);
            }
        }

        return out;
    }

    itpp::bvec FLChannelHandler::convCode(const VecU8 &input, const CodingParams &coding_params) const {
        const itpp::bvec bits_vec = bytesToBitsMSB(input);
        itpp::bvec bits_output;
        coding_params.cc.encode_tail(bits_vec, bits_output);   // === 等价 convenc(bits_bef_cod, trellis, punc_pat) :contentReference[oaicite:5]{index=5}

        bits_output.set_size(frame_info_.n_data * coding_params.bits_per_symb * coding_params.joint_frame, true);

        // std::cout << bits_vec.size() << " " << bits_output.size() <<std::endl; // 原始bit长度 + 6个0
        return bits_output;
    }

    itpp::bvec FLChannelHandler::helicalInterleaver(const itpp::bvec &input, const CodingParams &coding_params) {
        const int a = coding_params.h_inter_params.a;
        const int b = coding_params.h_inter_params.b;

        if (a <= 0 || b <= 0) {
            throw std::runtime_error("Invalid interleaver parameters");
        }
        const int N = a * b;

        if (input.size() != N) {
            throw std::runtime_error("Input size does not match helical interleaver parameters");
        }

        itpp::bvec out(N);

        for (int l = 0; l < a; ++l) {
            for (int n = 0; n < b; ++n) {
                const int k = l * b + n;                       // input index
                const int m = b * ((3 * n + l) % a) + n;       // output index
                out[m] = input[k];
            }
        }

        return out;
    }

    itpp::cvec FLChannelHandler::modulate(BlockBuffer &block, const CodingParams &coding_params) {
        switch (coding_params.mod_type) {
            case ModulationType::QPSK: {
                itpp::QPSK qpsk;
                return qpsk.modulate_bits(block.coded_bits);
            }
            case ModulationType::QAM16: {
                itpp::QAM qam16(16);
                return qam16.modulate_bits(block.coded_bits);
            }
            case ModulationType::QAM64: {
                itpp::QAM qam64(64);
                return qam64.modulate_bits(block.coded_bits);
            }
        }

        throw std::runtime_error("Unknown ModulationType");
    }

    itpp::cmat FLChannelHandler::subcarrier_allocation() {
        itpp::cmat data_matrix;

        return data_matrix;
    }


    void BC1_3Handler::submit(const PhySdu sdu, CMS cms) {
        std::cout << sdu.payload;
    }

    void BC1_3Handler::submit(const PhySdu sdu) {
        std::cout << sdu.payload;
    }


    void BC2Handler::submit(const PhySdu sdu, CMS cms) {
        std::cout << sdu.payload;
    }

    void BC2Handler::submit(const PhySdu sdu) {
        std::cout << sdu.payload;
    }

    void FLDataHandler::submit(const PhySdu sdu, CMS cms) {
        const CodingParams &coding_params = coding_table_.getCodingParams({cms, sdu.channel==CCCH ? 3 : 2});
        if (sdu.payload.size() != coding_params.bytes_per_pdu) {
            throw std::runtime_error("Input size does not match coding params");
        }

        VecU8 to_process = sdu.payload;

        randomizer(to_process, coding_params);
        RsEncodedUnit unit = rsEncoder(to_process, sdu.sdu_index, coding_params);

        {
            // lock
            std::lock_guard<std::mutex> lk(block_m_);

            const BlockKey key(sdu);

            const size_t int_count = getInterleaverCount(sdu);
            auto &buf = block_map_[key];
            if (buf.units.empty()) {
                buf.interleaver_count = int_count;
                buf.is_cc = sdu.channel == CCCH;
            }

            if (buf.interleaver_count != int_count) {
                throw std::runtime_error("Interleaver count does not match");
            }

            buf.units.push_back(std::move(unit));

            if (buf.units.size() == buf.interleaver_count) {
                BlockBuffer ready = std::move(buf);
                block_map_.erase(key);

                channelCoding(ready, coding_params);
                // SPDLOG_WARN("{}", ready.coded_bits.length());
                itpp::cvec mod = modulate(ready, coding_params); // 长度应该是一个ofdm frame的data symbol长度的两倍
                itpp::cmat tx_frames = subcarrier_allocation();

                // SPDLOG_WARN("{}", mod.length());
            }
            // unlock
        }
    }

    void FLDataHandler::submit(const PhySdu sdu) {
        switch (sdu.channel) {
            case CCCH:
                submit(sdu, CMS::QPSK_R12);
            case FL_DCH:
                if (sdu.acm_id == 0) {
                    submit(sdu, default_cms_);
                }else {
                    // user-specific channel coding
                }
                break;
            default:
                throw std::runtime_error("Unsupported channel type in FLDATAHandlr");
        }
    }

    void FLDataHandler::channelCoding(BlockBuffer &block, const CodingParams &coding_params) {

        std::ranges::sort(block.units,
                          [](const RsEncodedUnit& a, const RsEncodedUnit& b){
                              return a.sdu_index < b.sdu_index;
                          });

        // for (RsEncodedUnit u: block.units) {
        //     std::cout << u.rs_bytes  << std::endl;
        // }

        const VecU8 after_int = blockInterleaver(block.units, coding_params);

        // 字节层面转为bit层面
        const itpp::bvec conv_bits = convCode(after_int, coding_params);

        const itpp::bvec helical_bits = helicalInterleaver(conv_bits, coding_params);

        block.coded_bits = helical_bits;
    }


    void FLDataHandler::composeFrame() {
        Eigen::MatrixXi& pattern = frame_info_.frame_pattern;

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

        find_Xi(frame_info_.data_ind, pattern, static_cast<int>(SymbolValue::DATA));
        find_Xi(frame_info_.pilot_ind, pattern, static_cast<int>(SymbolValue::PILOT));
        frame_info_.n_data = frame_info_.data_ind.size();
        frame_info_.n_pilot = frame_info_.pilot_ind.size();

        //  indices for whole packages
        // for data
        Eigen::Map<const Eigen::VectorXi> data_ind_eigen(frame_info_.data_ind.data(), static_cast<int>(frame_info_.n_data));
        Eigen::RowVectorXi frame_offset = n_fft * n_fl_ofdm_symb_ * Eigen::RowVectorXi::LinSpaced(n_frames_, 0, n_frames_ - 1);
        frame_info_.data_ind_packet = data_ind_eigen.replicate(1, n_frames_).rowwise() + frame_offset;

        // for pilot
        Eigen::Map<const Eigen::VectorXi> pilot_ind_eigen(frame_info_.pilot_ind.data(), static_cast<int>(frame_info_.n_pilot));
        frame_info_.pilot_ind_packet = pilot_ind_eigen.replicate(1, n_frames_).rowwise() + frame_offset;

        // for sync
        const int n_sync = sync_ind1.size() + sync_ind2.size();
        std::vector<int> sync_combined;
        sync_combined.insert(sync_combined.end(), sync_ind1.begin(), sync_ind1.end());
        sync_combined.insert(sync_combined.end(), sync_ind2.begin(), sync_ind2.end());
        Eigen::Map<const Eigen::VectorXi> sync_ind_eigen(sync_combined.data(), n_sync);
        frame_info_.sync_ind_packet = sync_ind_eigen.replicate(1, n_frames_).rowwise() + frame_offset;
    }

    void FLDataHandler::setPilotsSyncSymbol() {
        {
            std::vector<cd>& seed = frame_info_.pilot_seeds;
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
        }

        std::vector<std::vector<cd>>& sync_symbols = frame_info_.sync_symbols;
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


    void PhyFl::processPacket(const PhySdu &sdu) const {
        FLChannelHandler& handler = getHandler(sdu.channel);

        handler.submit(sdu);
    }

}
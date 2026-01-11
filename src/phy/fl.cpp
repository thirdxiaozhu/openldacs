//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"
#include "util/util.h"

#include <itpp/itcomm.h>
#include <correct.h>

namespace openldacs::phy::link::fl {
    using namespace openldacs::util;

    void FLChannelHandler::buildFrame()  {
        getFrameIndices();
        calcSequences();
        composeFrame();
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

    itpp::cvec FLChannelHandler::modulate(BlockBuffer &block, const ModulationType mod_type) {
        switch (mod_type) {
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

    itpp::cmat FLChannelHandler::matrix_ifft(const itpp::cmat &to_process) {
        itpp::cmat result(to_process.rows(), to_process.cols());

        for (int i = 0; i < result.cols(); ++i) {
            itpp::cvec col = to_process.get_col(i);
            result.set_col(i, itpp::ifft(col));
        }

        return result;
    }

    itpp::cmat FLChannelHandler::matrix_fft(const itpp::cmat &to_process) {
        itpp::cmat result(to_process.rows(), to_process.cols());

        for (int i = 0; i < result.cols(); ++i) {
            itpp::cvec col = to_process.get_col(i);
            result.set_col(i, itpp::fft(col));
        }

        return result;
    }

    itpp::cvec FLChannelHandler::windowing(const itpp::cmat &to_process, const int joint_frame) {
        const int mat_cols = to_process.cols();
        const int frame_symbols = mat_cols / joint_frame;

        itpp::cvec result((n_fft + n_cp) * mat_cols);

        if (mat_cols % joint_frame != 0) {
            throw std::runtime_error("Matrix cols must be divisible by joint_frame");
        }

        if constexpr (n_g <= 0 || n_cp <= 0 || n_ws <= 0) {
            throw std::runtime_error("OFDM params must be positive");
        }

        if constexpr (n_g > n_fft) {
            throw std::runtime_error("N_g (CP length) must be <= Nfft");
        }

        if constexpr (n_cp > n_fft) {
            throw std::runtime_error("N_cp must be <= Nfft.");
        }

        // 循环前缀的，guard symbol
        itpp::cmat mat_with_cp(n_fft + n_g, mat_cols);
        {
            itpp::cmat tail_g = to_process.get_rows(n_fft - n_g, n_fft-1);
            mat_with_cp.set_rows(0, tail_g);
            mat_with_cp.set_rows(n_g, to_process);
        }

        // window 前缀 后缀
        itpp::cmat window_prefix = to_process.get_rows(n_fft - n_cp, n_fft - n_g -1);
        itpp::cmat window_postfix = to_process.get_rows(0, n_ws-1);

        // 前缀乘数
        itpp::vec ramp_up(n_ws);
        for (int i = 0; i < n_ws; ++i) {
            ramp_up(i) = 0.5 + 0.5 * std::cos(M_PI + (M_PI * i) / static_cast<double>(n_ws) );
        }
        // 后缀乘数
        itpp::vec ramp_down = fliplrRowvec(ramp_up);

        for (int c = 0; c < window_prefix.cols(); ++c) {
            for (int r = 0; r < window_postfix.rows(); ++r) {
                window_prefix(r, c) *= ramp_up(r);
                window_postfix(r, c) *= ramp_down(r);
            }
        }

        for (int i = 0; i < joint_frame; i++) {
            const int offset = i * frame_symbols;
            itpp::cmat window_part(n_ws, frame_symbols);

            window_part.set_col(0, window_prefix.get_col(0 + offset ));
            for (int col = 1; col < frame_symbols; ++col) {
                itpp::cvec sum = window_postfix.get_col((col - 1) + offset) + window_prefix.get_col(col + offset);
                window_part.set_col(col, sum);
            }


            itpp::cmat frame_matrix(n_fft + n_cp, frame_symbols);
            frame_matrix.set_rows(0, window_part);
            frame_matrix.set_rows(n_ws, mat_with_cp.get_cols(0 + offset, (frame_symbols - 1) + offset));

            // std::cout << frame_matrix << " " << frame_matrix.size() << std::endl << std::endl;
            result.set_subvector(((n_fft + n_cp) * frame_symbols) * i, itpp::cvectorize(frame_matrix));

        }

        std::cout << result.size() << std::endl;
        // std::cout << result << std::endl << std::endl;
        return result;
    }


    itpp::cmat FLDataHandler::subcarrier_allocation(const itpp::cvec &input, const int joint_frame) {
        int input_ind = 0;

        if (input.size() != frame_info_.n_data * joint_frame) {
            throw std::runtime_error("Input size does not match frame info in subcarrier allocation");
        }

        // 创建最终的大矩阵，列数为 coding_params.joint_frame，行数与单个 frame 相同
        itpp::cmat result_matrix(n_fft, frame_info_.frame.cols() * joint_frame);
        result_matrix.zeros();

        for (int i = 0; i < joint_frame; ++i) {
            itpp::cmat frame_matrix = frame_info_.frame;
            for (int j = 0; j < frame_info_.n_data; j++) {
                frame_matrix(frame_info_.data_ind[j]) = input(input_ind++);
            }

            // 将当前帧矩阵复制到结果矩阵的对应列范围内
            result_matrix.set_cols(i * frame_matrix.cols(), frame_matrix);
        }
        return result_matrix;
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
                const itpp::cvec mod = modulate(ready, coding_params.mod_type); // 长度应该是一个ofdm frame的data symbol长度的两倍
                // dump_constellation(mod, "/home/jiaxv/ldacs/openldacs/dump/mod.dat");

                itpp::cmat frames_freq = subcarrier_allocation(mod, coding_params.joint_frame);
                itpp::cmat frames_time = matrix_ifft(frames_freq);
                // dump_ofdm_mag_per_symbol(frames_freq, "/home/jiaxv/ldacs/openldacs/dump/freqmag");

                const itpp::cvec tx_vec = windowing(frames_time, coding_params.joint_frame);

                device_->sendData(tx_vec, sdu.channel == CCCH ? Priority::HIGH : Priority::NORMAL);

                // itpp::cmat frames_freq2 = matrix_fft(frames_time);
                // itpp::cmat diff = frames_freq2 - frames_freq;
                // // 最大绝对误差（看最坏点）
                // double max_err = max(max(abs(diff)));
                // std::cout << "max_err = " << max_err << "\n";
                // std::cout << std::endl;
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

        const VecU8 after_int = blockInterleaver(block.units, coding_params);

        // 字节层面转为bit层面
        const itpp::bvec conv_bits = convCode(after_int, coding_params);

        const itpp::bvec helical_bits = helicalInterleaver(conv_bits, coding_params);

        block.coded_bits = helical_bits;
    }


    void FLDataHandler::getFrameIndices() {
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

        // for sync
        frame_info_.sync_ind.insert(frame_info_.sync_ind.end(), sync_ind1.begin(), sync_ind1.end());
        frame_info_.sync_ind.insert(frame_info_.sync_ind.end(), sync_ind2.begin(), sync_ind2.end());
    }

    void FLDataHandler::calcSequences() {
        // pilot symbol
        {
            itpp::cvec& seed = frame_info_.pilot_seeds;
            seed.set_size(static_cast<int>(frame_info_.n_pilot));

            int idx = 0;

            // seed1 - pilot_seed0
            for (const auto current_pilot_seed : pilot_seed0) {
                seed(idx++) = current_pilot_seed;
            }

            for (int i = 0; i < n_fl_ofdm_symb_ - 4; i++) {
                const std::vector<cd>& current_pilot_seeds = pilot_seeds[i % 5];
                for (auto current_pilot_seed : current_pilot_seeds) {
                    seed(idx++) = current_pilot_seed;
                }
            }

            // seed7 - pilot_seed6
            for (const auto current_pilot_seed : pilot_seed6) {
                seed(idx++) = current_pilot_seed;
            }
        }

        // sync symbol2
        {

            const int N1 = static_cast<int>(frame_info_.n_sync1);
            itpp::cvec &sync_symbols1 = frame_info_.sync_symbols1;
            sync_symbols1.set_size(N1);

            for (int k = 0; k < N1; ++k) {
                const std::complex<double> W_N1 = std::exp(std::complex<double>(0.0, 5 * M_PI * k * k / N1));
                sync_symbols1(k) = std::sqrt(4) * W_N1;
            }
        }

        // sync symbol1
        {
            const int N2 = static_cast<int>(frame_info_.n_sync2);
            itpp::cvec &sync_symbols2 = frame_info_.sync_symbols2;
            sync_symbols2.set_size(N2);

            for (int k = 0; k < N2; ++k) {
                const std::complex<double> W_N2 = std::exp(std::complex<double>(0.0,  M_PI * k * k / N2));
                sync_symbols2(k) = std::sqrt(2) * W_N2;
            }
        }

    }

    void FLDataHandler::composeFrame() {
        itpp::cmat &frame = frame_info_.frame;

        frame.set_size(n_fft, n_fl_ofdm_symb_);
        frame.zeros();

        for (int i = 0; i < frame_info_.pilot_ind.size(); i++) {
            frame(frame_info_.pilot_ind[i]) = frame_info_.pilot_seeds[i];
        }

        for (int i = 0; i < sync_ind1.size(); i++) {
            frame(sync_ind1[i]) = frame_info_.sync_symbols1[i];
        }

        for (int i = 0; i < sync_ind2.size(); i++) {
            frame(sync_ind2[i]) = frame_info_.sync_symbols2[i];
        }

    }


    void PhyFl::processPacket(const PhySdu &sdu) const {
        FLChannelHandler& handler = getHandler(sdu.channel);

        handler.submit(sdu);
    }

}
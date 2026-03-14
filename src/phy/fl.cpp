//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"

#include <filesystem>
#include <fstream>
#include <sstream>


namespace openldacs::phy::link::fl {

    using namespace phy::params;
    using namespace util;

    namespace {
        itpp::cmat extractDataTimeWithDelta(const itpp::cvec &input,
                                           const std::vector<double> &t_fine,
                                           const std::vector<double> &f_fine,
                                           const int ofdm_symb,
                                           const int upsample_rate,
                                           const int fft_delta) {
            const int cp_sample = config::n_cp * upsample_rate;
            const int fft_sample = config::n_fft * upsample_rate;
            const int symb_span = cp_sample + fft_sample;
            const int frame_length_td = symb_span * ofdm_symb;
            const int num_frames = static_cast<int>(t_fine.size());
            const std::complex<double> J(0.0, 1.0);
            const double norm_factor = static_cast<double>(fft_sample);

            itpp::cmat data_time(fft_sample, num_frames * ofdm_symb);
            data_time.zeros();

            const itpp::vec t_vec = itpp::linspace(0, frame_length_td - 1, frame_length_td);
            for (int i = 0; i < num_frames; ++i) {
                int t_current = static_cast<int>(std::round(t_fine[i]));
                const double f_current = i < static_cast<int>(f_fine.size()) ? f_fine[i] : 0.0;

                int start_idx = t_current - cp_sample;
                int end_idx = start_idx + frame_length_td - 1;
                itpp::cvec data_extract;

                if (start_idx < 0) {
                    data_extract = concat(data_extract, itpp::zeros_c(-start_idx));
                    start_idx = 0;
                }

                if (end_idx >= input.length()) {
                    end_idx = input.length() - 1;
                }

                if (start_idx <= end_idx) {
                    data_extract = concat(data_extract, input.mid(start_idx, end_idx - start_idx + 1));
                }

                const int missing = frame_length_td - data_extract.size();
                if (missing > 0) {
                    data_extract = concat(data_extract, itpp::zeros_c(missing));
                }

                const itpp::cvec freq_cor_fac = itpp::exp(itpp::to_cvec(-2.0 * itpp::pi * f_current * t_vec / norm_factor) * J);
                const itpp::cvec data_freq_comp = elem_mult(data_extract, freq_cor_fac);

                for (int s = 0; s < ofdm_symb; ++s) {
                    const int sym_start = s * symb_span;
                    const int valid_start = sym_start + cp_sample + fft_delta;
                    const int global_col = i * ofdm_symb + s;

                    if (valid_start < 0 || valid_start + fft_sample > data_freq_comp.size()) {
                        data_time.set_col(global_col, itpp::zeros_c(fft_sample));
                        continue;
                    }

                    data_time.set_col(global_col, data_freq_comp.mid(valid_start, fft_sample));
                }
            }

            return data_time;
        }

        std::string deltaLabel(const int delta) {
            std::ostringstream oss;
            if (delta >= 0) {
                oss << 'p' << delta;
            } else {
                oss << 'm' << -delta;
            }
            return oss.str();
        }
    }

    FLChannelHandler& PhyFl::getHandler(const ChannelSlot type) const {
        switch (type) {
            case BCCH1_3:   return *bc13_;
            case BCCH2:     return *bc2_;
            case CCCH_DCH:
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

    std::vector<VecU8> FLChannelHandler::derandomizer(const std::vector<VecU8> &input,
                                                      const CodingParams &coding_params) {
        std::vector<VecU8> output;
        for (int i = 0; i < input.size(); ++i) {
            VecU8 to_process = input[i];
            randomizer(to_process, coding_params);
            output.push_back(to_process);
        }
        return output;
    }

    RsEncodedUnit FLChannelHandler::rsEncoder(const VecU8 &to_process, uint8_t index, const CodingParams &coding_params) {
        RsEncodedUnit unit;
        unit.sdu_index = index;
        if (!coding_params.rs_params.has_value()) {
            throw std::runtime_error("Reed-Solomon unsupported in this channel");
        }

        const RSCoderParams rs_params = coding_params.rs_params.value();

        if (to_process.size() != rs_params.k * coding_params.rs_per_pdu) {
            throw std::runtime_error("Input size does not match reed-solomon params");
        }

        for (int i = 0; i < coding_params.rs_per_pdu; i++) {
            std::vector<uint8_t> output(rs_params.n);

            VecU8 sub(to_process.begin() + i * rs_params.k, to_process.begin() + (i + 1) * rs_params.k);

            rs_params.rs.rsEncode(sub, output);
            unit.rs_bytes.insert(unit.rs_bytes.end(), output.begin(), output.end());

        }

        return unit;
    }

    std::vector<VecU8> FLChannelHandler::rsDecoder(const itpp::imat &input, const CodingParams &coding_params) {
        std::vector<VecU8> output;

        if (!coding_params.rs_params.has_value()) {
            throw std::runtime_error("Reed-Solomon unsupported in this channel");
        }

        const RSCoderParams rs_params = coding_params.rs_params.value();

        if (input.cols() != rs_params.n * coding_params.rs_per_pdu) {
            throw std::runtime_error("Input size does not match reed-solomon params");
        }

        for (int i = 0; i < input.rows(); ++i) {
            VecU8 input_vec;
            for (int j = 0; j < input.cols(); ++j) {
                input_vec.push_back(input(i, j));
            }

            VecU8 output_vec;
            for (int r = 0; r < coding_params.rs_per_pdu; r++) {
                VecU8 input_sub(input_vec.begin() + r * rs_params.n, input_vec.begin() + (r + 1) * rs_params.n);

                VecU8 output_sub(rs_params.k);
                try {
                    rs_params.rs.rsDecode(input_sub, output_sub);
                } catch (const std::exception& e) {
                    SPDLOG_WARN("RS decode failed at row {}: {}", r, e.what());
                    std::copy_n(input_vec.begin() + r * rs_params.n, output_vec.size(), output_vec.begin());
                }
                output_vec.insert(output_vec.end(), output_sub.begin(), output_sub.end() );
            }
            output.push_back(output_vec);
        }

        return output;
    }

    itpp::ivec FLChannelHandler::blockInterleaver(const std::vector<RsEncodedUnit> &units,
                                                  const CodingParams &coding_params) {
        const size_t rows = coding_params.joint_frame * coding_params.n_pdus;
        const size_t cols = units[0].rs_bytes.size();

        itpp::ivec out(rows * cols);

        int count = 0;
        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                out(count++) = units[j].rs_bytes[i];
            }
        }

        return out;
    }

    itpp::imat FLChannelHandler::blockDeinterleaver(const itpp::ivec &input, const CodingParams &coding_params) {
        const size_t rows = coding_params.joint_frame * coding_params.n_pdus;
        const size_t cols = input.size() / rows;

        itpp::imat out = itpp::reshape(input, rows, cols);

        return out;
    }


    itpp::bvec FLChannelHandler::convCode(const itpp::ivec &input, const CodingParams &coding_params) const {
        const itpp::bvec bits_vec = bytesToBitsMSB(input);
        itpp::bvec bits_output;

        coding_params.cc.encode_tail(bits_vec, bits_output);   // === 等价 convenc(bits_bef_cod, trellis, punc_pat) :contentReference[oaicite:5]{index=5}
        bits_output.set_size(frame_info_.n_data * coding_params.bits_per_symb * coding_params.joint_frame, true);

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
                out(m) = input(k);
            }
        }

        return out;
    }

    itpp::vec FLChannelHandler::helicalDeinterleaver(const itpp::vec &in, const CodingParams &p) {
        const int a = p.h_inter_params.a, b = p.h_inter_params.b;
        const int N = a * b;
        if (in.size() != N) throw std::runtime_error("LLR size mismatch");

        itpp::vec out(N);
        for (int l = 0; l < a; ++l) {
            for (int n = 0; n < b; ++n) {
                const int k = l * b + n; // 原始位置
                const int m = b * ((3 * n + l) % a) + n; // 交织后位置
                out(k) = in(m); // 逆操作
            }
        }
        return out;
    }


    void FLChannelHandler::matrixIfft(BlockBuffer &block) {
        block.frame_time = itpp::cmat(block.frames_freq.rows(), block.frames_freq.cols());

        for (int i = 0; i < block.frame_time.cols(); ++i) {
            itpp::cvec col = block.frames_freq.get_col(i);
            block.frame_time.set_col(i, itpp::ifft(col));
        }
    }

    itpp::cmat FLChannelHandler::matrixFft(const itpp::cmat &to_process) {
        itpp::cmat result(to_process.rows(), to_process.cols());

        for (int i = 0; i < result.cols(); ++i) {
            itpp::cvec col = to_process.get_col(i);
            result.set_col(i, itpp::fft(col));
        }

        return result;
    }

    itpp::cmat FLChannelHandler::downsamplingFreq(const itpp::cmat &signal, const int downsample) {
        const int num_rows = signal.rows();

        double scale_factor = 1.0 / std::sqrt(static_cast<double>(downsample));

        double use_len_d = static_cast<double>(num_rows) / downsample;
        if (num_rows % downsample != 0) {
            it_error("Downsampling factor should be a divisor of the signal length");
        }

        const int use_len = static_cast<int>(use_len_d);
        const int use_len1 = static_cast<int>(std::floor(use_len / 2.0));

        const int start_idx = (num_rows / 2) - use_len1;
        const int end_idx = start_idx + use_len - 1;

        // 边界安全检查
        if (start_idx < 0 || end_idx >= num_rows) {
            it_error("Calculated indices are out of bounds.");
        }

        itpp::cmat sig_down = signal.get_rows(start_idx, end_idx);
        const std::complex<double> scale_c(scale_factor, 0.0);

        return sig_down * scale_c;
    }

    void BC1_3Handler::submit(const PhySdu sdu) {
        const CodingParams &coding_params = coding_table_.getCodingParams({
            CMS::QPSK_R12, 1
        });
        processData(sdu, coding_params);
    }

    void BC2Handler::submit(const PhySdu sdu) {
        const CodingParams &coding_params = coding_table_.getCodingParams({
            CMS::QPSK_R12, 1
        });
        processData(sdu, coding_params);
    }

    void FLChannelHandler::processData(const PhySdu& sdu, const CodingParams &coding_params) {
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
                buf.is_cc = sdu.channel == CCCH_DCH;
            }
            if (buf.interleaver_count != int_count) {
                throw std::runtime_error("Interleaver count does not match");
            }

            buf.units.push_back(std::move(unit));

            if (buf.units.size() == buf.interleaver_count) {
                BlockBuffer block = std::move(buf);
                block_map_.erase(key);

                channelCoding(block, coding_params);
                modulate(block, coding_params.mod_type); // 长度应该是一个ofdm frame的data symbol长度的两倍

                subcarrierAllocation(block, coding_params.joint_frame);
                matrixIfft(block);
                config_.sink_.enqueue(block, sdu.channel);
            }
        }
    }

    void FLDataHandler::submit(const PhySdu sdu) {
        switch (sdu.channel) {
            case CCCH_DCH: {
                const CodingParams &coding_params = coding_table_.getCodingParams({
                    CMS::QPSK_R12, 3
                });
                processData(sdu, coding_params);
            }
            break;
            case FL_DCH: {
                if (sdu.acm_id == 0) {
                    const CodingParams &coding_params = coding_table_.getCodingParams({
                        getCms(), 2
                    });
                    processData(sdu, coding_params);
                } else {
                    // user-specific channel coding
                }
            }
            break;
            default:
                throw std::runtime_error("Unsupported channel type in FLDATAHandlr");
        }
    }

    void FLChannelHandler::channelCoding(BlockBuffer &block, const CodingParams &coding_params) const {

        std::ranges::sort(block.units,
                          [](const RsEncodedUnit& a, const RsEncodedUnit& b){
                              return a.sdu_index < b.sdu_index;
                          });

        const itpp::ivec block_int = blockInterleaver(block.units, coding_params);

        // 字节层面转为bit层面
        const itpp::bvec conv_bits = convCode(block_int, coding_params);

        const itpp::bvec helical_bits = helicalInterleaver(conv_bits, coding_params);

        block.coded_bits = helical_bits;
    }

    void FLChannelHandler::subcarrierAllocation(BlockBuffer &block, const int joint_frame) {
        int input_ind = 0;

        if (block.mod_vec.size() != frame_info_.n_data * joint_frame) {
            throw std::runtime_error("Input size does not match frame info in subcarrier allocation");
        }

        // 创建最终的大矩阵，列数为 coding_params.joint_frame，行数与单个 frame 相同
        block.frames_freq = itpp::cmat(n_fft, frame_info_.frame.cols() * joint_frame);
        block.frames_freq.zeros();

        for (int i = 0; i < joint_frame; ++i) {
            itpp::cmat frame_matrix = frame_info_.frame;
            for (int j = 0; j < frame_info_.n_data; j++) {
                frame_matrix(frame_info_.data_ind[j]) = block.mod_vec(input_ind++);
            }

            // 将当前帧矩阵复制到结果矩阵的对应列范围内
            block.frames_freq.set_cols(i * frame_matrix.cols(), frame_matrix);
        }
    }


    void FLChannelHandler::dumpFftOffsetSweepDebug(const itpp::cvec& input) {
        const auto &t_coarse_dbg = f_sync.lastTCoarse();
        const auto &f_coarse_dbg = f_sync.lastFCoarse();
        const auto &t_fine_dbg = f_sync.lastTFine();
        const auto &f_fine_dbg = f_sync.lastFFine();

        if (t_fine_dbg.empty()) {
            SPDLOG_WARN("Skip FFT offset debug dump because t_fine is empty");
            return;
        }

        std::filesystem::create_directories("dump");

        {
            std::ofstream ofs("dump/sync_debug.csv");
            ofs << "frame,t_coarse,f_coarse,t_fine,f_fine\n";
            const size_t n = t_fine_dbg.size();
            for (size_t i = 0; i < n; ++i) {
                const double tc = i < t_coarse_dbg.size() ? t_coarse_dbg[i] : 0.0;
                const double fc = i < f_coarse_dbg.size() ? f_coarse_dbg[i] : 0.0;
                const double tf = i < t_fine_dbg.size() ? t_fine_dbg[i] : 0.0;
                const double ff = i < f_fine_dbg.size() ? f_fine_dbg[i] : 0.0;
                ofs << i << ',' << tc << ',' << fc << ',' << tf << ',' << ff << '\n';
            }
        }

        for (int delta = -4; delta <= 4; ++delta) {
            const itpp::cmat data_time_dbg = extractDataTimeWithDelta(
                input,
                t_fine_dbg,
                f_fine_dbg,
                ofdm_symb_,
                f_sync.sync.upsample_rate,
                delta
            );

            const itpp::cmat data_freq_up_dbg = matrixFft(data_time_dbg);
            const itpp::cmat data_freq_dbg = downsamplingFreq(data_freq_up_dbg, f_sync.sync.upsample_rate);
            const itpp::cmat chan_coeff_dbg = channel_est_.channelEst(data_freq_dbg, frame_info_.pos_sync2);

            itpp::cmat data_equ_dbg;
            itpp::mat sigma2_dbg;
            equalizer_.equalize(data_freq_dbg, chan_coeff_dbg, data_equ_dbg, sigma2_dbg);

            const std::string suffix = deltaLabel(delta);
            util::dump_cmat_constellation(data_equ_dbg, "dump/constellation_equalized_delta_" + suffix + ".dat");
            util::dump_cmat_constellation(data_freq_dbg, "dump/constellation_fft_delta_" + suffix + ".dat");
        }

        SPDLOG_INFO("Dumped FFT offset sweep debug files to dump/constellation_* and dump/sync_debug.csv");
    }


    void FLChannelHandler::recvHandler(const itpp::cvec& input, const std::vector<double> &t_coarse, const std::vector<double> &f_coarse, const CodingParams &params) {
        itpp::cmat data_time;

        f_sync.synchronisation(input, t_coarse, f_coarse, data_time);
        const itpp::cmat data_freq_up = matrixFft(data_time);
        const itpp::cmat data_freq = downsamplingFreq(data_freq_up, f_sync.sync.upsample_rate);

        const itpp::cmat chan_coeff_mat = channel_est_.channelEst(data_freq, frame_info_.pos_sync2);

        itpp::cmat data_equ;
        itpp::mat sigma2_sum;
        equalizer_.equalize(data_freq, chan_coeff_mat, data_equ, sigma2_sum);

        if (debug_fft_offset_dump_count_ == 0) {
            std::filesystem::create_directories("dump");
            util::dump_cmat_constellation(data_equ, "dump/constellation_equalized_delta_p0_main.dat");
            dumpFftOffsetSweepDebug(input);
            debug_fft_offset_dump_count_++;
        }


        //--------------------- [ZMQ 实时数据发送模块] -----------------------
        // 将 itpp::cmat (double) 转换为 std::complex<float> 的 std::vector
        {
            itpp::cvec data_equ_vec = itpp::cvectorize(data_equ); // 先展平成一维向量
            std::vector<std::complex<float>> gr_buffer;
            gr_buffer.reserve(data_equ_vec.length());

            for (int i = 0; i < data_equ_vec.length(); ++i) {
                // 取出有效的数据符号 (如果你的 data_equ 里包含了不需要画图的导频，可以在这里滤除)
                gr_buffer.emplace_back(
                    static_cast<float>(std::real(data_equ_vec(i))),
                    static_cast<float>(std::imag(data_equ_vec(i)))
                );
            }

            // 通过 ZMQ 发送这段内存
            zmq::message_t zmq_msg(gr_buffer.data(), gr_buffer.size() * sizeof(std::complex<float>));
            config_.zmq_pub_cons_.send(zmq_msg, zmq::send_flags::dontwait); // non-blocking 发送
        }
        // -----------------------------------------------------------------

        const itpp::mat demod = demodulate(data_equ, sigma2_sum, params.mod_type); // 临时参数
        itpp::vec LLR_int = itpp::cvectorize(demod);

        if (LLR_int.size() != params.h_inter_params.int_bits_size_) {
            SPDLOG_ERROR("unmatched size for helical inteleaver");
            return;
        }

        itpp::vec deint = helicalDeinterleaver(LLR_int, params);
        deint.set_size(deint.size() - params.conv_params.pad_bits_after_cc, true);

        itpp::bvec vit_dec = params.cc.decode_tail(deint);

        if (vit_dec.size() % params.rs_params.value().bits_after_rs ) {
            throw std::runtime_error("unmatched size for rs decoder");
        }

        itpp::ivec block_int = bitsToBytesMSB(vit_dec);

        itpp::imat rs_coded = blockDeinterleaver(block_int, params);
        std::vector<VecU8> random = rsDecoder(rs_coded, params);
        std::vector<VecU8> data = derandomizer(random, params);

        std::cout << data << std::endl;
    }


    void PhyFl::processPacket(const PhySdu &sdu) const {
        FLChannelHandler& handler = getHandler(sdu.channel);
        handler.submit(sdu);
    }

    itpp::cvec PhySink::windowing(const BlockBuffer &block) {
        const itpp::cmat &to_process = block.frame_time;
        const int mat_cols = to_process.cols();

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

        itpp::cmat window_part(n_ws, mat_cols);

        if (!prev_post_) {
            window_part.set_col(0, window_prefix.get_col(0));
        }else {
            window_part.set_col(0, window_prefix.get_col(0) + prev_post_.value());
        }
        prev_post_.emplace(window_postfix.get_col(mat_cols - 1));

        for (int col = 1; col < mat_cols; ++col) {
            itpp::cvec sum = window_postfix.get_col(col - 1) + window_prefix.get_col(col);
            window_part.set_col(col, sum);
        }

        itpp::cmat frame_mat(n_fft + n_cp, mat_cols);
        frame_mat.set_rows(0, window_part);
        frame_mat.set_rows(n_ws, mat_with_cp.get_cols(0, mat_cols - 1));

        itpp::cvec frames_symbols(frame_mat.size());
        frames_symbols.set_subvector(0, itpp::cvectorize(frame_mat));

        return frames_symbols;
    }

}

//
// Created by 邹嘉旭 on 2025/12/10.
//

#include "phy/fl.h"
#include "util/util.h"

#include <itpp/itcomm.h>
#include <correct.h>

namespace openldacs::phy::link::fl {

    FLChannelHandler& PhyFl::getHandler(const CHANNEL type) const {
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

    void FLChannelHandler::modulate(BlockBuffer &block, const ModulationType mod_type) {
        switch (mod_type) {
            case ModulationType::QPSK: {
                itpp::QPSK qpsk;
                block.mod_vec = qpsk.modulate_bits(block.coded_bits);
                return;
            }
            case ModulationType::QAM16: {
                itpp::QAM qam16(16);
                block.mod_vec = qam16.modulate_bits(block.coded_bits);
                return;
            }
            case ModulationType::QAM64: {
                itpp::QAM qam64(64);
                block.mod_vec = qam64.modulate_bits(block.coded_bits);
                return;
            }
        }

        throw std::runtime_error("Unknown ModulationType");
    }

    void FLChannelHandler::matrix_ifft(BlockBuffer &block) {
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

    itpp::cmat FLChannelHandler::channelEst(const itpp::cmat &input) {
        itpp::cmat ce_pil = channel_coeff_pil(input);
        line_int_2d(ce_pil);
        return input;
    }

    itpp::cmat FLChannelHandler::channel_coeff_pil(const itpp::cmat &input) {
        int num_frames = input.size() / ofdm_symb_ / n_fft;
        int sync_pos = -1;

        // 判断信道是否存在同步符号 ,在FL试验阶段直接就true
        if (1) {
            sync_pos = pos_sync2;
        }
        itpp::cmat chan_coeff_mat = itpp::zeros_c(n_fft, ofdm_symb_ * num_frames);

        for (int i = 0; i < num_frames; ++i) {
            // --- 提取一帧数据 ---
            // MATLAB: data_sync(:, frame_count*N_ofdm_symb+1:(frame_count+1)*N_ofdm_symb)
            // IT++: get_cols(start_col, end_col) 使用 0-based 索引
            const int start_col = i * ofdm_symb_;
            const int end_col = (i + 1) * ofdm_symb_ - 1;
            itpp::cmat data_frame = input.get_cols(start_col, end_col);

            for (int k = 0; k < frame_info_.pilot_ind.size(); ++k) {
                // 获取 MATLAB 风格的线性索引 (假设是 1-based)
                int mat_linear_idx = frame_info_.pilot_ind[k];


                // 计算行列坐标 (MATLAB 是列优先 Column-Major)
                int r = mat_linear_idx % n_fft; // 行索引
                int c = mat_linear_idx / n_fft; // 列索引 (相对于当前帧)

                // 提取接收到的导频值
                std::complex<double> rec_val = data_frame(r, c);
                std::complex<double> sent_val = frame_info_.pilot_seeds(k);

                // LS 估计: H = Y / X
                std::complex<double> h_est;

                if (std::abs(sent_val) > 1e-9) {
                    h_est = rec_val / sent_val;
                } else {
                    h_est = 0.0;
                }

                // 防止除零错误 (Zero-Forcing safety)
                if (std::abs(h_est) == 0.0) {
                    h_est = std::complex<double>(1e-5, 0.0);
                }

                // 存入大矩阵 (chan_coeff_mat)
                // 注意列坐标要加上当前帧的偏移量
                chan_coeff_mat(r, start_col + c) = h_est;
            }

            // --- 处理同步符号 (Sync Symbols) ---
            if (sync_pos >= 0) {
                // MATLAB: sync_symbols(13:end). 对应 C++ index 12 到最后
                // 我们假设 sync_ind2 也是针对 data_frame 的线性索引

                for (int k = 0; k < sync_ind2.size(); ++k) {
                    const int mat_idx = sync_ind2[k];

                    // 计算相对索引，复刻 MATLAB 逻辑:
                    // rec_ref(frame_info.sync_ind2-(pos_sync_pi-1)*N_fft)
                    // 这实际上是在计算 sync_ind2 对应在 这一列(pos_sync_pi) 中的行号
                    // 但简单起见，我们直接解析 sync_ind2 的位置

                    int r = mat_idx % n_fft;
                    int c = mat_idx / n_fft; // 这一步应该等于 (pos_sync_pi - 1)

                    std::complex<double> rec_ref_val = data_frame(r, c);

                    // 获取对应的发送符号
                    // MATLAB: sync_symbols(13:end) -> index k 对应 sync_symbols[12 + k]
                    // 注意：这里需要确保 sync_symbols 长度足够
                    std::complex<double> sent_ref_val = frame_info_.sync_symbols(12 + k);

                    std::complex<double> h_ref = rec_ref_val / sent_ref_val;

                    if (std::abs(h_ref) == 0.0) {
                        h_ref = std::complex<double>(1e-5, 0.0);
                    }

                    // 存入大矩阵
                    // 位置: 当前帧的起始列 + 同步符号所在的列(pos_sync_pi - 1)
                    chan_coeff_mat(r, start_col + sync_pos) = h_ref;
                }
            }
        }

        return chan_coeff_mat;
    }

    void FLChannelHandler::line_int_2d(const itpp::cmat &input) {
        SPDLOG_INFO("{} {} {}", input.size(), input.rows(), input.cols());
        // TODO: 判断rl
        //     if strcmp(transmission, 'rl')
        //          N_ofdm_symb = N_ofdm_symb*ce_param.N_tile_joint/2;
        //     end

        int num_frames = input.cols() / ofdm_symb_;

        for (int i = 0; i < num_frames; ++i) {
            const int col_offset = num_frames * ofdm_symb_;
            for (int k = 0; k < channel_est_.data_ind.size(); i++) {
                const auto& pilot_ind_aux = channel_est_.pilot_ind_cell[k];
                const auto& pilot_dist_aux = channel_est_.pilot_dist_cell[k];

                std::complex<double> num(0.0, 0.0);
                double den = 0.0;
                double w_sum = 0.0;

            }
        }


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
        const CodingParams &coding_params = coding_table_.getCodingParams({cms, sdu.channel==CCCH_DCH ? 3 : 2});
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
                // dump_constellation(mod, "/home/jiaxv/ldacs/openldacs/dump/mod.dat");

                subcarrier_allocation(block, coding_params.joint_frame);
                matrix_ifft(block);

                config_.sink_.enqueue(block, sdu.channel);

                // dump_ofdm_mag_per_symbol(frames_freq, "/home/jiaxv/ldacs/openldacs/dump/freqmag");


                // for (const auto &vec : tx_vecs) {
                //     device_->sendData(vec, sdu.channel == CCCH ? Priority::HIGH : Priority::NORMAL);
                // }
                //
                // // 测试接收
                // itpp::cvec recv_data = tx_vecs[0]; //后面要给他扩大随机长度，所有扩大的位置用零填充
                // synchronisation(recv_data);

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
            case CCCH_DCH:
                submit(sdu, CMS::QPSK_R12);
                break;
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



    void FLDataHandler::subcarrier_allocation(BlockBuffer &block, const int joint_frame) {
        int input_ind = 0;

        if (block.mod_vec.size() != frame_info_.n_data * joint_frame) {
            throw std::runtime_error("Input size does not match frame info in subcarrier allocation");
        }

        // 创建最终的大矩阵，列数为 coding_params.joint_frame，行数与单个 frame 相同
        // itpp::cmat result_matrix(n_fft, frame_info_.frame.cols() * joint_frame);
        // result_matrix.zeros();
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


    void PhyFl::processPacket(const PhySdu &sdu) const {
        FLChannelHandler& handler = getHandler(sdu.channel);
        handler.submit(sdu);
    }

    itpp::cvec PhySink::windowing(const BlockBuffer &block) {

        const itpp::cmat &to_process = block.frame_time;

        const int mat_cols = to_process.cols();
        // const int frame_symbols = mat_cols / joint_frame;


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
        frame_mat.set_rows(n_ws, mat_with_cp.get_cols(0, mat_cols));

        itpp::cvec frames_symbols(frame_mat.size());
        frames_symbols.set_subvector(0, itpp::cvectorize(frame_mat));

        return frames_symbols;
    }

}
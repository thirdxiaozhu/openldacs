//
// Created by 邹嘉旭 on 2025/12/20.
//

#include "phy/params.h"

#include <filesystem>
#include <fstream>
#include <itpp/base/matfunc.h>
#include <itpp/base/specmat.h>
#include <itpp/base/svec.h>

#include "phy/fl.h"

namespace openldacs::phy::params {
    using namespace openldacs::util;

    CodingParams CodingTable::setCodingParams(CodingKey key, CHANNEL ch) const {
        CodingParams params = get_initial_coding_param(key, ch);

        auto [cms, joint_frame] = key;

        SPDLOG_WARN("\n\n========= {} = {} ==========", static_cast<int>(cms), joint_frame);

        // constrain length is 7
        itpp::ivec gen(2);
        gen(0) = 0171; // G1 = 171oct
        gen(1) = 0133; // G2 = 133oct

        params.cc.set_generator_polynomials(gen, params.L);                                 // 相当于 poly2trellis 的“码定义”部分
        params.cc.set_method(itpp::Trunc);                                                   // 不自动加尾
        params.cc.set_puncture_matrix(util::puncpatToMatrix2output(params.puncpat));


        SPDLOG_INFO("N_symbols: {}; N_bits_per_symbol: {}； N_frame_tile_joint: {}", frame_info.n_data, params.bits_per_symb,joint_frame);
        int bits_coded_frame = frame_info.n_data * params.bits_per_symb * joint_frame;
        SPDLOG_INFO("N_bits_coded_frame: {}; N_cc_cod: {}；", bits_coded_frame, params.cc_cod);
        int bits_with_pad = bits_coded_frame / params.cc_cod;
        SPDLOG_INFO("N_bits_with_pad: {}; b: {}； a: {}", bits_with_pad, params.b, params.a);
        int bits_with_term = std::floor(bits_with_pad / params.b) * params.a;
        SPDLOG_INFO("N_bits_with_term: {}; term_bits: {}", bits_with_term, params.term_bits);
        int bits_before_cc = bits_with_term - params.term_bits;
        int bits_before_cc_frame = bits_before_cc * params.cc_cod;

        // interleaver
        params.int_size = bits_with_pad * params.interleaver;
        SPDLOG_INFO("int size: {}; N_int: {}", params.int_size, params.interleaver);
        params.h_inter_params.pattern = util::interleaveHelical(params.int_size, params.h_inter_params.a, params.h_inter_params.b);

        // rs params
        SPDLOG_INFO("N_bits_before_cc_frame: {}; N_bits_after_RS: {}", bits_before_cc_frame, params.rs_params.bits_after_rs);

        //conv coding params
        params.conv_params.bits_before_cc = params.rs_params.bits_after_rs * 3 * joint_frame;
        const double bits_coded_double = static_cast<double>((params.conv_params.bits_before_cc + params.term_bits)) / (static_cast<double>(params.a) / (static_cast<double>(params.b)));
        params.conv_params.bits_coded = std::ceil(bits_coded_double);
        SPDLOG_INFO("N_bits_before_cc: {}; N_pad_coded: {}", params.conv_params.bits_before_cc, params.conv_params.bits_coded);
        params.conv_params.pad_bits_after_rs = 0;
        params.conv_params.pad_bits_after_cc = bits_with_pad - params.conv_params.bits_coded;
        SPDLOG_INFO("N_pad_bits_after_rs: {}; N_pad_bits_after_cc: {}", params.conv_params.pad_bits_after_rs, params.conv_params.pad_bits_after_cc);

        params.rate_cod = (static_cast<double>(params.a) / (static_cast<double>(params.b)) * (static_cast<double>(params.rs_params.k) / (static_cast<double>(params.rs_params.n))));
        params.bits_per_pdu = params.rs_params.bits_uncoded * params.rs_per_pdu;
        params.bytes_per_pdu = params.bits_per_pdu >> 3;
        SPDLOG_INFO("rate_cod: {}; bits_per_sdu: {}; bytes_per_pdu: {}", params.rate_cod, params.bits_per_pdu, params.bytes_per_pdu);

        params.randomize_vec.resize(params.bytes_per_pdu);
        const uint8_t* src = random_output.data();
        params.randomize_vec.assign(
            src,
            src + params.bytes_per_pdu
        );

        return params;
    }

    void CodingTable::initCodingTable(const std::initializer_list<CodingKey> keys, CHANNEL ch) {
        for (const auto& key : keys) {
            coding_table.try_emplace(key, setCodingParams(key, ch));
        }
    }

    void SyncParam::frameSync(const itpp::cvec &input) {
        const int up_corr_len1 =  upsample_rate * corr_len1;
        const int up_corr_diff1 =  upsample_rate * corr_diff1;
        const int up_corr_len2 =  upsample_rate * corr_len2;
        const int up_corr_diff2 =  upsample_rate * corr_diff2;
        const int sync_offset = (config::n_g + config::n_ws / 4) * upsample_rate;

        itpp::vec pre_M1;
        itpp::vec pre_angle_metric1;
        itpp::vec pre_M2;
        itpp::vec pre_angle_metric2;

        syncCorrelation(input, up_corr_len1, up_corr_diff1, pre_M1, pre_angle_metric1);
        syncCorrelation(input, up_corr_len2, up_corr_diff2, pre_M2, pre_angle_metric2);

        M1.set_length(pre_M1.length());
        M1.zeros();
        M1.set_subvector(sync_offset, pre_M1.left(pre_M1.length() - sync_offset));

        angle1.set_length(pre_angle_metric1.length());
        angle1.zeros();
        angle1.set_subvector(sync_offset, pre_angle_metric1.left(pre_angle_metric1.length() - sync_offset));

        M2.set_length(pre_M2.length());
        M2.zeros();
        M2.set_subvector(sync_offset, pre_M2.left(pre_M2.length() - sync_offset));

        // itpp::vec v = abs(M1);
        // std::filesystem::create_directories("dump");
        // std::ofstream ofs("dump/corr_peak.csv");
        // for (int i = 0; i < v.length(); ++i) {
        //     ofs << i << "," << v(i) << "\n";
        // }


        angle2.set_length(pre_angle_metric2.length());
        angle2.zeros();
        angle2.set_subvector(sync_offset, pre_angle_metric2.left(pre_angle_metric2.length() - sync_offset));

        freq1 = static_cast<double>(config::n_fft * upsample_rate) / corr_diff1 / (2 * M_PI) * angle1;
        freq2 = static_cast<double>(config::n_fft * upsample_rate) / corr_diff2 / (2 * M_PI) * angle2;

    }

    void SyncParam::findPeaks(std::vector<int> &peak_indices, std::vector<double> &peak_values) {
        // std::vector<int> peak_indices;
        // std::vector<double> peak_values;
        const int min_dist = static_cast<int>(std::round(0.75 * (config::n_fft + config::n_cp) * upsample_rate));

        std::vector<int> peak_ind_all;
        for (int i = 0; i < M1.size(); ++i) {
            if (M1(i) > threshold_peak) {
                peak_ind_all.push_back(i);
            }
        }

        while (peak_ind_all.size() > 0) {
            int start = peak_ind_all[0];
            int end = std::min(start + min_dist, M1.size() - 1);

            double peak_value = M1(start);
            int peak_ind = start;
            for (int i = start; i <= end; ++i) {
                if (M1(i) > peak_value) {
                    peak_value = M1(i);
                    peak_ind = i;
                }
            }

            peak_indices.push_back(peak_ind);
            peak_values.push_back(peak_value);

            // 往后查找
            std::vector<int> filtered;
            filtered.reserve(peak_ind_all.size());
            for (int idx : peak_ind_all) {
                if (idx > end) {
                    filtered.push_back(idx);
                }
            }
            peak_ind_all.swap(filtered);
        }
        //
        // for (int i = 0; i < peak_indices.size(); ++i) {
        //     std::cout << peak_indices[i] << " " << peak_values[i] << std::endl;
        // }
        // std::cout << std::endl;

    }

    void SyncParam::findReliablePeak(std::vector<int> &peak_indices, std::vector<double> &peak_values, double &reliable_peak, double &peak_freq) {
        constexpr double relation_value = 0.7;
        const int nom_dist = (config::n_fft + config::n_cp) * upsample_rate;
        const int tol = config::n_cp * upsample_rate;

        double peak_value1 = 0.0;
        int peak_ind1 = 0;
        getPeak(M1, peak_indices[0]-tol+nom_dist, peak_indices[0]+tol+nom_dist, peak_value1, peak_ind1);
        double peak_value2 = 0.0;
        int peak_ind2 = 0;
        getPeak(M1, peak_indices[0]-tol-nom_dist, peak_indices[0]+tol-nom_dist, peak_value2, peak_ind2);

        // SPDLOG_INFO("{} {}", peak_value2, peak_ind2);

        // both peaks above the threshold
        if (peak_indices.size() > 1 && std::abs(peak_indices[1] - peak_indices[0] - nom_dist) < tol) {
            // look for peak of second correlation
            double peak_value3 = 0.0;
            int peak_ind3 = 0;
            getPeak(M2, peak_indices[0]-tol, peak_indices[0]+tol, peak_value3, peak_ind3);

            //   time synchronization
            reliable_peak = std::round(
                (peak_indices[0] * peak_values[0] + (peak_indices[1] - nom_dist) * peak_values[1] + peak_ind3 *
                 peak_value3) / (peak_values[0] + peak_values[1] + peak_value3));

            //   freq synchronization
            peak_freq = (freq1(peak_indices[0]) * peak_values[0] + freq1(peak_indices[1]) * peak_values[1] +
                                freq2(peak_ind3) * peak_value3) / (peak_values[0] + peak_values[1] + peak_value3);

            if (peak_indices.size() >= 2) {
                peak_indices.erase(peak_indices.begin(), peak_indices.begin() + 2);
            }
            if (peak_values.size() >= 2) {
                peak_values.erase(peak_values.begin(), peak_values.begin() + 2);
            }

            // second peak above relation_value*first peak at +nom_dist
        }else if (peak_indices.size() > 1 && peak_value1 > relation_value * peak_values[0]) {
            double peak_value3 = 0.0;
            int peak_ind3 = 0;
            getPeak(M2, peak_indices[0]-tol, peak_indices[0]+tol, peak_value3, peak_ind3);

            //   time synchronization
            reliable_peak = std::round(
                (peak_indices[0] * peak_values[0] + (peak_ind1 - nom_dist) * peak_value1 + peak_ind3 *
                 peak_value3) / (peak_values[0] + peak_value1 + peak_value3));

            //   freq synchronization
            peak_freq = (freq1(peak_indices[0]) * peak_values[0] + freq1(peak_indices[1]) * peak_value1 +
                                freq2(peak_ind3) * peak_value3) / (peak_values[0] + peak_value1 + peak_value3);

            if (!peak_indices.empty()) {
                peak_indices.erase(peak_indices.begin(), peak_indices.begin() + 1);
            }
            if (!peak_values.empty()) {
                peak_values.erase(peak_values.begin(), peak_values.begin() + 1);
            }
        }else if (peak_value2 > relation_value * peak_values[0]) {

            double peak_value3 = 0.0;
            int peak_ind3 = 0;
            getPeak(M2, peak_ind2-tol, peak_ind2+tol, peak_value3, peak_ind3);

            //   time synchronization
            reliable_peak = std::round(
                ((peak_indices[0] - nom_dist) * peak_values[0] + peak_ind2 * peak_value2 + peak_ind3 *
                 peak_value3) / (peak_values[0] + peak_value2 + peak_value3));

            //   freq synchronization
            peak_freq = (freq1(peak_indices[0]) * peak_values[0] + freq1(peak_ind2) * peak_value2 +
                                freq2(peak_ind3) * peak_value3) / (peak_values[0] + peak_value2 + peak_value3);

            if (!peak_indices.empty()) {
                peak_indices.erase(peak_indices.begin(), peak_indices.begin() + 1);
            }
            if (!peak_values.empty()) {
                peak_values.erase(peak_values.begin(), peak_values.begin() + 1);
            }

        }else {
            double peak_value3 = 0.0;
            int peak_ind3 = 0;
            getPeak(M2, peak_indices[0]-tol, peak_indices[0]+tol, peak_value3, peak_ind3);

            if (peak_value3 > relation_value * peak_values[0] || peak_value3 > threshold_peak) {
                //   time synchronization
                reliable_peak = std::round(
                    (peak_indices[0] * peak_values[0] + peak_ind3 * peak_value3) / (
                        peak_values[0] + peak_value3));

                //   freq synchronization
                peak_freq = (freq1(peak_indices[0]) * peak_values[0] +
                                    freq2(peak_ind3) * peak_value3) / (peak_values[0] + peak_value3);
            }else {
                getPeak(M2, peak_indices[0] - tol - nom_dist, peak_indices[0] + tol - nom_dist, peak_value3, peak_ind3);
                if (peak_value3 > relation_value * peak_values[0] ||  peak_value3 > threshold_peak) {

                    //   time synchronization
                    reliable_peak = std::round(
                        ((peak_indices[0] - nom_dist) * peak_values[0] + peak_ind1*peak_value1 + peak_ind3 * peak_value3) / (
                            peak_values[0] + peak_value1 + peak_value3));

                    //   freq synchronization
                    peak_freq = (freq1(peak_indices[0]) * peak_values[0] + freq1(peak_ind1) * peak_value1 +
                                        freq2(peak_ind3) * peak_value3) / (peak_values[0] + peak_value1 + peak_value3);
                }else {
                    reliable_peak = peak_indices[0] - nom_dist;
                    peak_freq = freq1(peak_indices[0]);
                }
            }
            if (!peak_indices.empty()) {
                peak_indices.erase(peak_indices.begin(), peak_indices.begin() + 1);
            }
            if (!peak_values.empty()) {
                peak_values.erase(peak_values.begin(), peak_values.begin() + 1);
            }
        }

    }

    void SyncParam::getPeak(const itpp::vec &input, const int start, const int end, double &peak_value, int &peak_ind) {
        std::vector<int> interval;

        for (int i = start; i <= end; ++i) {
            interval.push_back(i);
        }

        // for (const int aa: interval) {
        //     std::cout << aa << " ";
        // }
        // std::cout << std::endl;

        // Filter interval to [0, input_vec.size()-1]
        std::vector<int> valid;
        valid.reserve(interval.size());
        for (int idx : interval) {
            if (idx >= 0 && idx < input.size()) {
                valid.push_back(idx);
            }
        }

        if (!valid.empty()) {
            peak_value = input(valid[0]);
            peak_ind = valid[0];
            for (const int idx : valid) {
                if (input(idx) > peak_value) {
                    peak_value = input(idx);
                    peak_ind = idx;
                }
            }
        } else {
            peak_value = 0.0;
            peak_ind = 0;
        }
    }

    void SyncParam::findSyncInstances() {

        std::vector<int> peak_indices;
        std::vector<double> peak_values;

        findPeaks(peak_indices,peak_values);

        std::cout << peak_values << std::endl;

        while (!peak_indices.empty()) {
            double reliable_peak = 0.0;
            double freq_peak = 0.0;
            findReliablePeak(peak_indices,peak_values, reliable_peak, freq_peak);

            t_coarse.push_back(reliable_peak);
            f_coarse.push_back(freq_peak);
        }
    }

    void SyncParam::evalResultsFl(std::vector<double> &t_sync, std::vector<double> &f_sync) {

        // int time_offset = 0;
        // int freq_offset = 0;
        //
        // std::vector<int> success_sync_ind()

    }

    void SyncParam::symbolSync(const itpp::cvec &input, itpp::vec &M, itpp::vec &angle) const {
        const int corr_diff = config::n_fft * upsample_rate;
        const int corr_len = (config::n_g + config::n_ws / 2)  * upsample_rate;
        const int symbol_bamc = (config::n_fft + config::n_cp) * upsample_rate;

        itpp::cvec P(input.size() - 2 * symbol_bamc);
        itpp::vec R = itpp::zeros(input.size() - 2 * symbol_bamc);

        // %-------------------------
        //         %correlation
        // is performed
        // %-------------------------
        itpp::cvec vec_1 = input.left(input.length() - corr_diff);
        itpp::cvec vec_2 = input.right(input.length() - corr_diff);
        itpp::cvec corr_vec = itpp::elem_mult(vec_2, itpp::conj(vec_1));

        // %--------------------------
        // %iterative calculation of P
        // %--------------------------
        P(0) = itpp::sum(corr_vec.left(corr_len));
        for (int i = 1; i < P.length(); ++i) {
            P(i) = P(i - 1) + corr_vec(corr_len + i - 1) - corr_vec(i - 1);
        }

        // %--------------------------
        // %iterative calculation of R
        // %--------------------------
        itpp::vec cor_fac_vec = itpp::sqr(abs(input));
        R(0) = itpp::sum(cor_fac_vec.left(corr_len)) + itpp::sum(cor_fac_vec.mid(corr_diff, corr_len));
        for (int i = 1; i < R.length(); ++i) {
            R(i) = R(i - 1) + cor_fac_vec(corr_len + i - 1) - cor_fac_vec(i - 1) +
                cor_fac_vec(corr_len + corr_diff + i - 1) - cor_fac_vec(corr_diff + i - 1);
        }
        R = R / 2.0;

        // %-----------------
        // % calculation of M
        // %-----------------
        itpp::vec M_pre = itpp::elem_div(itpp::sqr(itpp::abs(P)), itpp::sqr(R));
        //上限裁剪（clipping）
        for (int i = 0; i < M_pre.size(); ++i) {
            if (M_pre(i) > 2.0) M_pre(i) = 2.0;
        }

        // %-------------------------------
        // % shift the metric half a symbol
        // %-------------------------------
        const int shift = static_cast<int>(std::round(symbol_bamc / 2.0
                                                      + (config::n_g + config::n_ws / 4.0) * upsample_rate));

        itpp::vec  M_shift = concat(itpp::zeros(shift), M_pre);
        itpp::cvec P_shift = concat(itpp::zeros_c(shift), P);

        M = M_shift;
        angle = itpp::angle(P_shift);
    }

    void SyncParam::fineSyncCalc(const itpp::vec &M, const itpp::vec &angle) const {
        const int symbol_bamc = (config::n_fft + config::n_cp) * upsample_rate;

        itpp::vec t_tra = itpp::zeros(t_coarse.size());
        itpp::vec f_tra = itpp::zeros(f_coarse.size());

        for (int i = 0; i < t_tra.size(); ++i) {
            if (t_tra(i) + )
        }
    }

    void SyncParam::coarseSync(const itpp::cvec &input) {
        frameSync(input);

        findSyncInstances();
        // 真实场景下用不到
        //eval_results_fl(t_sync, f_sync);

    }


    void SyncParam::fineSync(const itpp::cvec &input) {
        itpp::vec M;
        itpp::vec angle_P;
        symbolSync(input, M, angle_P);
        fineSyncCalc(M, angle_P);
    }


    void SyncParam::syncCorrelation(const itpp::cvec &input, const int corr_len, const int corr_diff, itpp::vec &M, itpp::vec &angle_metric) {
        const int out_len = input.size() - corr_len - corr_diff;
        itpp::cvec P = itpp::zeros_c(out_len);
        itpp::vec R = itpp::zeros(out_len);

        itpp::cvec vec_1 = input.left(input.length() - corr_diff);
        itpp::cvec vec_2 = input.right(input.length() - corr_diff);

        {
            itpp::cvec corr_vec = itpp::elem_mult(vec_2, itpp::conj(vec_1));
            // iterative calculation of P
            P(0) = itpp::sum(corr_vec.left(corr_len));
            for (int i = 1; i < out_len; i++) {
                P(i) = P(i - 1) + corr_vec(i + corr_len - 1) - corr_vec(i - 1);
            }
        }

        angle_metric = itpp::angle(P);

        {
            itpp::vec cor_fac =  itpp::sqr(abs(input));

            itpp::vec R11 = itpp::zeros(R.length());
            itpp::vec R22 = itpp::zeros(R.length());
            R11(0) = itpp::sum(cor_fac.left(corr_diff)) + itpp::sum(cor_fac.mid(corr_len, corr_diff));
            R22(0) = itpp::sum(cor_fac.mid(corr_diff, corr_len-corr_diff));

            for (int i = 1; i < out_len; i++) {
                R11(i) = R11(i - 1) + cor_fac(i + corr_diff - 1) + cor_fac(i + corr_diff + corr_len - 1) -
                    cor_fac(i - 1) - cor_fac(i + corr_len - 1);
                R22(i) = R22(i - 1) + cor_fac(i + corr_len - 1) - cor_fac(i + corr_diff - 1);
            }

            R = 0.5 * R11 + R22;
        }

        M = itpp::elem_div(itpp::sqr(abs(P)) ,itpp::sqr(R));
    }
}

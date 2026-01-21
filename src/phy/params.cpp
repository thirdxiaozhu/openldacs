//
// Created by 邹嘉旭 on 2025/12/20.
//

#include "phy/params.h"

#include <filesystem>
#include <fstream>
#include <itpp/base/matfunc.h>
#include <itpp/base/specmat.h>

#include "phy/fl.h"

namespace openldacs::phy::params {
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

    void SyncParam::frame_sync(const itpp::cvec &input) {
        const int up_corr_len1 =  upsample_rate * corr_len1;
        const int up_corr_diff1 =  upsample_rate * corr_diff1;
        const int up_corr_len2 =  upsample_rate * corr_len2;
        const int up_corr_diff2 =  upsample_rate * corr_diff2;
        const int sync_offset = (config::n_g + config::n_ws / 4) * upsample_rate;

        itpp::vec pre_M1;
        itpp::vec pre_angle_metric1;
        itpp::vec pre_M2;
        itpp::vec pre_angle_metric2;

        sync_correlation(input, up_corr_len1, up_corr_diff1, pre_M1, pre_angle_metric1);
        sync_correlation(input, up_corr_len2, up_corr_diff2, pre_M2, pre_angle_metric2);

        M1.set_length(pre_M1.length());
        M1.zeros();
        M1.set_subvector(sync_offset, pre_M1.left(pre_M1.length() - sync_offset));

        angle1.set_length(pre_angle_metric1.length());
        angle1.zeros();
        angle1.set_subvector(sync_offset, pre_angle_metric1.left(pre_angle_metric1.length() - sync_offset));

        M2.set_length(pre_M2.length());
        M2.zeros();
        M2.set_subvector(sync_offset, pre_M2.left(pre_M2.length() - sync_offset));

        angle2.set_length(pre_angle_metric2.length());
        angle2.zeros();
        angle2.set_subvector(sync_offset, pre_angle_metric2.left(pre_angle_metric2.length() - sync_offset));

        freq1 = static_cast<double>(config::n_fft * upsample_rate) / corr_diff1 / (2 * M_PI) * angle1;
        freq2 = static_cast<double>(config::n_fft * upsample_rate) / corr_diff2 / (2 * M_PI) * angle2;
    }

    void SyncParam::find_peeks() {

    }

    void SyncParam::find_sync_instances() {
        find_peeks()
    }

    void SyncParam::coarse_sync(const itpp::cvec &input) {

        frame_sync(input);

    }


    void SyncParam::sync_correlation(const itpp::cvec &input, const int corr_len, const int corr_diff, itpp::vec &M, itpp::vec &angle_metric) {
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

        // itpp::vec v = abs(P);
        // std::filesystem::create_directories("dump");
        // std::ofstream ofs("dump/corr_peak.csv");
        // for (int i = 0; i < v.length(); ++i) {
        //     ofs << i << "," << v(i) << "\n";
        // }

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

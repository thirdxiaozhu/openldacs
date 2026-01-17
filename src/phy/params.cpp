//
// Created by 邹嘉旭 on 2025/12/20.
//

#include "phy/params.h"

#include <itpp/base/specmat.h>

#include "phy/fl.h"

namespace openldacs::phy::params {
    CodingParams CodingTable::setCodingParams(CodingKey key) const {
        CodingParams params = get_initial_coding_param(key);

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
        const uint8_t* src = fl::random_output.data();
        params.randomize_vec.assign(
            src,
            src + params.bytes_per_pdu
        );

        return params;
    }

    void CodingTable::initCodingTable(const std::initializer_list<CodingKey> keys) {
        for (const auto& key : keys) {
            coding_table.try_emplace(key, setCodingParams(key));
        }
    }

    void SyncParam::frame_sync(const itpp::cvec &input) {
        int up_corr_len1 =  upsample_rate * corr_len1;
        int up_corr_diff1 =  upsample_rate * corr_diff1;
        int up_corr_len2 =  upsample_rate * corr_len2;
        int up_corr_diff2 =  upsample_rate * corr_diff2;
        int sync_offset = (config::n_g + config::n_ws / 4) * upsample_rate;
        sync_correlation(input, corr_len1, corr_diff1);
    }

    void SyncParam::coarse_sync(const itpp::cvec &input) {
        frame_sync(input);
    }


    void SyncParam::sync_correlation(const itpp::cvec &input, const int corr_len, const int corr_diff) {
        const int out_len = input.size() - corr_len - corr_diff;
        itpp::cvec P = itpp::zeros_c(out_len);
        itpp::cvec R = itpp::zeros_c(out_len);

        itpp::cvec vec_1 = input.left(input.length() - corr_diff);
        itpp::cvec vec_2 = input.right(input.length() - corr_diff);

        itpp::cvec corr_vec = itpp::elem_mult(vec_2, itpp::conj(vec_1));
        SPDLOG_INFO("{} {}", vec_1.length(), vec_2.length());

    }
}

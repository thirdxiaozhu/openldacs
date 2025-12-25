//
// Created by 邹嘉旭 on 2025/12/20.
//

#include "phy/params.h"

namespace openldacs::phy::params {

    CodingParams CodingTable::set_coding_params(CodingKey key) {

        const CodingParams& pre_params = get_specific_coding_params(key);

        CodingParams params = pre_params;
        auto [modulation_type, coding_rate, joint_frame] = key;
        params.modulation_type = modulation_type;

        if (coding_rate == CodingRate::R12) {
            params.a = 1;
            params.b = 2;
            params.coding_rate = 0.5;
            params.puncpat = {1,1};
            params.bits_per_symb = 2;
        }else if (coding_rate == CodingRate::R23) {
            params.a = 2;
            params.b = 3;
            params.coding_rate = 0.67;
            params.puncpat = {1,1,0,1};
            params.bits_per_symb = 4;
        }else if (coding_rate == CodingRate::R34) {
            params.a = 3;
            params.b = 4;
            params.coding_rate = 0.75;
            params.puncpat = {1,1,0,1,1,0};
            params.bits_per_symb = 6;
        }else {
            throw std::invalid_argument("Unsupported rate_cc");
        }

        // constrain length is 7
        itpp::ivec gen(2);
        gen(0) = 0171; // G1 = 171oct
        gen(1) = 0133; // G2 = 133oct

        params.cc.set_generator_polynomials(gen, params.L);                                 // 相当于 poly2trellis 的“码定义”部分
        params.cc.set_method(itpp::Trunc);                                                   // 不自动加尾
        params.cc.set_puncture_matrix(util::puncpat_to_matrix_2output(params.puncpat));

        // bits_bef_cod：你 MATLAB 的输入（已包含手动补的 K-1 个 0）
        // itpp::bvec bits_bef_cod = "1 0 1 1 0 0 1 0 0 0 0 0 0 0"; // 示例：最后是否含 K-1 个 0 取决于你上游
        // itpp::bvec bits_cod;
        // params.cc.encode_trunc(bits_bef_cod, bits_cod);   // === 等价 convenc(bits_bef_cod, trellis, punc_pat) :contentReference[oaicite:5]{index=5}
        // std::cout << bits_cod <<std::endl;

        SPDLOG_INFO("N_symbols: {}; N_bits_per_symbol: {}； N_frame_tile_joint: {}", frame_info.frame_info_.n_data, params.bits_per_symb,joint_frame);
        int bits_coded_frame = frame_info.frame_info_.n_data * params.bits_per_symb * joint_frame;
        SPDLOG_INFO("N_bits_coded_frame: {}; N_cc_cod: {}；", bits_coded_frame, params.cc_cod);
        int bits_with_pad = bits_coded_frame / params.cc_cod;
        SPDLOG_INFO("N_bits_with_pad: {}; b: {}； a: {}", bits_with_pad, params.b, params.a);
        int bits_with_term = std::floor(bits_with_pad / params.b) * params.a;
        SPDLOG_INFO("N_bits_with_term: {}; term_bits: {}", bits_with_term, params.term_bits);
        int bits_before_cc = bits_with_term - params.term_bits;
        int bits_before_cc_frame = bits_before_cc * params.cc_cod;
        SPDLOG_INFO("N_bits_before_cc_frame: {}", bits_before_cc_frame);

        // interleaver
        params.int_size = bits_with_pad * params.interleaver;
        SPDLOG_INFO("int size: {}; N_int: {}", params.int_size, params.interleaver);




        SPDLOG_WARN("\n==================================\n");

        return params;
    }

    void CodingTable::init_coding_table(const std::initializer_list<CodingKey> keys) {
        for (const auto& key : keys) {
            coding_table[key] = set_coding_params(key);
        }
    }


}

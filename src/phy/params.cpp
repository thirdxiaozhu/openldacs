//
// Created by 邹嘉旭 on 2025/12/20.
//

#include "phy/params.h"

namespace openldacs::phy::params {

    CodingParams CodingTable::get_coding_params(ModulationType modulation_type,  CodingRate coding_rate) {
        CodingParams params;
        params.modulation_type = modulation_type;

        if (coding_rate == CodingRate::R12) {
            params.a = 1;
            params.b = 2;
            params.coding_rate = 0.5;
            params.puncpat = {1,1};
        }else if (coding_rate == CodingRate::R23) {
            params.a = 2;
            params.b = 3;
            params.coding_rate = 0.67;
            params.puncpat = {1,1,0,1};
        }else if (coding_rate == CodingRate::R34) {
            params.a = 3;
            params.b = 4;
            params.coding_rate = 0.75;
            params.puncpat = {1,1,0,1,1,0};
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

        SPDLOG_INFO("N_symbols {}; N_bits_per_symbol {}； N_frame_tile_joint {}", 1,2,3 );


        return params;
    }


}

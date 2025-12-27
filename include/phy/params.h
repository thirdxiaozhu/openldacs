//
// Created by 邹嘉旭 on 2025/12/20.
//

#ifndef OPENLDACS_PARAMS_H
#define OPENLDACS_PARAMS_H

#include "openldacs.h"
#include  "link.h"
#include  "util/util.h"

namespace openldacs::phy::params {
    using namespace phy::link;
    using cd = std::complex<double>;
    struct FrameInfo {
        std::vector<int> data_ind;
        std::vector<int> pilot_ind;
        Eigen::MatrixXi frame_pattern;
        size_t n_data = 0;
        size_t n_pilot = 0;
        Eigen::MatrixXi data_ind_packet;
        Eigen::MatrixXi pilot_ind_packet;
        Eigen::MatrixXi sync_ind_packet;

        std::vector<cd> pilot_seeds;
        std::vector<std::vector<cd>> sync_symbols;
    };

    struct ParamStruct {
        FrameInfo frame_info_;
    };

    enum class ModulationType : int { QPSK, QAM16, QAM64, };
    enum class SymbolValue : int { GUARD = 0, DATA = 1, PILOT = 2, };
    enum class CodingRate : int { R12, R23, R34 };
    struct HelicalInterleaverParams {
        int a, b = 0;
        std::vector<int> pattern;
    };

    struct RSCoderParams {
        int n, k;
        int bits_uncoded;
        int bits_after_rs;

        // 添加构造函数
        RSCoderParams() = default;
        RSCoderParams(const int n_val, const int k_val) : n(n_val), k(k_val) {
            bits_uncoded = k_val * 8;      // 假设每个符号8位
            bits_after_rs = n_val * 8;
        }
    };

    struct ConvCodingParams {
        int bits_before_cc;
        int bits_coded;
        int pad_bits_after_rs;
        int pad_bits_after_cc;
    };

    struct CodingParams {
        ModulationType modulation_type;
        double coding_rate;                     //0.5 / 0.67 / 0.75

        int bits_per_symb = 2;
        itpp::Punctured_Convolutional_Code cc;
        std::vector<int> puncpat;               // 0/1 pattern; empty or {0} means "no puncture"
        int int_size = 1;

        double rate_cod;

        // 固定参数
        int L = 7;                              // constraint length
        int term_bits = 6;                      // L-1
        double rate_rs = 0.9;

        // 初始参数
        int a = 0, b = 0;                       // a / b
        HelicalInterleaverParams h_inter_params;
        RSCoderParams rs_params;
        ConvCodingParams conv_params;

        // 当前是否有意义？
        int cc_cod = 1;
        int interleaver = 1;
    };


    using CodingKey = std::tuple<ModulationType, CodingRate, int>;


    struct CodingTable {
        std::map<CodingKey, CodingParams> coding_table;
        ParamStruct& frame_info;

        CodingTable(ParamStruct &frame_info) : frame_info(frame_info) {}

        void set_coding_params(CodingKey key, CodingParams &params) const;
        void init_coding_table(std::initializer_list<CodingKey> keys);
    };

    static const std::array<std::pair<CodingKey, CodingParams>, 2> init_coding_param_pairs = {
        {
            {{ModulationType::QPSK, CodingRate::R12, 2}, CodingParams{.h_inter_params = {132, 74}, .rs_params = {101, 91}}},
            {{ModulationType::QPSK, CodingRate::R12, 3}, CodingParams{.h_inter_params = {111, 132}, .rs_params = {101, 91}}}
        }
    };

    inline void get_initial_coding_param(const CodingKey &key, CodingParams& params) {
        bool is_found = false;
        for (const auto &[fst, snd] : init_coding_param_pairs) {
            if (fst == key) {
                params = {snd};
                is_found = true;
                break;

            }
        }
        if (!is_found) {
            throw std::runtime_error("No such coding params");
        }

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
    }


}

#endif //OPENLDACS_PARAMS_H
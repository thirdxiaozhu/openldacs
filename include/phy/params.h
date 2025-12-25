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
    struct HelicalInterleaver {
        int a, b = 0;
    };

    struct CodingParams {
        ModulationType modulation_type;
        double coding_rate;                     //0.5 / 0.67 / 0.75

        int bits_per_symb = 2;
        itpp::Punctured_Convolutional_Code cc;
        std::vector<int> puncpat;               // 0/1 pattern; empty or {0} means "no puncture"
        int int_size = 1;

        // 固定参数
        int L = 7;                              // constraint length
        int term_bits = 6;                      // L-1
        double rate_rs = 0.9;

        // 初始参数
        HelicalInterleaver h_inter_params;
        int a = 0, b = 0;                       // a / b

        // 当前是否有意义？
        int cc_cod = 1;
        int interleaver = 1;
    };


    using CodingKey = std::tuple<ModulationType, CodingRate, int>;


    struct CodingTable {
        std::map<CodingKey, CodingParams> coding_table;
        ParamStruct& frame_info;

        CodingTable(ParamStruct &frame_info) : frame_info(frame_info) {}

        CodingParams set_coding_params(CodingKey key);
        void init_coding_table(std::initializer_list<CodingKey> keys);
    };

    static const std::array<std::pair<CodingKey, CodingParams>, 2> coding_params = {
        {
            {{ModulationType::QPSK, CodingRate::R12, 2}, CodingParams{.h_inter_params = {132, 74}, .a=1, .b=2}},
            {{ModulationType::QPSK, CodingRate::R12, 3}, CodingParams{.h_inter_params = {111, 132}, .a=1, .b=2}}
        }
    };

    inline const CodingParams &get_specific_coding_params(const CodingKey &key) {
        for (const auto &[fst, snd] : coding_params) {
            if (fst == key) {
                return snd;
            }
        }

        throw std::runtime_error("No such coding params");
    }


}

#endif //OPENLDACS_PARAMS_H